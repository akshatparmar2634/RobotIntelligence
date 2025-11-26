import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import torchvision.transforms as T
from torchvision.transforms.functional import InterpolationMode
from PIL import Image
from transformers import AutoTokenizer, AutoModel
import cv2
import math
import numpy as np
import json
import os
from datetime import datetime
import gc 

# --- CONFIGURATION ---
YOLO_MODEL_PATH = "yolov10s.pt"
# UPDATED: Using the latest InternVL 3.5 1B Instruct model
VLM_MODEL_PATH = "OpenGVLab/InternVL3_5-1B-Instruct"

# --- INTERNVL 3.5 PREPROCESSING HELPERS ---
# These helper functions are required to handle the "Dynamic High Resolution" 
# tiling strategy used by InternVL models.
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)

def build_transform(input_size):
    MEAN, STD = IMAGENET_MEAN, IMAGENET_STD
    transform = T.Compose([
        T.Lambda(lambda img: img.convert('RGB') if img.mode != 'RGB' else img),
        T.Resize((input_size, input_size), interpolation=InterpolationMode.BICUBIC),
        T.ToTensor(),
        T.Normalize(mean=MEAN, std=STD)
    ])
    return transform

def find_closest_aspect_ratio(aspect_ratio, target_ratios, width, height, image_size):
    best_ratio_diff = float('inf')
    best_ratio = (1, 1)
    area = width * height
    for ratio in target_ratios:
        target_aspect_ratio = ratio[0] / ratio[1]
        ratio_diff = abs(aspect_ratio - target_aspect_ratio)
        if ratio_diff < best_ratio_diff:
            best_ratio_diff = ratio_diff
            best_ratio = ratio
        elif ratio_diff == best_ratio_diff:
            if area > 0.5 * image_size * image_size * ratio[0] * ratio[1]:
                best_ratio = ratio
    return best_ratio

def dynamic_preprocess(image, min_num=1, max_num=12, image_size=448, use_thumbnail=True):
    orig_width, orig_height = image.size
    aspect_ratio = orig_width / orig_height
    target_ratios = set(
        (i, j) for n in range(min_num, max_num + 1)
        for i in range(1, n + 1) for j in range(1, n + 1)
        if i * j <= max_num and i * j >= min_num)
    target_ratios = sorted(target_ratios, key=lambda x: x[0] * x[1])
    target_aspect_ratio = find_closest_aspect_ratio(
        aspect_ratio, target_ratios, orig_width, orig_height, image_size)
    target_width = image_size * target_aspect_ratio[0]
    target_height = image_size * target_aspect_ratio[1]
    blocks = target_aspect_ratio[0] * target_aspect_ratio[1]
    resized_img = image.resize((target_width, target_height))
    processed_images = []
    for i in range(blocks):
        box = (
            (i % (target_width // image_size)) * image_size,
            (i // (target_width // image_size)) * image_size,
            ((i % (target_width // image_size)) + 1) * image_size,
            ((i // (target_width // image_size)) + 1) * image_size
        )
        split_img = resized_img.crop(box)
        processed_images.append(split_img)
    if use_thumbnail and len(processed_images) > 1:
        thumbnail_img = image.resize((image_size, image_size))
        processed_images.append(thumbnail_img)
    return processed_images

# --- MAIN ROS 2 NODE ---

class InternVL3Node(Node):
    def __init__(self):
        super().__init__('internvl3_perception_node')
        self.bridge = CvBridge()
        
        # 1. Hardware Setup
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Running on device: {self.device}")

        # 2. Load YOLO (Object Detector)
        self.get_logger().info(f"Loading YOLO: {YOLO_MODEL_PATH}...")
        try:
            self.yolo_model = YOLO(YOLO_MODEL_PATH)
            self.yolo_model.to(self.device)
        except Exception as e:
            self.get_logger().error(f"Error loading YOLO: {e}")

        # 3. Load InternVL 3.5 (VLM)
        self.get_logger().info(f"Loading InternVL 3.5: {VLM_MODEL_PATH}...")
        try:
            # Load the model with custom code trust enabled
            self.vlm_model = AutoModel.from_pretrained(
                VLM_MODEL_PATH,
                torch_dtype=torch.bfloat16, # Efficient precision for 6GB VRAM
                low_cpu_mem_usage=True,
                use_flash_attn=False,       # Keep False to avoid dependency headaches
                trust_remote_code=True
            ).eval().to(self.device)

            self.vlm_tokenizer = AutoTokenizer.from_pretrained(
                VLM_MODEL_PATH, 
                trust_remote_code=True, 
                use_fast=False
            )
            
            self.get_logger().info("InternVL 3.5 Loaded Successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load InternVL 3.5: {e}")
            self.vlm_model = None

        # Robot State & Storage
        self.robot_pose = None
        self.lidar_data = None
        self.detected_objects_db = []
        self.next_object_id = 1
        
        # File Config
        PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.objects_file_path = os.path.join(PACKAGE_DIR, "detected_objects_map.json")
        self.coordinate_threshold = 1.0 
        
        self.load_objects_from_file()

        # ROS 2 Communication
        self.image_subscription = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.compressed_image_callback, 1)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        
        # Publisher
        self.detection_publisher = self.create_publisher(String, '/vlm/detection_result', 10)

    # --- SENSOR CALLBACKS ---
    def odom_callback(self, msg):
        self.robot_pose = {
            'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'orientation': {
                'x': msg.pose.pose.orientation.x, 'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z, 'w': msg.pose.pose.orientation.w
            }
        }

    def lidar_callback(self, msg):
        self.lidar_data = {
            'ranges': list(msg.ranges), 'angle_min': msg.angle_min,
            'angle_max': msg.angle_max, 'angle_increment': msg.angle_increment,
            'range_min': msg.range_min, 'range_max': msg.range_max
        }

    # --- MATH UTILS ---
    def quaternion_to_yaw(self, orientation):
        x, y, z, w = orientation['x'], orientation['y'], orientation['z'], orientation['w']
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    
    def calculate_object_global_coordinates(self, lidar_data):
        if self.robot_pose is None or lidar_data is None or lidar_data['distance'] is None:
            return None
        rx, ry = self.robot_pose['x'], self.robot_pose['y']
        ryaw = self.quaternion_to_yaw(self.robot_pose['orientation'])
        g_angle = ryaw + lidar_data['angle_rad']
        return {
            'x': rx + lidar_data['distance'] * math.cos(g_angle),
            'y': ry + lidar_data['distance'] * math.sin(g_angle),
            'robot_yaw_deg': math.degrees(ryaw),
            'global_angle_deg': math.degrees(g_angle)
        }

    def get_object_distance_from_bbox(self, bbox_left_x, bbox_right_x, image_width, num_samples=5, outlier_filter=True):
        if self.lidar_data is None: return None
        ranges = np.array(self.lidar_data['ranges'], dtype=float)
        ranges[~np.isfinite(ranges)] = np.nan
        
        bbox_width = bbox_right_x - bbox_left_x
        if bbox_width <= 0: sample_x = [bbox_left_x]
        else: sample_x = np.linspace(bbox_left_x, bbox_right_x, num_samples)

        dists = []
        cx = float(image_width) / 2.0
        for x in sample_x:
            nx = max(-1.0, min(1.0, (float(x) - cx) / cx))
            angle = math.radians((-30.0 * nx) % 360.0)
            idx = int(round((angle - self.lidar_data['angle_min']) / self.lidar_data['angle_increment'])) % ranges.size
            if np.isfinite(ranges[idx]): dists.append(float(ranges[idx]))

        if not dists: return None
        
        # Outlier Filter
        if outlier_filter and len(dists) > 2:
            s_dists = sorted(dists)
            final_dist = float(np.mean(s_dists[:-1])) if s_dists[-1] > np.median(s_dists[:-1])*1.3 else float(np.mean(s_dists))
        else:
            final_dist = float(np.mean(dists))

        norm_c = max(-1.0, min(1.0, (((bbox_left_x + bbox_right_x)/2.0) - cx) / cx))
        return {
            'distance': final_dist, 'angle_rad': math.radians((-30.0 * norm_c) % 360.0),
            'angle_deg': (-30.0 * norm_c) % 360.0, 'bbox_center_x': (bbox_left_x + bbox_right_x) / 2.0
        }

    # --- DATABASE LOGIC ---
    def load_objects_from_file(self):
        if os.path.exists(self.objects_file_path):
            try:
                with open(self.objects_file_path, 'r') as f: self.detected_objects_db = json.load(f)
                print(f"Loaded {len(self.detected_objects_db)} objects.")
            except: self.detected_objects_db = []
        else: self.detected_objects_db = []
        self.next_object_id = (max(obj.get('id', 0) for obj in self.detected_objects_db) + 1) if self.detected_objects_db else 1

    def save_objects_to_file(self):
        try:
            os.makedirs(os.path.dirname(self.objects_file_path), exist_ok=True)
            with open(self.objects_file_path, 'w') as f: json.dump(self.detected_objects_db, f, indent=2)
        except Exception as e: print(f"Error saving DB: {e}")

    def is_duplicate_location(self, global_coords):
        for obj in self.detected_objects_db:
            d = math.sqrt((global_coords['x'] - obj['world_coordinates']['x'])**2 + 
                          (global_coords['y'] - obj['world_coordinates']['y'])**2)
            if d <= self.coordinate_threshold:
                return True, obj['id'], obj['label'], obj['confidence']
        return False, None, None, None

    # --- CORE VLM PIPELINE ---
    def run_vlm_inference(self, cv2_image, bbox):
        """
        Runs InternVL 3.5 Inference on the cropped object with aggressive memory cleanup.
        """
        if self.vlm_model is None: return "VLM_Error", 0.0

        # 1. Crop Image
        x1, y1, x2, y2 = map(int, bbox)
        h, w, _ = cv2_image.shape
        x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
        if x2 <= x1 or y2 <= y1: return "Invalid_Crop", 0.0
        
        crop = cv2_image[y1:y2, x1:x2]
        pil_image = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
        
        # Initialize variables to None so we can safely check them in 'finally'
        pixel_values = None

        try:
            # 2. Dynamic Tiling (Standard InternVL Preprocessing)
            images = dynamic_preprocess(pil_image, image_size=448, use_thumbnail=True, max_num=12)
            
            # 3. Transform & Stack
            transform = build_transform(input_size=448)
            pixel_values = [transform(image) for image in images]
            pixel_values = torch.stack(pixel_values).to(torch.bfloat16).to(self.device)

            # 4. Prompting
            question = '<image>\nIdentify the main object in this image. Provide a short Name and a Confidence Score (0.0-1.0). Format: Name, Score'
            generation_config = dict(max_new_tokens=128, do_sample=False)

            # 5. Inference
            response = self.vlm_model.chat(
                self.vlm_tokenizer, 
                pixel_values, 
                question, 
                generation_config
            )
            
            # 6. Parse Output
            if "," in response:
                parts = response.split(",")
                name = parts[0].strip()
                try: score = float(parts[1].strip())
                except: score = 0.5
                return name, score
            else:
                return response.strip(), 0.8

        except Exception as e:
            self.get_logger().error(f"InternVL Inference Failed: {e}")
            return "Inference_Failed", 0.0

        finally:
            # --- MEMORY CLEANUP START ---
            # 1. Delete the heavy tensor holding the images on GPU
            if pixel_values is not None:
                del pixel_values
            
            # 2. Force Python's garbage collector to release references
            gc.collect()
            
            # 3. Force PyTorch to release cached memory back to the GPU allocator
            torch.cuda.empty_cache()
            # --- MEMORY CLEANUP END ---

    def add_object_with_vlm(self, frame, bbox, global_coords, lidar_data, yolo_label):
        print(f"--> NEW LOCATION ({global_coords['x']:.1f}, {global_coords['y']:.1f}). Triggering InternVL 3.5...")
        
        vlm_name, vlm_conf = self.run_vlm_inference(frame, bbox)
        print(f"--> RESULT: {vlm_name} ({vlm_conf})")

        new_object = {
            "id": self.next_object_id,
            "label": vlm_name,
            "yolo_hint": yolo_label,
            "confidence": vlm_conf,
            "world_coordinates": {
                "x": round(global_coords['x'], 2),
                "y": round(global_coords['y'], 2),
                "distance": round(lidar_data['distance'], 2),
                "angle": round(lidar_data['angle_rad'], 2),
            },
            "timestamp": datetime.now().isoformat()
        }
        
        self.detected_objects_db.append(new_object)
        self.next_object_id += 1
        self.save_objects_to_file()
        return True, vlm_name, vlm_conf

    # --- MAIN LOOP ---
    def compressed_image_callback(self, msg):
        try: frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except: return
        self.process_frame(frame)

    def process_frame(self, frame):
        # 1. Fast Detection with YOLO
        results = self.yolo_model(frame, verbose=False)
        annotated = frame.copy()
        h, w = frame.shape[:2]
        
        detections_msg_list = []

        for r in results:
            boxes = r.boxes
            if boxes is not None:
                for box in boxes:
                    b = box.xyxy[0].int().cpu().numpy()
                    conf = float(box.conf[0])
                    yolo_lbl = self.yolo_model.names[int(box.cls[0])]
                    
                    # 2. Get 3D Info
                    lidar_info = self.get_object_distance_from_bbox(float(b[0]), float(b[2]), w)
                    g_coords = self.calculate_object_global_coordinates(lidar_info)
                    
                    vlm_lbl, vlm_scr = None, None

                    # 3. Intelligent Trigger
                    if g_coords:
                        is_dup, _, ex_lbl, ex_conf = self.is_duplicate_location(g_coords)
                        if is_dup:
                            vlm_lbl, vlm_scr = ex_lbl, ex_conf
                        else:
                            _, vlm_lbl, vlm_scr = self.add_object_with_vlm(frame, b, g_coords, lidar_info, yolo_lbl)

                    # 4. Visualize
                    cv2.rectangle(annotated, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                    
                    final_lbl = f"{vlm_lbl if vlm_lbl else yolo_lbl}"
                    if vlm_scr: final_lbl += f" ({vlm_scr:.2f})"
                    
                    # Color: Orange for VLM confirmed, Green for YOLO raw
                    color = (0, 165, 255) if vlm_lbl else (0, 255, 0)
                    cv2.putText(annotated, final_lbl, (b[0], b[1]-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    
                    if g_coords:
                        coords_txt = f"X:{g_coords['x']:.1f} Y:{g_coords['y']:.1f}"
                        cv2.putText(annotated, coords_txt, (b[0], b[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    # Prepare ROS Msg
                    bbox_str = f"{b[0]},{b[1]},{b[2]},{b[3]}"
                    if g_coords:
                        detections_msg_list.append(f"{final_lbl},{vlm_scr if vlm_scr else conf:.2f},{g_coords['x']:.2f},{g_coords['y']:.2f},{lidar_info['distance']:.2f},{bbox_str}")
                    else:
                        detections_msg_list.append(f"{final_lbl},{vlm_scr if vlm_scr else conf:.2f},None,None,None,{bbox_str}")

        # Publish
        msg = String()
        msg.data = ";".join(detections_msg_list) if detections_msg_list else "None"
        self.detection_publisher.publish(msg)
        
        cv2.imshow("InternVL 3.5 Perception", annotated)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = InternVL3Node()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()