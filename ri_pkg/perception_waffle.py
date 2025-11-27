import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import cv2
import math
import numpy as np
import json
import os
from datetime import datetime
import gc  # For garbage collection

# --- VLM IMPORTS ---
import torchvision.transforms as T
from torchvision.transforms.functional import InterpolationMode
from PIL import Image
from transformers import AutoTokenizer, AutoModel

# --- CONFIGURATION ---
YOLO_MODEL_PATH = "yolov8s.pt"
VLM_MODEL_PATH = "OpenGVLab/InternVL3_5-1B-Instruct"

# --- INTERNVL PREPROCESSING HELPERS ---
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

# --- MAIN CLASS ---

class YoloClipCameraNode(Node):
    def __init__(self):
        super().__init__('yolo_vlm_camera_node')
        self.bridge = CvBridge()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Running on device: {self.device}")

        # Robot state variables
        self.robot_pose = None
        self.lidar_data = None
        
        # Object storage
        self.detected_objects_db = []
        self.next_object_id = 1
        
        # Get the package directory path for proper file location
        PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.objects_file_path = os.path.join(PACKAGE_DIR, "detected_objects_map.json")
        self.coordinate_threshold = 1  # meters for duplicate detection
        
        # Load existing objects if file exists
        self.load_objects_from_file()

        # --- LOAD YOLO ---
        self.get_logger().info(f"Loading YOLO: {YOLO_MODEL_PATH}...")
        self.yolo_model = YOLO(YOLO_MODEL_PATH)
        self.yolo_model.to(self.device)

        # --- LOAD INTERNVL 3.5 ---
        self.get_logger().info(f"Loading InternVL 3.5: {VLM_MODEL_PATH}...")
        try:
            self.vlm_model = AutoModel.from_pretrained(
                VLM_MODEL_PATH,
                torch_dtype=torch.bfloat16,
                low_cpu_mem_usage=True,
                use_flash_attn=False,
                trust_remote_code=True
            ).eval().to(self.device)

            self.vlm_tokenizer = AutoTokenizer.from_pretrained(
                VLM_MODEL_PATH, 
                trust_remote_code=True, 
                use_fast=False
            )
            self.get_logger().info("InternVL Loaded Successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load VLM: {e}")
            self.vlm_model = None

        # Subscriptions
        self.image_subscription = self.create_subscription(
            CompressedImage, 
            '/camera/image_raw/compressed', 
            self.compressed_image_callback, 
            5
        )
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 5)
        
        # Set QoS for LiDAR to BEST_EFFORT to match publisher
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, lidar_qos)
        
        # Publishers
        self.detection_publisher = self.create_publisher(String, '/yolo/detection_result', 10)

    # --- ODOM & LIDAR CALLBACKS (UNCHANGED) ---
    def odom_callback(self, msg):
        self.robot_pose = {
            'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y, 'z': msg.pose.pose.position.z,
            'orientation': {
                'x': msg.pose.pose.orientation.x, 'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z, 'w': msg.pose.pose.orientation.w
            }
        }

    def lidar_callback(self, msg):
        self.lidar_data = {
            'ranges': list(msg.ranges), 'angle_min': msg.angle_min, 'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment, 'range_min': msg.range_min, 'range_max': msg.range_max
        }

    def quaternion_to_yaw(self, orientation):
        x, y, z, w = orientation['x'], orientation['y'], orientation['z'], orientation['w']
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def calculate_object_global_coordinates(self, lidar_data):
        if self.robot_pose is None or lidar_data is None or lidar_data['distance'] is None:
            return None
        robot_x = self.robot_pose['x']
        robot_y = self.robot_pose['y']
        robot_yaw = self.quaternion_to_yaw(self.robot_pose['orientation'])
        object_angle_rad = lidar_data['angle_rad']
        object_distance = lidar_data['distance']
        global_angle = robot_yaw + object_angle_rad
        return {
            'x': robot_x + object_distance * math.cos(global_angle),
            'y': robot_y + object_distance * math.sin(global_angle),
            'robot_yaw_deg': math.degrees(robot_yaw),
            'global_angle_deg': math.degrees(global_angle)
        }

    def get_object_distance_from_bbox(self, bbox_left_x, bbox_right_x, image_width, num_samples=5, outlier_filter=True):
        if self.lidar_data is None: return None
        ranges = np.array(self.lidar_data['ranges'], dtype=float)
        ranges[~np.isfinite(ranges)] = np.nan
        bbox_width = bbox_right_x - bbox_left_x
        if bbox_width <= 0: sample_x_positions = [bbox_left_x]
        else: sample_x_positions = np.linspace(bbox_left_x, bbox_right_x, num_samples)

        distance_measurements = []
        image_center_x = float(image_width) / 2.0
        
        for sample_x in sample_x_positions:
            normalized_x = max(-1.0, min(1.0, (float(sample_x) - image_center_x) / image_center_x))
            angle_rad = math.radians((-30.0 * normalized_x) % 360.0)
            idx = int(round((angle_rad - self.lidar_data['angle_min']) / self.lidar_data['angle_increment'])) % ranges.size
            if np.isfinite(ranges[idx]): distance_measurements.append(float(ranges[idx]))

        if not distance_measurements: return None
        
        if outlier_filter and len(distance_measurements) > 2:
            sorted_distances = sorted(distance_measurements)
            if sorted_distances[-1] > np.median(sorted_distances[:-1]) * 1.3:
                final_distance = float(np.mean(sorted_distances[:-1]))
            else:
                final_distance = float(np.mean(sorted_distances))
        else:
            final_distance = float(np.mean(distance_measurements))

        normalized_center_x = max(-1.0, min(1.0, (((bbox_left_x + bbox_right_x)/2.0) - image_center_x) / image_center_x))
        return {
            'distance': final_distance, 'angle_rad': math.radians((-30.0 * normalized_center_x) % 360.0),
            'angle_deg': (-30.0 * normalized_center_x) % 360.0, 'bbox_center_x': (bbox_left_x + bbox_right_x) / 2.0
        }

    # --- DATABASE & VLM LOGIC ---

    def load_objects_from_file(self):
        if os.path.exists(self.objects_file_path):
            try:
                with open(self.objects_file_path, 'r') as f: self.detected_objects_db = json.load(f)
                print(f"Loaded {len(self.detected_objects_db)} objects from DB.")
            except Exception as e:
                print(f"Failed to load objects: {e}"); self.detected_objects_db = []
        else: self.detected_objects_db = []
        
        if self.detected_objects_db:
            self.next_object_id = max(obj.get('id', 0) for obj in self.detected_objects_db) + 1
        else: self.next_object_id = 1

    def save_objects_to_file(self):
        try:
            os.makedirs(os.path.dirname(self.objects_file_path), exist_ok=True)
            with open(self.objects_file_path, 'w') as f: json.dump(self.detected_objects_db, f, indent=2)
        except Exception as e: print(f"Error saving objects: {e}")

    def is_duplicate_location(self, global_coords):
        """
        Check if object exists based on location.
        Returns: (is_duplicate, id, label, confidence)
        """
        for obj in self.detected_objects_db:
            stored_coords = obj['world_coordinates']
            distance = math.sqrt((global_coords['x'] - stored_coords['x'])**2 + (global_coords['y'] - stored_coords['y'])**2)
            if distance <= self.coordinate_threshold:
                return True, obj['id'], obj['label'], obj['confidence']
        return False, None, None, None

    def run_vlm_inference(self, cv2_image, bbox):
        """
        Runs InternVL 3.5 Inference with memory cleanup.
        """
        if self.vlm_model is None: return "VLM_Error", 0.0

        x1, y1, x2, y2 = map(int, bbox)
        h, w, _ = cv2_image.shape
        x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
        if x2 <= x1 or y2 <= y1: return "Invalid_Crop", 0.0
        
        crop = cv2_image[y1:y2, x1:x2]
        pil_image = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
        pixel_values = None

        try:
            images = dynamic_preprocess(pil_image, image_size=448, use_thumbnail=True, max_num=12)
            transform = build_transform(input_size=448)
            pixel_values = [transform(image) for image in images]
            pixel_values = torch.stack(pixel_values).to(torch.bfloat16).to(self.device)

            question = '<image>\nIdentify the main object in this image. Provide a short Name and a Confidence Score (0.0-1.0). Format: Name, Score'
            generation_config = dict(max_new_tokens=128, do_sample=False)

            response = self.vlm_model.chat(self.vlm_tokenizer, pixel_values, question, generation_config)
            
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
            if pixel_values is not None: del pixel_values
            gc.collect()
            torch.cuda.empty_cache()

    def add_object_with_vlm(self, frame, bbox, global_coords, lidar_data, yolo_label):
        print(f"--> NEW OBJECT at ({global_coords['x']:.1f}, {global_coords['y']:.1f}). Running VLM...")
        vlm_name, vlm_conf = self.run_vlm_inference(frame, bbox)
        print(f"--> VLM RESULT: {vlm_name} ({vlm_conf})")

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

    def publish_detection_results(self, detected_objects):
        """Modified to include VLM Labels"""
        detection_msg = String()
        
        if detected_objects:
            best = max(detected_objects, key=lambda obj: obj['confidence'])
            label = best.get('vlm_label', best['label'])
            conf = best.get('vlm_conf', best['confidence'])
            bbox = best['bbox']
            bbox_str = f"{bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}"
            
            if best['global_coords'] and best['lidar_data']:
                gc = best['global_coords']
                ld = best['lidar_data']
                detection_msg.data = f"{label},{conf:.4f},{gc['x']:.2f},{gc['y']:.2f},{ld['distance']:.2f},{ld['angle_rad']:.2f},{bbox_str}"
            else:
                detection_msg.data = f"{label},{conf:.4f},None,None,None,None,{bbox_str}"
        else:
            detection_msg.data = "None,0.0,None,None,None,None,None,None,None,None"
        
        self.detection_publisher.publish(detection_msg)

    # --- MAIN LOOP ---
    def compressed_image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e: return
        self.process_frame(frame)

    def process_frame(self, frame):
        results = self.yolo_model(frame, verbose=False)
        annotated = frame.copy()
        h, w = frame.shape[:2]
        
        current_frame_detections = []

        for r in results:
            boxes = r.boxes
            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    b = box.xyxy[0].int().cpu().numpy()
                    conf = float(box.conf[0])
                    yolo_label = self.yolo_model.names[int(box.cls[0])]
                    
                    lidar_data = self.get_object_distance_from_bbox(float(b[0]), float(b[2]), w)
                    global_coords = self.calculate_object_global_coordinates(lidar_data)
                    
                    vlm_label = None
                    vlm_score = None

                    # --- LOGIC TRIGGER ---
                    if global_coords:
                        is_dup, _, exist_lbl, exist_conf = self.is_duplicate_location(global_coords)
                        if is_dup:
                            vlm_label = exist_lbl
                            vlm_score = exist_conf
                        else:
                            _, vlm_label, vlm_score = self.add_object_with_vlm(frame, b, global_coords, lidar_data, yolo_label)

                    # --- VISUALIZATION ---
                    cv2.rectangle(annotated, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                    
                    # Determine what text to show (VLM Orange vs YOLO Green)
                    if vlm_label:
                        disp_text = f"{vlm_label} ({vlm_score:.2f})"
                        cv2.putText(annotated, disp_text, (b[0], b[1]-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                    else:
                        disp_text = f"{yolo_label} {conf:.2f}"
                        cv2.putText(annotated, disp_text, (b[0], b[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    if global_coords:
                         coord_text = f"X:{global_coords['x']:.1f} Y:{global_coords['y']:.1f}"
                         cv2.putText(annotated, coord_text, (b[0], b[1]-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                    current_frame_detections.append({
                        'label': yolo_label,
                        'vlm_label': vlm_label if vlm_label else yolo_label,
                        'vlm_conf': vlm_score if vlm_score else conf,
                        'confidence': conf,
                        'bbox': b,
                        'lidar_data': lidar_data,
                        'global_coords': global_coords
                    })

        self.publish_detection_results(current_frame_detections)
        cv2.imshow("YOLO + InternVL 3.5 Perception", annotated)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YoloClipCameraNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()