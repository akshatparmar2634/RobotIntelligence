import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
from transformers import AutoProcessor, AutoModelForImageTextToText
from PIL import Image
import cv2
import math
import numpy as np
import json
import os
from datetime import datetime

# --- CONFIGURATION ---
YOLO_MODEL_PATH = "yolov10s.pt"  # Changing to v10s as requested
VLM_MODEL_ID = "HuggingFaceTB/SmolVLM-500M-Instruct"

class VLMClipCameraNode(Node):
    def __init__(self):
        super().__init__('vlm_perception_node')
        self.bridge = CvBridge()
        
        # Hardware Check
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Running on device: {self.device}")

        # --- LOAD YOLO ---
        self.get_logger().info(f"Loading YOLO: {YOLO_MODEL_PATH}...")
        try:
            self.yolo_model = YOLO(YOLO_MODEL_PATH)
            self.yolo_model.to(self.device)
        except Exception as e:
            self.get_logger().error(f"Error loading YOLO: {e}. Make sure {YOLO_MODEL_PATH} exists.")

        # --- LOAD VLM (SmolVLM) ---
        self.get_logger().info(f"Loading VLM: {VLM_MODEL_ID}...")
        try:
            self.processor = AutoProcessor.from_pretrained(VLM_MODEL_ID)
            
            # Using AutoModelForImageTextToText and 'eager' attention to fix the FlashAttn error
            self.vlm_model = AutoModelForImageTextToText.from_pretrained(
                VLM_MODEL_ID,
                torch_dtype=torch.float16 if self.device == "cuda" else torch.float32,
                _attn_implementation="eager", 
            ).to(self.device)
            
            self.get_logger().info("VLM Loaded Successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load VLM: {e}")
            self.vlm_model = None

        # Robot state variables
        self.robot_pose = None
        self.lidar_data = None
        
        # Object storage
        self.detected_objects_db = []
        self.next_object_id = 1
        
        # File paths
        PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.objects_file_path = os.path.join(PACKAGE_DIR, "detected_objects_map.json")
        self.coordinate_threshold = 1.0  # meters for duplicate detection
        
        self.load_objects_from_file()

        # ROS 2 Subscriptions
        self.image_subscription = self.create_subscription(
            CompressedImage, 
            '/camera/image_raw/compressed', 
            self.compressed_image_callback, 
            1
        )
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)
        
        # ROS 2 Publisher (Updated topic)
        self.detection_publisher = self.create_publisher(String, '/vlm/detection_result', 10)


    def odom_callback(self, msg):
        """Callback for robot odometry data"""
        self.robot_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            }
        }

    def lidar_callback(self, msg):
        """Callback for LiDAR scan data"""
        self.lidar_data = {
            'ranges': list(msg.ranges),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max
        }

    def quaternion_to_yaw(self, orientation):
        """Convert quaternion to yaw angle in radians"""
        x, y, z, w = orientation['x'], orientation['y'], orientation['z'], orientation['w']
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def calculate_object_global_coordinates(self, lidar_data):
        """Calculate global coordinates of object using robot pose and LiDAR data"""
        if self.robot_pose is None or lidar_data is None or lidar_data['distance'] is None:
            return None
        
        robot_x = self.robot_pose['x']
        robot_y = self.robot_pose['y']
        robot_yaw = self.quaternion_to_yaw(self.robot_pose['orientation'])
        
        object_angle_rad = lidar_data['angle_rad']
        object_distance = lidar_data['distance']
        
        global_angle = robot_yaw + object_angle_rad
        
        object_global_x = robot_x + object_distance * math.cos(global_angle)
        object_global_y = robot_y + object_distance * math.sin(global_angle)
        
        return {
            'x': object_global_x,
            'y': object_global_y,
            'robot_yaw_deg': math.degrees(robot_yaw),
            'global_angle_deg': math.degrees(global_angle)
        }

    def get_object_distance_from_bbox(self, bbox_left_x, bbox_right_x, image_width,
                                    num_samples=5, outlier_filter=True):
        """
        Your custom logic for distance calculation
        """
        if self.lidar_data is None:
            return None

        ranges = np.array(self.lidar_data['ranges'], dtype=float)
        angle_min = float(self.lidar_data['angle_min'])
        angle_inc = float(self.lidar_data['angle_increment'])
        n = ranges.size

        ranges[~np.isfinite(ranges)] = np.nan

        bbox_width = bbox_right_x - bbox_left_x
        if bbox_width <= 0:
            sample_x_positions = [bbox_left_x]
        else:
            sample_x_positions = np.linspace(bbox_left_x, bbox_right_x, num_samples)

        distance_measurements = []
        angle_measurements = []
        lidar_indices = []
        
        image_center_x = float(image_width) / 2.0
        if image_center_x == 0:
            return None # Safety check

        for sample_x in sample_x_positions:
            normalized_x = (float(sample_x) - image_center_x) / image_center_x
            normalized_x = max(-1.0, min(1.0, normalized_x))
            angle_deg = -30.0 * normalized_x
            angle_deg = angle_deg % 360.0
            angle_rad = math.radians(angle_deg)
            angle_measurements.append(angle_deg)

            idx_float = (angle_rad - angle_min) / angle_inc
            idx = int(round(idx_float)) % n
            lidar_indices.append(idx)

            if np.isfinite(ranges[idx]):
                distance_measurements.append(float(ranges[idx]))
            else:
                distance_measurements.append(None)

        valid_distances = [d for d in distance_measurements if d is not None]
        
        if len(valid_distances) == 0:
            final_distance = None
        elif outlier_filter and len(valid_distances) > 2:
            sorted_distances = sorted(valid_distances)
            if len(sorted_distances) >= 3:
                median_val = np.median(sorted_distances[:-1]) 
                max_val = sorted_distances[-1]
                if max_val > median_val * 1.3:
                    filtered_distances = sorted_distances[:-1]
                else:
                    filtered_distances = sorted_distances
            else:
                filtered_distances = sorted_distances
            final_distance = float(np.mean(filtered_distances))
        else:
            final_distance = float(np.mean(valid_distances))

        bbox_center_x = (bbox_left_x + bbox_right_x) / 2.0
        normalized_center_x = (bbox_center_x - image_center_x) / image_center_x
        normalized_center_x = max(-1.0, min(1.0, normalized_center_x))
        center_angle_deg = -30.0 * normalized_center_x
        center_angle_deg = center_angle_deg % 360.0
        center_angle_rad = math.radians(center_angle_deg)
        
        center_idx_float = (center_angle_rad - angle_min) / angle_inc
        center_idx = int(round(center_idx_float)) % n

        return {
            'distance': final_distance,
            'angle_deg': center_angle_deg,
            'angle_rad': center_angle_rad,
            'lidar_index': center_idx,
            'bbox_center_x': bbox_center_x,
            'bbox_left_x': bbox_left_x,
            'bbox_right_x': bbox_right_x
        }

    # =========================================================================
    #  VLM & DATABASE LOGIC
    # =========================================================================

    def load_objects_from_file(self):
        if os.path.exists(self.objects_file_path):
            try:
                with open(self.objects_file_path, 'r') as f:
                    self.detected_objects_db = json.load(f)
                print(f"Loaded {len(self.detected_objects_db)} objects from DB.")
            except Exception as e:
                print(f"Failed to load objects: {e}")
                self.detected_objects_db = []
        else:
            self.detected_objects_db = []
        
        if self.detected_objects_db:
            self.next_object_id = max(obj.get('id', 0) for obj in self.detected_objects_db) + 1
        else:
            self.next_object_id = 1

    def save_objects_to_file(self):
        try:
            os.makedirs(os.path.dirname(self.objects_file_path), exist_ok=True)
            with open(self.objects_file_path, 'w') as f:
                json.dump(self.detected_objects_db, f, indent=2)
            print(f"Saved DB to {self.objects_file_path}")
        except Exception as e:
            print(f"Error saving DB: {e}")

    def is_duplicate_location(self, global_coords):
        """
        Check strictly based on location. 
        If something exists at (x,y) within threshold, we return its ID and its VLM Name.
        """
        for obj in self.detected_objects_db:
            stored_coords = obj['world_coordinates']
            distance = math.sqrt(
                (global_coords['x'] - stored_coords['x'])**2 + 
                (global_coords['y'] - stored_coords['y'])**2
            )
            if distance <= self.coordinate_threshold:
                return True, obj['id'], obj['label'], obj['confidence']
        return False, None, None, None

    def run_vlm_inference(self, cv2_image, bbox):
        """
        Crops the image and runs SmolVLM inference.
        Returns: (name, confidence)
        """
        if self.vlm_model is None:
            return "VLM_Error", 0.0

        # Crop the image (y1:y2, x1:x2)
        x1, y1, x2, y2 = map(int, bbox)
        # Clamp to image dimensions
        h, w, _ = cv2_image.shape
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        
        if x2 <= x1 or y2 <= y1:
            return "Invalid_Crop", 0.0

        crop = cv2_image[y1:y2, x1:x2]
        
        # Convert to PIL for VLM
        pil_image = Image.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))

        # Create the prompt
        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image"},
                    {"type": "text", "text": "Identify the main object in this image. Provide a short name and a confidence score (0.0 to 1.0). Format: Name, Score"}
                ]
            },
        ]

        try:
            # Prepare inputs
            prompt = self.processor.apply_chat_template(messages, add_generation_prompt=True)
            inputs = self.processor(text=prompt, images=[pil_image], return_tensors="pt")
            inputs = inputs.to(self.device)

            # Generate (Blocking call)
            generated_ids = self.vlm_model.generate(**inputs, max_new_tokens=50)
            generated_texts = self.processor.batch_decode(
                generated_ids, 
                skip_special_tokens=True,
            )
            
            # Parsing response
            response = generated_texts[0].split("Assistant:")[-1].strip()
            
            # Attempt to parse "Name, Score"
            if "," in response:
                parts = response.split(",")
                name = parts[0].strip()
                try:
                    score = float(parts[1].strip())
                except ValueError:
                    score = 0.5 
                return name, score
            else:
                return response, 0.8  # Fallback

        except Exception as e:
            self.get_logger().error(f"VLM Inference Error: {e}")
            return "Inference_Failed", 0.0

    def add_object_with_vlm(self, frame, bbox, global_coords, lidar_data, yolo_label):
        """
        Runs VLM and adds to DB. 
        Returns: (Success, VLM_Name, VLM_Score)
        """
        print(f"--> NEW LOCATION ({global_coords['x']:.1f}, {global_coords['y']:.1f}). Running VLM...")
        
        # --- BLOCKING VLM INFERENCE ---
        vlm_name, vlm_conf = self.run_vlm_inference(frame, bbox)
        print(f"--> VLM RESULT: {vlm_name} ({vlm_conf})")

        new_object = {
            "id": self.next_object_id,
            "label": vlm_name,            # VLM Label
            "yolo_hint": yolo_label,      # Original YOLO Label
            "confidence": vlm_conf,       # VLM Score
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
        """Publish results to /vlm/detection_result"""
        detection_msg = String()
        
        if detected_objects:
            best = max(detected_objects, key=lambda obj: obj['confidence'])
            
            # Use VLM label if available
            label = best.get('vlm_label', best['label'])
            conf = best.get('vlm_conf', best['confidence'])
            bbox = best['bbox']
            bbox_str = f"{bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}"
            
            if best['global_coords'] and best['lidar_data']:
                gc = best['global_coords']
                ld = best['lidar_data']
                # Format: label, conf, x, y, dist, angle, bbox
                detection_msg.data = f"{label},{conf:.4f},{gc['x']:.2f},{gc['y']:.2f},{ld['distance']:.2f},{ld['angle_rad']:.2f},{bbox_str}"
            else:
                detection_msg.data = f"{label},{conf:.4f},None,None,None,None,{bbox_str}"
        else:
            detection_msg.data = "None,0.0,None,None,None,None,None,None,None,None"
        
        self.detection_publisher.publish(detection_msg)

    def compressed_image_callback(self, msg):
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        self.process_frame(frame)

    def process_frame(self, frame):
        # 1. Run YOLO
        results = self.yolo_model(frame, verbose=False)
        annotated = frame.copy()
        h, w = frame.shape[:2]
        
        current_frame_detections = []

        for r in results:
            boxes = r.boxes
            if boxes is not None:
                for box in boxes:
                    b = box.xyxy[0].int().cpu().numpy()
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    yolo_label = self.yolo_model.names[cls]
                    
                    # 2. Get LiDAR data (New Logic)
                    lidar_data = self.get_object_distance_from_bbox(float(b[0]), float(b[2]), w)
                    
                    # 3. Calculate Global Coords (New Logic)
                    global_coords = self.calculate_object_global_coordinates(lidar_data)
                    
                    vlm_label = None
                    vlm_score = None

                    # 4. IF we have coords, Logic Tree:
                    if global_coords:
                        # Check if location exists
                        is_dup, exist_id, exist_label, exist_conf = self.is_duplicate_location(global_coords)
                        
                        if is_dup:
                            # EXISTING OBJECT: Use stored VLM data
                            vlm_label = exist_label
                            vlm_score = exist_conf
                        else:
                            # NEW OBJECT: Run VLM
                            added, v_name, v_conf = self.add_object_with_vlm(frame, b, global_coords, lidar_data, yolo_label)
                            vlm_label = v_name
                            vlm_score = v_conf

                    # 5. Visualization
                    cv2.rectangle(annotated, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                    
                    if vlm_label:
                        # Show VLM Label in Orange
                        display_text = f"{vlm_label} ({vlm_score:.2f})"
                        cv2.putText(annotated, display_text, (b[0], b[1]-25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                        
                        # Show Distance info
                        if lidar_data and lidar_data['distance']:
                             dist_text = f"{lidar_data['distance']:.1f}m"
                             cv2.putText(annotated, dist_text, (b[0], b[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    else:
                        # Fallback to YOLO label if no VLM run (e.g. no coords)
                        cv2.putText(annotated, f"{yolo_label}", (b[0], b[1]-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    current_frame_detections.append({
                        'label': yolo_label,
                        'vlm_label': vlm_label if vlm_label else yolo_label,
                        'vlm_conf': vlm_score if vlm_score else conf,
                        'confidence': conf,
                        'bbox': b,
                        'global_coords': global_coords,
                        'lidar_data': lidar_data
                    })

        self.publish_detection_results(current_frame_detections)
        
        cv2.imshow("YOLO + VLM Perception", annotated)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = VLMClipCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()