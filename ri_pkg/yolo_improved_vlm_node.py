#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO
from transformers import AutoProcessor, AutoModelForImageTextToText
from PIL import Image as PILImage
import time
import re
import json
import math
import os
from datetime import datetime

# Get the package directory path for JSON storage
PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBJECTS_JSON_FILE = os.path.join(PACKAGE_DIR, "detected_objects_vlm.json")
DUPLICATE_THRESHOLD = 0.5  # meters - minimum distance to consider objects as different

class YoloSmolVLM2Node(Node):
    def __init__(self):
        super().__init__("yolo_smolvlm2_node")
        self.bridge = CvBridge()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        # --- Robot state variables for coordinate mapping ---
        self.robot_pose = None
        self.lidar_data = None
        self.detected_objects = self.load_detected_objects()
        
        # Log where objects will be saved
        self.get_logger().info(f"Objects will be saved to: {OBJECTS_JSON_FILE}")

        # --- Frame rate control (2 FPS) ---
        self.last_frame_time = 0.0
        self.frame_interval = 0.5  # seconds → 2 FPS

        # --- YOLO ---
        self.get_logger().info("Loading YOLO model...")
        self.yolo = YOLO("yolov8s.pt")  
        self.yolo.to(self.device)
        self.get_logger().info("YOLO loaded successfully.")

        # --- SmolVLM2 ---
        self.get_logger().info("Loading SmolVLM2 model...")
        try:
            model_path = "HuggingFaceTB/SmolVLM2-256M-Video-Instruct"
            self.processor = AutoProcessor.from_pretrained(model_path)
            self.vlm_model = AutoModelForImageTextToText.from_pretrained(
                model_path,
                torch_dtype=torch.bfloat16 if self.device == "cuda" else torch.float32,
                _attn_implementation="eager"
            ).to(self.device)
            self.vlm_enabled = True
            self.get_logger().info("SmolVLM2 loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load SmolVLM2: {e}")
            self.vlm_enabled = False

        # --- Subscriptions to camera, odometry, and lidar topics ---
        self.sub = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        self.get_logger().info("Node started. Waiting for camera frames, pose, and lidar data...")

    def load_detected_objects(self):
        """Load previously detected objects from JSON file"""
        if os.path.exists(OBJECTS_JSON_FILE):
            try:
                with open(OBJECTS_JSON_FILE, 'r') as f:
                    return json.load(f)
            except Exception as e:
                self.get_logger().warn(f"Failed to load objects file: {e}")
        return []

    def save_detected_objects(self):
        """Save detected objects to JSON file"""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(OBJECTS_JSON_FILE), exist_ok=True)
            with open(OBJECTS_JSON_FILE, 'w') as f:
                json.dump(self.detected_objects, f, indent=2)
            self.get_logger().info(f"Objects saved to: {OBJECTS_JSON_FILE}")
        except Exception as e:
            self.get_logger().error(f"Failed to save objects file: {e}")

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

    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        x, y, z, w = quaternion['x'], quaternion['y'], quaternion['z'], quaternion['w']
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_object_world_coordinates(self, bbox_center_x, bbox_center_y, image_width, image_height):
        """Calculate object's world coordinates using robot pose and lidar data"""
        if self.robot_pose is None or self.lidar_data is None:
            return None

        # Convert image coordinates to angles relative to robot's heading
        # Assuming camera FOV is approximately 62 degrees (typical for many cameras)
        camera_fov = math.radians(62)
        
        # Calculate angle offset from center of image
        center_offset_ratio = (bbox_center_x - image_width / 2) / (image_width / 2)
        angle_offset = center_offset_ratio * (camera_fov / 2)
        
        # Get robot's current yaw
        robot_yaw = self.quaternion_to_yaw(self.robot_pose['orientation'])
        
        # Calculate absolute angle of the object
        object_angle = robot_yaw + angle_offset
        
        # Find corresponding lidar reading
        lidar_angle_index = int((object_angle - self.lidar_data['angle_min']) / self.lidar_data['angle_increment'])
        
        # Clamp index to valid range
        lidar_angle_index = max(0, min(len(self.lidar_data['ranges']) - 1, lidar_angle_index))
        
        # Get distance from lidar
        distance = self.lidar_data['ranges'][lidar_angle_index]
        
        # Check if distance is valid
        if distance < self.lidar_data['range_min'] or distance > self.lidar_data['range_max'] or math.isnan(distance) or math.isinf(distance):
            return None
        
        # Calculate world coordinates
        world_x = self.robot_pose['x'] + distance * math.cos(object_angle)
        world_y = self.robot_pose['y'] + distance * math.sin(object_angle)
        
        return {
            'x': world_x,
            'y': world_y,
            'distance': distance,
            'angle': object_angle
        }

    def is_duplicate_object(self, new_coords, label):
        """Check if object is already detected (within threshold distance)"""
        for existing_obj in self.detected_objects:
            if existing_obj['label'] == label:
                existing_coords = existing_obj['world_coordinates']
                distance = math.sqrt(
                    (new_coords['x'] - existing_coords['x']) ** 2 + 
                    (new_coords['y'] - existing_coords['y']) ** 2
                )
                if distance < DUPLICATE_THRESHOLD:
                    return True
        return False

    def add_detected_object(self, label, confidence, world_coords, vlm_description=None):
        """Add new detected object to the list"""
        obj_data = {
            'id': len(self.detected_objects) + 1,
            'label': label,
            'confidence': confidence,
            'world_coordinates': world_coords,
            'robot_pose_at_detection': self.robot_pose.copy(),
            'timestamp': datetime.now().isoformat()
        }
        
        if vlm_description:
            obj_data['vlm_description'] = vlm_description
        
        self.detected_objects.append(obj_data)
        self.save_detected_objects()
        
        self.get_logger().info(
            f"New object detected: {label} at ({world_coords['x']:.2f}, {world_coords['y']:.2f}), "
            f"distance: {world_coords['distance']:.2f}m"
        )

    def image_callback(self, msg: Image):
        # --- Frame rate limiter ---
        current_time = time.time()
        if current_time - self.last_frame_time < self.frame_interval:
            return  # Skip frame to maintain 2 FPS
        self.last_frame_time = current_time

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self.yolo(frame)
        annotated = frame.copy()
        
        image_height, image_width = frame.shape[:2]

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = self.yolo.model.names[cls]

                # Calculate bounding box center for coordinate mapping
                bbox_center_x = (x1 + x2) / 2
                bbox_center_y = (y1 + y2) / 2

                # Get world coordinates
                world_coords = self.get_object_world_coordinates(
                    bbox_center_x, bbox_center_y, image_width, image_height
                )

                # Draw bounding box
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Base label with confidence
                display_text = f"{label} {conf:.2f}"
                
                # Add coordinate info if available
                if world_coords:
                    display_text += f" ({world_coords['x']:.1f}, {world_coords['y']:.1f})"
                
                cv2.putText(annotated, display_text, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                vlm_description = None
                if not self.vlm_enabled:
                    # If VLM is disabled, still save object if coordinates are valid
                    if world_coords and not self.is_duplicate_object(world_coords, label):
                        self.add_detected_object(label, conf, world_coords)
                    continue

                crop = frame[y1:y2, x1:x2]
                if crop.size == 0:
                    continue

                pil_img = PILImage.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
                messages = [
                    {
                        "role": "user",
                        "content": [
                            {"type": "image", "image": pil_img},
                            {"type": "text", "text": "Describe this object briefly."},
                        ]
                    }
                ]

                try:
                    inputs = self.processor.apply_chat_template(
                        messages,
                        add_generation_prompt=True,
                        tokenize=True,
                        return_dict=True,
                        return_tensors="pt"
                    ).to(self.device, dtype=torch.bfloat16 if self.device == "cuda" else torch.float32)

                    with torch.no_grad():
                        generated_ids = self.vlm_model.generate(**inputs, do_sample=False, max_new_tokens=50)
                        text = self.processor.batch_decode(generated_ids, skip_special_tokens=True)[0]

                    # --- Robust parsing of assistant response ---
                    try:
                        if "<assistant>" in text.lower():
                            match = re.search(r"<\s*assistant\s*>\s*(.*)", text,
                                              flags=re.IGNORECASE | re.DOTALL)
                            text = match.group(1).strip() if match else text
                        elif "assistant:" in text.lower():
                            text = re.split(r"assistant:\s*", text,
                                            flags=re.IGNORECASE)[-1].strip()
                        elif "Assistant" in text:
                            text = text.split("Assistant")[-1].strip()
                        text = text.strip()
                    except Exception as e:
                        self.get_logger().warn(f"Assistant parsing failed: {e}")
                        text = text.strip()

                    # Store the VLM description for potential saving
                    vlm_description = text

                    # --- Optional: trim very long responses for overlay ---
                    if len(text) > 80:
                        text = text[:77] + "..."

                    cv2.putText(annotated, f"VLM: {text}", (x1, y2 + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

                except torch.cuda.OutOfMemoryError:
                    self.get_logger().warn("GPU OOM during VLM inference — skipping this box.")
                    torch.cuda.empty_cache()
                except Exception as e:
                    self.get_logger().warn(f"SmolVLM2 inference failed: {e}")

                # Save object if valid coordinates and not duplicate
                if world_coords and not self.is_duplicate_object(world_coords, label):
                    self.add_detected_object(label, conf, world_coords, vlm_description)

        # Display total detected objects count
        cv2.putText(annotated, f"Total objects saved: {len(self.detected_objects)}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("YOLO + SmolVLM2 with Coordinates", annotated)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YoloSmolVLM2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
 