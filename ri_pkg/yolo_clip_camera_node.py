import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import cv2
from PIL import Image as PILImage
import numpy as np
from transformers import CLIPProcessor, CLIPModel
import json
import math
import os
from datetime import datetime

YOLO_MODEL_PATH = "yolov8s.pt"
RUN_CLIP = True
CANDIDATE_LABELS = ["chair", "table", "sofa", "robot", "person", "monitor", "cup", "book", "door", "window"]

# Get the package directory path
PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBJECTS_JSON_FILE = os.path.join(PACKAGE_DIR, "detected_objects.json")
DUPLICATE_THRESHOLD = 0.5  # meters - minimum distance to consider objects as different

class YoloClipCameraNode(Node):
    def __init__(self):
        super().__init__('yolo_clip_camera_node')
        self.bridge = CvBridge()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        # Robot state variables
        self.robot_pose = None
        self.lidar_data = None
        self.detected_objects = self.load_detected_objects()
        
        # Log where objects will be saved
        self.get_logger().info(f"Objects will be saved to: {OBJECTS_JSON_FILE}")

        self.get_logger().info("Loading YOLOv8 model...")
        self.yolo_model = YOLO(YOLO_MODEL_PATH)
        self.yolo_model.to(self.device)
        self.get_logger().info("YOLOv8 loaded.")

        self.run_clip = RUN_CLIP
        if self.run_clip:
            self.get_logger().info("Loading CLIP...")
            try:
                self.clip_model = CLIPModel.from_pretrained("openai/clip-vit-base-patch32").to(self.device)
                self.clip_processor = CLIPProcessor.from_pretrained("openai/clip-vit-base-patch32")
                inputs = self.clip_processor(text=CANDIDATE_LABELS, return_tensors="pt", padding=True).to(self.device)
                with torch.no_grad():
                    text_emb = self.clip_model.get_text_features(**inputs)
                    text_emb = text_emb / text_emb.norm(p=2, dim=-1, keepdim=True)
                self.text_embeddings = text_emb
                self.get_logger().info("CLIP loaded.")
            except Exception as e:
                self.get_logger().error(f"CLIP load failed: {e}")
                self.run_clip = False

        # Subscriptions
        self.image_subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        
        self.get_logger().info("Yolo+CLIP camera node started. Waiting for images, pose, and lidar data...")

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

    def add_detected_object(self, label, confidence, world_coords, clip_label=None, clip_score=None):
        """Add new detected object to the list"""
        obj_data = {
            'id': len(self.detected_objects) + 1,
            'label': label,
            'confidence': confidence,
            'world_coordinates': world_coords,
            'robot_pose_at_detection': self.robot_pose.copy(),
            'timestamp': datetime.now().isoformat()
        }
        
        if clip_label and clip_score:
            obj_data['clip_label'] = clip_label
            obj_data['clip_score'] = clip_score
        
        self.detected_objects.append(obj_data)
        self.save_detected_objects()
        
        self.get_logger().info(
            f"New object detected: {label} at ({world_coords['x']:.2f}, {world_coords['y']:.2f}), "
            f"distance: {world_coords['distance']:.2f}m"
        )

    def image_callback(self, msg):
        """Enhanced callback for camera images with object coordinate calculation"""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.yolo_model(frame)
        annotated = frame.copy()
        
        image_height, image_width = frame.shape[:2]

        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].int().cpu().numpy()
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                label = self.yolo_model.names[cls]

                # Calculate bounding box center
                bbox_center_x = (b[0] + b[2]) / 2
                bbox_center_y = (b[1] + b[3]) / 2

                # Get world coordinates
                world_coords = self.get_object_world_coordinates(
                    bbox_center_x, bbox_center_y, image_width, image_height
                )

                # Draw bounding box
                cv2.rectangle(annotated, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                
                # Base label with confidence
                display_text = f"{label} {conf:.2f}"
                
                # Add coordinate info if available
                if world_coords:
                    display_text += f" ({world_coords['x']:.1f}, {world_coords['y']:.1f})"
                
                cv2.putText(annotated, display_text, (b[0], b[1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # CLIP processing
                clip_label = None
                clip_score = None
                if self.run_clip:
                    crop = frame[b[1]:b[3], b[0]:b[2]]
                    if crop.size != 0:
                        pil_crop = PILImage.fromarray(cv2.cvtColor(crop, cv2.COLOR_BGR2RGB))
                        inputs = self.clip_processor(text=None, images=pil_crop, return_tensors="pt").to(self.device)
                        with torch.no_grad():
                            img_emb = self.clip_model.get_image_features(**inputs)
                            img_emb = img_emb / img_emb.norm(p=2, dim=-1, keepdim=True)
                            sim = (img_emb @ self.text_embeddings.T).squeeze(0)
                            best_idx = sim.argmax().item()
                            clip_label = CANDIDATE_LABELS[best_idx]
                            clip_score = sim[best_idx].item()
                            cv2.putText(annotated, f"CLIP: {clip_label} ({clip_score:.2f})", (b[0], b[3]+15),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 200, 0), 2)

                # Save object if valid coordinates and not duplicate
                if world_coords and not self.is_duplicate_object(world_coords, label):
                    self.add_detected_object(label, conf, world_coords, clip_label, clip_score)

        # Display total detected objects count
        cv2.putText(annotated, f"Total objects saved: {len(self.detected_objects)}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("YOLOv8 + CLIP Detection with Coordinates", annotated)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YoloClipCameraNode()
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
