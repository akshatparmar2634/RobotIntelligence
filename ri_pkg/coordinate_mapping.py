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

YOLO_MODEL_PATH = "yolov8s.pt"

class YoloClipCameraNode(Node):
    def __init__(self):
        super().__init__('yolo_clip_camera_node')
        self.bridge = CvBridge()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        # Robot state variables
        self.robot_pose = None
        self.lidar_data = None
        
        # Object storage
        self.detected_objects_db = []
        self.next_object_id = 1
        
        # Get the package directory path for proper file location
        PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.objects_file_path = os.path.join(PACKAGE_DIR, "assets", "detected_objects_map.json")
        self.coordinate_threshold = 1  # meters for duplicate detection
        
        # Load existing objects if file exists
        self.load_objects_from_file()

        self.yolo_model = YOLO(YOLO_MODEL_PATH)
        self.yolo_model.to(self.device)

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
        # Print odom data only when requested (removed continuous printing)

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
        x = orientation['x']
        y = orientation['y']
        z = orientation['z']
        w = orientation['w']
        
        # Calculate yaw (rotation around z-axis)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def calculate_object_global_coordinates(self, lidar_data):
        """Calculate global coordinates of object using robot pose and LiDAR data"""
        if self.robot_pose is None or lidar_data is None or lidar_data['distance'] is None:
            return None
        
        # Get robot's current position and orientation
        robot_x = self.robot_pose['x']
        robot_y = self.robot_pose['y']
        robot_yaw = self.quaternion_to_yaw(self.robot_pose['orientation'])
        
        # Object's angle relative to robot (in radians)
        object_angle_rad = lidar_data['angle_rad']
        object_distance = lidar_data['distance']
        
        # Calculate object's global angle (robot's yaw + object's relative angle)
        global_angle = robot_yaw + object_angle_rad
        
        # Calculate object's global coordinates
        object_global_x = robot_x + object_distance * math.cos(global_angle)
        object_global_y = robot_y + object_distance * math.sin(global_angle)
        
        return {
            'x': object_global_x,
            'y': object_global_y,
            'robot_yaw_deg': math.degrees(robot_yaw),
            'global_angle_deg': math.degrees(global_angle)
        }

    def load_objects_from_file(self):
        """Load previously detected objects from JSON file"""
        if os.path.exists(self.objects_file_path):
            try:
                with open(self.objects_file_path, 'r') as f:
                    self.detected_objects_db = json.load(f)
                print(f"Loaded {len(self.detected_objects_db)} objects from {self.objects_file_path}")
            except Exception as e:
                print(f"Failed to load objects file: {e}")
                self.detected_objects_db = []
        else:
            print(f"No existing object file found. Starting fresh.")
            self.detected_objects_db = []
        
        # Set next_object_id based on existing objects
        if self.detected_objects_db:
            self.next_object_id = max(obj.get('id', 0) for obj in self.detected_objects_db) + 1
        else:
            self.next_object_id = 1

    def save_objects_to_file(self):
        """Save detected objects to JSON file"""
        try:
            # Ensure directory exists
            os.makedirs(os.path.dirname(self.objects_file_path), exist_ok=True)
            with open(self.objects_file_path, 'w') as f:
                json.dump(self.detected_objects_db, f, indent=2)
            print(f"Saved {len(self.detected_objects_db)} objects to {self.objects_file_path}")
        except Exception as e:
            print(f"Error saving objects file: {e}")

    def is_duplicate_object(self, global_coords, label):
        """Check if object already exists within threshold distance"""
        for obj in self.detected_objects_db:
            stored_coords = obj['world_coordinates']
            
            # Calculate distance between coordinates
            distance = math.sqrt(
                (global_coords['x'] - stored_coords['x'])**2 + 
                (global_coords['y'] - stored_coords['y'])**2
            )
            
            # Check if within threshold and same label
            if distance <= self.coordinate_threshold and obj['label'] == label:
                return True, obj['id']
        
        return False, None

    def add_object_to_database(self, label, confidence, global_coords, lidar_data):
        """Add new object to database if it's not a duplicate"""
        # Check for duplicates
        is_duplicate, existing_id = self.is_duplicate_object(global_coords, label)
        
        if is_duplicate:
            print(f"Duplicate {label} detected near existing object ID {existing_id}, skipping...")
            return False
        
        # Create new object entry following the exact format from reference code
        new_object = {
            "id": self.next_object_id,
            "label": label,
            "confidence": confidence,
            "world_coordinates": {
                "x": round(global_coords['x'], 2),
                "y": round(global_coords['y'], 2),
                "distance": round(lidar_data['distance'], 2),
                "angle": round(lidar_data['angle_rad'], 2),
            },
            "robot_pose_at_detection": {
                "x": self.robot_pose['x'],
                "y": self.robot_pose['y'],
                "z": self.robot_pose['z'],
                "orientation": {
                    "x": self.robot_pose['orientation']['x'],
                    "y": self.robot_pose['orientation']['y'],
                    "z": self.robot_pose['orientation']['z'],
                    "w": self.robot_pose['orientation']['w']
                }
            },
            "timestamp": datetime.now().isoformat()
        }
        
        # Add to database
        self.detected_objects_db.append(new_object)
        print(f"NEW OBJECT SAVED: {label} (ID: {self.next_object_id}) at ({global_coords['x']:.2f}, {global_coords['y']:.2f})")
        
        self.next_object_id += 1
        
        # Save to file immediately
        self.save_objects_to_file()
        
        return True

    def publish_detection_results(self, detected_objects):
        """Publish YOLO detection results to ROS topic"""
        detection_msg = String()
        
        if detected_objects:
            # Find the detection with highest confidence
            best_detection = max(detected_objects, key=lambda obj: obj['confidence'])
            
            # Get bbox coordinates
            bbox = best_detection['bbox']
            bbox_str = f"{bbox[0]},{bbox[1]},{bbox[2]},{bbox[3]}"
            
            # Check if we have global coordinates and lidar data
            if best_detection['global_coords'] and best_detection['lidar_data']:
                gc = best_detection['global_coords']
                ld = best_detection['lidar_data']
                # Format: "label,confidence,x,y,distance,angle,bbox_left,bbox_top,bbox_right,bbox_bottom"
                detection_msg.data = f"{best_detection['label']},{best_detection['confidence']:.4f},{gc['x']:.2f},{gc['y']:.2f},{ld['distance']:.2f},{ld['angle_rad']:.2f},{bbox_str}"
            else:
                # Only label, confidence and bbox available
                detection_msg.data = f"{best_detection['label']},{best_detection['confidence']:.4f},None,None,None,None,{bbox_str}"
        else:
            # No detections found
            detection_msg.data = "None,0.0,None,None,None,None,None,None,None,None"
        
        # Publish the combined message
        self.detection_publisher.publish(detection_msg)
        
        # Log the published data
        if detected_objects:
            parts = detection_msg.data.split(',')
            if parts[2] != "None":
                self.get_logger().info(f"Published detection: {parts[0]} (conf: {parts[1]}) at ({parts[2]}, {parts[3]}) dist: {parts[4]}m angle: {parts[5]}rad bbox: ({parts[6]},{parts[7]},{parts[8]},{parts[9]})")
            else:
                self.get_logger().info(f"Published detection: {parts[0]} with confidence {parts[1]} bbox: ({parts[6]},{parts[7]},{parts[8]},{parts[9]}) (no position data)")
        else:
            self.get_logger().info("Published: No detections found")

    def get_object_distance_from_bbox(self, bbox_left_x, bbox_right_x, image_width,
                                    num_samples=5, outlier_filter=True):
        """
        Returns dict with distance + metadata or None if lidar not available / no valid returns.
        Uses the entire bbox width to sample multiple angles and applies smart filtering.
        
        Args:
            bbox_left_x: Left edge of bounding box
            bbox_right_x: Right edge of bounding box  
            image_width: Width of the image
            num_samples: Number of sample points across bbox width
            outlier_filter: Whether to apply outlier filtering to distance array
        """
        if self.lidar_data is None:
            return None

        # Basic params from stored scan
        ranges = np.array(self.lidar_data['ranges'], dtype=float)
        angle_min = float(self.lidar_data['angle_min'])
        angle_inc = float(self.lidar_data['angle_increment'])
        n = ranges.size

        # sanitize lidar invalid returns to NaN for easy filtering
        ranges[~np.isfinite(ranges)] = np.nan

        # Calculate sample points across the bbox width
        bbox_width = bbox_right_x - bbox_left_x
        if bbox_width <= 0:
            # Fallback to center point if invalid bbox
            sample_x_positions = [bbox_left_x]
        else:
            # Create evenly spaced sample points across bbox width
            sample_x_positions = np.linspace(bbox_left_x, bbox_right_x, num_samples)

        # Collect distance measurements for each sample point
        distance_measurements = []
        angle_measurements = []
        lidar_indices = []
        
        image_center_x = float(image_width) / 2.0
        if image_center_x == 0:
            raise ValueError("image_width must be > 0")

        for sample_x in sample_x_positions:
            # Compute normalized_x in [-1, 1] (left -> -1, center 0, right -> +1)
            normalized_x = (float(sample_x) - image_center_x) / image_center_x
            # clamp to [-1, 1] to avoid out-of-bounds
            normalized_x = max(-1.0, min(1.0, normalized_x))

            # Map normalized_x -> angle in degrees (linear mapping)
            # left (nx=-1) -> +30 deg, center (0) -> 0 deg, right (1) -> -30 deg
            angle_deg = -30.0 * normalized_x

            # Normalize to [0, 360)
            angle_deg = angle_deg % 360.0

            # Convert to radians
            angle_rad = math.radians(angle_deg)
            angle_measurements.append(angle_deg)

            # Compute floating index and round to nearest index (handle wrap with modulo)
            idx_float = (angle_rad - angle_min) / angle_inc
            idx = int(round(idx_float)) % n
            lidar_indices.append(idx)

            # Get distance at this angle
            if np.isfinite(ranges[idx]):
                distance_measurements.append(float(ranges[idx]))
            else:
                distance_measurements.append(None)

        # Filter out None values
        valid_distances = [d for d in distance_measurements if d is not None]
        
        if len(valid_distances) == 0:
            final_distance = None
        elif outlier_filter and len(valid_distances) > 2:
            # Apply outlier filtering logic
            # Sort distances to identify potential outliers
            sorted_distances = sorted(valid_distances)
            
            # Remove the largest value if it's significantly different (outlier detection)
            if len(sorted_distances) >= 3:
                # Calculate median and check if max value is an outlier
                median_val = np.median(sorted_distances[:-1])  # median without the max
                max_val = sorted_distances[-1]
                
                # If max value is more than 30% larger than median, consider it an outlier
                if max_val > median_val * 1.3:
                    filtered_distances = sorted_distances[:-1]
                else:
                    filtered_distances = sorted_distances
            else:
                filtered_distances = sorted_distances
            
            # Take average of remaining valid distances
            final_distance = float(np.mean(filtered_distances))
        else:
            # No outlier filtering or too few samples - just take average
            final_distance = float(np.mean(valid_distances))

        # Calculate center angle for backwards compatibility
        bbox_center_x = (bbox_left_x + bbox_right_x) / 2.0
        normalized_center_x = (bbox_center_x - image_center_x) / image_center_x
        normalized_center_x = max(-1.0, min(1.0, normalized_center_x))
        center_angle_deg = -30.0 * normalized_center_x
        center_angle_deg = center_angle_deg % 360.0
        center_angle_rad = math.radians(center_angle_deg)
        
        # Find the lidar index corresponding to center angle
        center_idx_float = (center_angle_rad - angle_min) / angle_inc
        center_idx = int(round(center_idx_float)) % n

        return {
            'distance': final_distance,
            'angle_deg': center_angle_deg,  # Center angle for backwards compatibility
            'angle_rad': center_angle_rad,
            'lidar_index': center_idx,
            'bbox_center_x': bbox_center_x,
            'bbox_left_x': bbox_left_x,
            'bbox_right_x': bbox_right_x,
            'normalized_x': normalized_center_x,
            'distance_array': distance_measurements,
            'valid_distances': valid_distances,
            'angle_array': angle_measurements,
            'lidar_indices': lidar_indices,
            'num_samples': len(sample_x_positions),
            'outlier_filtered': outlier_filter and len(valid_distances) > 2
        }

    def compressed_image_callback(self, msg):
        """Callback for compressed camera images"""
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            return
        self.process_frame(frame)

    def process_frame(self, frame):
        """Process frame for YOLO detection"""
        results = self.yolo_model(frame)
        annotated = frame.copy()
        
        image_height, image_width = frame.shape[:2]
        
        # Collect all detected objects
        detected_objects = []

        for r in results:
            boxes = r.boxes
            if boxes is not None and len(boxes) > 0:
                
                for box in boxes:
                    b = box.xyxy[0].int().cpu().numpy()
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    label = self.yolo_model.names[cls]
                    
                    bbox_left_x = float(b[0])
                    bbox_right_x = float(b[2])
                    bbox_center_x = (bbox_left_x + bbox_right_x) / 2
                    bbox_center_y = (b[1] + b[3]) / 2

                    # Draw bounding box
                    cv2.rectangle(annotated, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                    
                    # Draw yellow center point
                    center_x, center_y = int(bbox_center_x), int(bbox_center_y)
                    cv2.circle(annotated, (center_x, center_y), 5, (0, 255, 255), -1)
                    
                    # Calculate LiDAR distance for this object using the full bbox width
                    lidar_data = self.get_object_distance_from_bbox(bbox_left_x, bbox_right_x, image_width)
                    
                    if lidar_data and lidar_data['distance'] is not None:
                        # Calculate global coordinates
                        global_coords = self.calculate_object_global_coordinates(lidar_data)
                        
                        if global_coords:
                            # Add object to database (with duplicate checking)
                            self.add_object_to_database(label, conf, global_coords, lidar_data)
                        
                        detected_objects.append({
                            'label': label,
                            'bbox': b,
                            'center': (bbox_center_x, bbox_center_y),
                            'confidence': conf,
                            'lidar_data': lidar_data,
                            'global_coords': global_coords
                        })
                        
                        # Display distance, angle, and coordinates on image
                        if global_coords:
                            lidar_text = f"{label}: {lidar_data['distance']:.1f}m @ {lidar_data['angle_deg']:.1f}°"
                            coord_text = f"({global_coords['x']:.1f}, {global_coords['y']:.1f})"
                            cv2.putText(annotated, lidar_text, (b[0], b[1]-30),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                            cv2.putText(annotated, coord_text, (b[0], b[1]-10),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                        else:
                            lidar_text = f"{label}: {lidar_data['distance']:.1f}m @ {lidar_data['angle_deg']:.1f}°"
                            cv2.putText(annotated, lidar_text, (b[0], b[1]-10),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    else:
                        # Object detected but no valid LiDAR data
                        detected_objects.append({
                            'label': label,
                            'bbox': b,
                            'center': (bbox_center_x, bbox_center_y),
                            'confidence': conf,
                            'lidar_data': None,
                            'global_coords': None
                        })
                        
                        cv2.putText(annotated, f"{label}: No LiDAR", (b[0], b[1]-10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # Print detection summary when objects are found
        if detected_objects:
            print(f"\n=== DETECTED {len(detected_objects)} OBJECT(S) ===")
            print(f"Total objects in database: {len(self.detected_objects_db)}")
            
            # Print odom data
            if self.robot_pose:
                print(f"Robot Position: x={self.robot_pose['x']:.2f}, y={self.robot_pose['y']:.2f}, z={self.robot_pose['z']:.2f}")
            
            print("Object Details:")
            print("Label        | Center (x,y)  | Distance | Angle    | Global Coords (x,y) | Robot Yaw | Confidence")
            print("-" * 100)
            
            for i, obj in enumerate(detected_objects):
                if obj['lidar_data'] and obj['global_coords']:
                    ld = obj['lidar_data']
                    gc = obj['global_coords']
                    print(f"{obj['label']:<12} | ({ld['bbox_center_x']:4.0f},{obj['center'][1]:4.0f}) | "
                          f"{ld['distance']:6.2f}m | {ld['angle_deg']:6.1f}° | "
                          f"({gc['x']:6.1f},{gc['y']:6.1f}) | {gc['robot_yaw_deg']:7.1f}° | {obj['confidence']:9.2f}")
                else:
                    print(f"{obj['label']:<12} | ({obj['center'][0]:4.0f},{obj['center'][1]:4.0f}) | "
                          f"  No Data | No Data  |        No Data      |  No Data | {obj['confidence']:9.2f}")
            
            print("=" * 100)

        # Publish detection results
        self.publish_detection_results(detected_objects)

        cv2.imshow("Object Detection with LiDAR Data", annotated)
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
