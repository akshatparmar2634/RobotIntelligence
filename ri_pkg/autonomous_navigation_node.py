#!/usr/bin/env python3
"""
Autonomous Navigation Node for Waypoint Navigation

This node provides autonomous navigation capabilities for the TurtleBot3 to systematically
navigate through predefined waypoints in the environment.

Features:
- Waypoint-based navigation using Nav2
- Quick waypoint-to-waypoint movement
- Integration with semantic mapping
- Dynamic waypoint generation
- Recovery behaviors
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import GetCostmap, ClearEntireCostmap
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Bool, String
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

try:
    from tts_node import speak  # type: ignore[import]
except Exception:  # pragma: no cover - optional dependency or missing credentials
    speak = None  # type: ignore[assignment]

try:
    from ri_pkg.ri_pkg.stt_node import capture_and_transcribe  # type: ignore[import]
except ImportError:  # pragma: no cover - optional dependency
    capture_and_transcribe = None

import math
import time
import json
import os
import threading
from datetime import datetime
from enum import Enum
from typing import List, Tuple, Optional

import cv2
from cv_bridge import CvBridge, CvBridgeError
# Navigation states
class NavigationState(Enum):
    IDLE = "idle"
    NAVIGATING = "navigating"
    EXPLORING = "exploring" 
    PAUSED = "paused"
    COMPLETED = "completed"
    ERROR = "error"

# Get package directory for saving navigation data
PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WAYPOINTS_FILE = os.path.join(PACKAGE_DIR, "navigation_waypoints.json")
COVERAGE_FILE = os.path.join(PACKAGE_DIR, "coverage_data.json")
HUMAN_CLARIFICATION_FILE = os.path.join(PACKAGE_DIR, "human_clarification.json")
LOW_CONFIDENCE_THRESHOLD = 0.6

class AutonomousNavigationNode(Node):
    def __init__(self):
        super().__init__('autonomous_navigation_node')
        
        # Navigation state
        self.state = NavigationState.IDLE
        self.current_goal_index = 0
        self.exploration_active = False
        
        # Robot pose and sensor data
        self.current_pose = None
        self.lidar_data = None
        self.costmap_data = None
        
        # Waypoint management
        self.predefined_waypoints = []
        self.frontier_waypoints = []
        self.coverage_waypoints = []
        self.visited_waypoints = []
        self.current_waypoint = None
        
        # Navigation parameters
        self.goal_tolerance = 0.5  # meters
        self.rotation_tolerance = 0.2  # radians
        self.exploration_radius = 8.0  # meters
        self.waypoint_spacing = 2.0  # meters for coverage
        self.max_retry_attempts = 3
        self.pause_duration = 0.5  # seconds - minimal pause for fast scanning
        self.awaiting_clarification = False
        self.paused_waypoint = None
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_image_stamp = None
        self.frame_lock = threading.Lock()
        self.clarification_lock = threading.Lock()
        self.low_confidence_threshold = LOW_CONFIDENCE_THRESHOLD
        self.camera_topic = self.declare_parameter('camera_topic', '/camera/color/image_raw').value
        self.clarification_prompt_active = False
        self.clarified_signatures = set()
        self.clarified_labels = set()

        # Speech-to-text configuration (leverages ElevenLabs helper functions)
        stt_enabled_default = True if capture_and_transcribe is not None else False
        self.stt_enabled = bool(self.declare_parameter('stt_enabled', stt_enabled_default).value)
        if capture_and_transcribe is None and self.stt_enabled:
            self.get_logger().warn(
                "Speech-to-text requested but ElevenLabs STT integration is unavailable."
            )
            self.stt_enabled = False

        self.stt_duration = float(self.declare_parameter('stt_duration', 5.0).value)
        self.stt_sample_rate = int(self.declare_parameter('stt_sample_rate', 16000).value)

        language_param = self.declare_parameter('stt_language_code', '').value
        self.stt_language_code = language_param.strip() or None if isinstance(language_param, str) else None

        model_param = self.declare_parameter('stt_model_id', '').value
        self.stt_model_id = model_param.strip() or None if isinstance(model_param, str) else None

        api_key_param = self.declare_parameter('stt_api_key', '').value
        self.stt_api_key = api_key_param.strip() or None if isinstance(api_key_param, str) else None

        self.stt_tag_audio_events = self.declare_parameter('stt_tag_audio_events', False).value
        self.stt_diarize = self.declare_parameter('stt_diarize', False).value
        self.stt_keep_temp_recording = bool(self.declare_parameter('stt_keep_temp_recording', False).value)
        
        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_nav_goal = None
        self.send_goal_future = None
        self.get_result_future = None
        
        # Publishers and Subscribers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, qos_profile)
        self.camera_sub = self.create_subscription(
            Image, self.camera_topic, self.camera_callback, 10)
        self.detection_sub = self.create_subscription(
            String, '/yolo/detection_result', self.detection_callback, 10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.exploration_status_pub = self.create_publisher(Bool, '/exploration_active', 10)
        
        # Services
        self.costmap_client = self.create_client(GetCostmap, '/global_costmap/get_costmap')
        self.clear_costmap_client = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        
        # Timers
        self.navigation_timer = self.create_timer(1.0, self.navigation_loop)
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        # Load saved waypoints and coverage data
        self.load_navigation_data()
        self.load_human_clarifications()
        
        # Initialize predefined waypoints for house exploration
        self.initialize_house_waypoints()
        
        self.get_logger().info("Autonomous Navigation Node initialized")
        self.get_logger().info("Waiting for Nav2 to become available...")
        
        # Wait for Nav2 to be ready
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Nav2 action server not available, waiting...")

        self.get_logger().info("Nav2 action server is ready!")
        
        # Run diagnostic check
        self.diagnostic_check()

    def initialize_house_waypoints(self):
        """Initialize predefined waypoints for systematic house exploration"""
        # Custom waypoints specified by user for targeted exploration
        house_waypoints = [
            # Starting position
            {"x": 0.0, "y": 0.0, "theta": 0.0, "description": "Starting position"},
            
            # User-specified main goal points
            {"x": -4.5, "y": 4.0, "theta": 1.57, "description": "Goal point 2: (-4.5, 4)"},
            {"x": -7.0, "y": 6.25, "theta": 0.0, "description": "Goal point 1: (-7, 6.25)"},
            {"x": -2.5, "y": 7.0, "theta": 3.14, "description": "Goal point 3: (-2.5, 7)"},
            
            # Return to start
            {"x": 0.0, "y": 0.0, "theta": 0.0, "description": "Return to start"}
            # User-specified main goal points for IRAS Hub
            # {"x": 4.5, "y": 0.17, "theta": 0.0, "description": "Goal point 1: (4.5, 0.17)"},
            # {"x": 4.0, "y": 3.15, "theta": 1.57, "description": "Goal point 2: (4, 3.15)"},
            # {"x": 0.67, "y": 3.4, "theta": 3.14, "description": "Goal point 3: (0.67, 3.4)"},
            # {"x": 1.8, "y": 0.485, "theta": -1.57, "description": "Goal point 4: (1.8, 0.485)"},
            
        ]
        
        self.predefined_waypoints = house_waypoints
        self.get_logger().info(f"Initialized {len(house_waypoints)} custom waypoints")
        self.get_logger().info("Custom waypoint sequence:")
        for i, wp in enumerate(house_waypoints):
            self.get_logger().info(f"  {i+1}. ({wp['x']:.2f}, {wp['y']:.2f}) - {wp['description']}")

    def load_navigation_data(self):
        """Load saved navigation data from files"""
        try:
            if os.path.exists(WAYPOINTS_FILE):
                with open(WAYPOINTS_FILE, 'r') as f:
                    data = json.load(f)
                    self.visited_waypoints = data.get('visited', [])
                    self.current_goal_index = data.get('current_index', 0)
                    
                    # Check if exploration was already completed
                    if self.current_goal_index >= len(self.predefined_waypoints):
                        self.get_logger().info(f"Previous exploration completed! Resetting for new run...")
                        self.get_logger().info(f"Previous run visited {len(self.visited_waypoints)} waypoints")
                        
                        # Reset for new exploration
                        self.visited_waypoints = []
                        self.current_goal_index = 0
                        
                        # Save the reset state
                        self.save_navigation_data()
                    else:
                        self.get_logger().info(f"Loaded navigation data: {len(self.visited_waypoints)} visited waypoints")
                        self.get_logger().info(f"Resuming from waypoint {self.current_goal_index + 1}/{len(self.predefined_waypoints)}")
        except Exception as e:
            self.get_logger().warn(f"Could not load navigation data: {e}")
            self.visited_waypoints = []
            self.current_goal_index = 0

    def save_navigation_data(self):
        """Save navigation progress to file"""
        try:
            data = {
                'visited': self.visited_waypoints,
                'current_index': self.current_goal_index,
                'timestamp': datetime.now().isoformat()
            }
            with open(WAYPOINTS_FILE, 'w') as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            self.get_logger().error(f"Could not save navigation data: {e}")

    def load_human_clarifications(self):
        """Load previously clarified detections to suppress repeat prompts."""
        self.clarified_signatures = set()
        self.clarified_labels = set()
        if not os.path.exists(HUMAN_CLARIFICATION_FILE):
            return

        try:
            with open(HUMAN_CLARIFICATION_FILE, 'r') as file_obj:
                data = json.load(file_obj)

            if isinstance(data, list):
                entries = data
            elif isinstance(data, dict):
                entries = data.get('entries', []) or []
            else:
                entries = []

            for record in entries:
                signature = self._signature_from_record(record)
                if signature is not None:
                    self.clarified_signatures.add(signature)
                detected_label = record.get('detected_label') or record.get('label')
                if detected_label:
                    self.clarified_labels.add(detected_label)
                actual_label = record.get('label')
                if actual_label:
                    self.clarified_labels.add(actual_label)
        except Exception as exc:
            self.get_logger().warn(f"Could not load human clarifications: {exc}")

    def diagnostic_check(self):
        """Perform diagnostic checks for navigation readiness"""
        self.get_logger().info("🔧 Running navigation diagnostics...")
        
        # Check if we can get topic info
        try:
            topic_list = self.get_topic_names_and_types()
            available_topics = [topic for topic, _ in topic_list]
            
            # Check essential topics
            essential_topics = ['/odom', '/scan', '/map', '/amcl_pose', '/cmd_vel']
            for topic in essential_topics:
                status = "✅" if topic in available_topics else "❌"
                self.get_logger().info(f"  {status} Topic {topic}")
            
            # Check if odom is publishing
            if '/odom' in available_topics:
                self.get_logger().info("📡 /odom topic found, checking for data...")
                # Force a callback processing
                rclpy.spin_once(self, timeout_sec=2.0)
                if self.current_pose:
                    self.get_logger().info(f"✅ Robot pose available: ({self.current_pose['x']:.3f}, {self.current_pose['y']:.3f})")
                else:
                    self.get_logger().warn("⚠️ /odom topic exists but no pose data received")
            
        except Exception as e:
            self.get_logger().error(f"❌ Diagnostic check failed: {e}")

    def odom_callback(self, msg):
        """Callback for odometry data"""
        try:
            self.current_pose = {
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
            # Only log occasionally to avoid spam
            if not hasattr(self, '_pose_log_count'):
                self._pose_log_count = 0
            self._pose_log_count += 1
            
            if self._pose_log_count % 100 == 1:  # Log every 100th callback
                self.get_logger().debug(f"📍 Robot pose updated: ({self.current_pose['x']:.3f}, {self.current_pose['y']:.3f})")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error in odom callback: {e}")

    def scan_callback(self, msg):
        """Callback for laser scan data"""
        self.lidar_data = {
            'ranges': list(msg.ranges),
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max
        }

    def costmap_callback(self, msg):
        """Callback for global costmap"""
        self.costmap_data = msg

    def camera_callback(self, msg):
        """Store the latest camera frame for clarification prompts."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.frame_lock:
                self.latest_image = frame.copy()
                self.latest_image_stamp = msg.header.stamp
        except CvBridgeError as exc:
            self.get_logger().error(f"❌ Failed to convert camera image: {exc}")

    def detection_callback(self, msg):
        """React to YOLO detection results and trigger human clarification when needed."""
        detection = self.parse_detection_message(msg.data)
        if not detection or detection['label'] is None:
            return

        detected_label = detection['label']
        if detected_label in self.clarified_labels:
            return

        signature = self.get_detection_signature(detection)
        if signature and signature in self.clarified_signatures:
            return

        confidence = detection.get('confidence')
        if confidence is None:
            return

        if confidence < self.low_confidence_threshold:
            if self.awaiting_clarification:
                return

            self.awaiting_clarification = True
            self.get_logger().warn(
                f"⚠️ Low confidence detection ({confidence:.2f}) for label '{detection['label']}'. Awaiting human input."
            )
            try:
                threading.Thread(
                    target=self.handle_low_confidence_detection,
                    args=(detection,),
                    daemon=True
                ).start()
            except Exception as exc:
                self.awaiting_clarification = False
                self.get_logger().error(f"Failed to start clarification thread: {exc}")

    def parse_detection_message(self, data: str):
        """Convert detection string into structured data."""
        try:
            parts = [part.strip() for part in data.split(',')]
            if len(parts) != 10:
                self.get_logger().debug(f"Unexpected detection payload length: {len(parts)}")
                return None

            label = parts[0] if parts[0] not in ('None', '') else None

            def parse_float(value):
                return float(value) if value not in ('None', '') else None

            confidence = parse_float(parts[1])
            world_x = parse_float(parts[2])
            world_y = parse_float(parts[3])
            world_distance = parse_float(parts[4])
            world_angle = parse_float(parts[5])
            bbox = {
                'x_min': parse_float(parts[6]),
                'y_min': parse_float(parts[7]),
                'x_max': parse_float(parts[8]),
                'y_max': parse_float(parts[9])
            }

            return {
                'label': label,
                'confidence': confidence,
                'world_coordinates': {
                    'x': world_x,
                    'y': world_y,
                    'distance': world_distance,
                    'angle': world_angle
                },
                'bbox': bbox,
                'raw': data
            }
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse detection data: {exc}")
            return None

    def get_detection_signature(self, detection):
        """Create a signature for a detection payload to identify repeats."""
        if not detection or not isinstance(detection, dict):
            return None

        signature = self._build_signature_tuple(
            detection.get('label'),
            detection.get('world_coordinates'),
            detection.get('bbox')
        )

        if any(item is not None for item in signature):
            return signature

        raw = detection.get('raw')
        return raw.strip() if raw else None

    def _signature_from_record(self, record):
        """Build signature from a stored clarification record."""
        if not isinstance(record, dict):
            return None

        detected_label = record.get('detected_label') or record.get('label')
        signature = self._build_signature_tuple(
            detected_label,
            record.get('world_coordinates'),
            record.get('bbox')
        )

        if any(item is not None for item in signature):
            return signature

        raw = record.get('raw')
        return raw.strip() if raw else None

    def _build_signature_tuple(self, label, world_coords, bbox):
        world_coords = world_coords or {}
        bbox = bbox or {}
        return (
            label,
            self._round_or_none(world_coords.get('x')),
            self._round_or_none(world_coords.get('y')),
            self._round_or_none(world_coords.get('distance')),
            self._round_or_none(world_coords.get('angle')),
            self._int_or_none(bbox.get('x_min')),
            self._int_or_none(bbox.get('y_min')),
            self._int_or_none(bbox.get('x_max')),
            self._int_or_none(bbox.get('y_max'))
        )

    @staticmethod
    def _round_or_none(value, digits=2):
        return round(value, digits) if isinstance(value, (int, float)) else None

    @staticmethod
    def _int_or_none(value):
        if value is None:
            return None
        try:
            return int(round(float(value)))
        except (TypeError, ValueError):
            return None

    def handle_low_confidence_detection(self, detection):
        """Pause navigation and gather human clarification for a detection."""
        try:
            self.pause_navigation_for_clarification()

            frame = self.get_latest_frame_copy()
            annotated_frame = None
            if frame is not None:
                annotated_frame = self.draw_bounding_box(frame, detection)
                window_title = "Low Confidence Detection"
                cv2.imshow(window_title, annotated_frame)
                cv2.waitKey(1)
            else:
                self.get_logger().warn("No camera frame available for human clarification.")

            self.clarification_prompt_active = True
            actual_label = self.prompt_for_label(detection.get('label'))
            self.save_human_clarification(detection, actual_label)

            if annotated_frame is not None:
                try:
                    cv2.destroyWindow("Low Confidence Detection")
                except cv2.error:
                    pass

            self.get_logger().info("✅ Human clarification recorded. Resuming navigation.")
        finally:
            self.clarification_prompt_active = False
            self.resume_navigation_after_clarification()
            self.awaiting_clarification = False

    def pause_navigation_for_clarification(self):
        """Stop the robot and cancel the current goal to await human input."""
        self.get_logger().info("⏸️ Pausing navigation for human clarification...")
        self.paused_waypoint = self.current_waypoint
        self.state = NavigationState.PAUSED
        self.stop_robot_motion()

        if self.current_nav_goal:
            try:
                cancel_future = self.current_nav_goal.cancel_goal_async()
                if cancel_future:
                    rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
            except Exception as exc:
                self.get_logger().debug(f"Cancel goal exception (ignored): {exc}")
            self.current_nav_goal = None

    def resume_navigation_after_clarification(self):
        """Resume navigation flow after human feedback."""
        if self.state != NavigationState.PAUSED:
            return

        if self.exploration_active:
            self.state = NavigationState.IDLE
        else:
            self.state = NavigationState.IDLE

        if self.paused_waypoint:
            self.get_logger().info("🧭 Re-attempting waypoint after clarification.")
        self.paused_waypoint = None

    def stop_robot_motion(self):
        """Publish zero velocity to halt the robot."""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def get_latest_frame_copy(self):
        """Return a copy of the most recent camera frame if available."""
        with self.frame_lock:
            if self.latest_image is None:
                return None
            return self.latest_image.copy()

    def draw_bounding_box(self, frame, detection):
        """Annotate frame with bounding box and label when provided."""
        annotated = frame.copy()
        bbox = detection.get('bbox', {})
        if all(bbox.get(key) is not None for key in ['x_min', 'y_min', 'x_max', 'y_max']):
            x_min = int(bbox['x_min'])
            y_min = int(bbox['y_min'])
            x_max = int(bbox['x_max'])
            y_max = int(bbox['y_max'])
            cv2.rectangle(annotated, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            label = detection.get('label') or 'Unknown'
            confidence = detection.get('confidence')
            caption = f"{label} ({confidence:.2f})" if confidence is not None else label
            cv2.putText(
                annotated,
                caption,
                (x_min, max(y_min - 10, 0)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )
        else:
            cv2.putText(
                annotated,
                "No bounding box provided",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )
        return annotated

    def prompt_for_label(self, detected_label: Optional[str]):
        """Prompt the operator for the true label of the detected object."""
        try:
            message = f"\nEnter the actual object label (detected: {detected_label or 'Unknown'}):"
            print(message, flush=True)
            audio_message = f"Please enter the actual object label. The detected label is {detected_label}."
            if speak is not None:
                try:
                    speak(audio_message)
                except RuntimeError as exc:
                    self.get_logger().warn(f"TTS playback unavailable: {exc}")
                except Exception as exc:
                    self.get_logger().warn(f"TTS playback failed: {exc}")
            else:
                self.get_logger().warn(
                    "Text-to-speech playback unavailable. Ensure ElevenLabs credentials and audio dependencies are configured."
                )
            if self.stt_enabled and capture_and_transcribe is not None:
                try:
                    print("🎙️ Listening for spoken response... (speak clearly within 5 seconds)", flush=True)
                    stt_kwargs = {
                        'duration': max(self.stt_duration, 0.5),  # enforce a minimum duration
                        'sample_rate': max(self.stt_sample_rate, 8000),
                        'cleanup': not self.stt_keep_temp_recording,
                    }

                    if self.stt_language_code:
                        stt_kwargs['language_code'] = self.stt_language_code
                    if self.stt_model_id:
                        stt_kwargs['model_id'] = self.stt_model_id
                    if self.stt_api_key:
                        stt_kwargs['api_key'] = self.stt_api_key
                    if self.stt_tag_audio_events is not None:
                        stt_kwargs['tag_audio_events'] = bool(self.stt_tag_audio_events)
                    if self.stt_diarize is not None:
                        stt_kwargs['diarize'] = bool(self.stt_diarize)

                    spoken_label = capture_and_transcribe(**stt_kwargs)
                    if spoken_label:
                        print(f"📝 Transcription: {spoken_label}", flush=True)
                        confirmation = input(
                            "Press Enter to accept transcription, or type the correct label: "
                        ).strip()
                        if confirmation:
                            return confirmation
                        speak("Thanks for the clarification")
                        return spoken_label
                except RuntimeError as exc:
                    self.get_logger().warn(f"Voice-to-text unavailable: {exc}")
                except Exception as exc:
                    self.get_logger().warn(f"Voice-to-text failed: {exc}")

            response = input("Speech was not understood. Please type the correct label: ")
            speak("Thanks for the clarification")
            if not response.strip():
                return detected_label
            return response.strip()
        except EOFError:
            self.get_logger().warn("Input unavailable. Falling back to detected label.")
            return detected_label

    def save_human_clarification(self, detection, actual_label: Optional[str]):
        """Persist human clarification details to JSON file."""
        with self.clarification_lock:
            entries = []
            if os.path.exists(HUMAN_CLARIFICATION_FILE):
                try:
                    with open(HUMAN_CLARIFICATION_FILE, 'r') as file_obj:
                        existing = json.load(file_obj)
                        if isinstance(existing, list):
                            entries = existing
                        elif isinstance(existing, dict) and 'entries' in existing:
                            entries = existing['entries']
                except json.JSONDecodeError:
                    self.get_logger().warn("Human clarification JSON corrupted. Overwriting with fresh list.")

            entry_id = (entries[-1]["id"] + 1) if entries else 1
            record = {
                "id": entry_id,
                "label": actual_label,
                "confidence": detection.get('confidence'),
                "world_coordinates": detection.get('world_coordinates'),
                "bbox": detection.get('bbox'),
                "detected_label": detection.get('label'),
                "raw": detection.get('raw'),
                "recorded_at": datetime.now().isoformat()
            }
            entries.append(record)

            with open(HUMAN_CLARIFICATION_FILE, 'w') as file_obj:
                json.dump(entries, file_obj, indent=2)

            if detection.get('label'):
                self.clarified_labels.add(detection['label'])
            if actual_label:
                self.clarified_labels.add(actual_label)

            signature = self.get_detection_signature(detection)
            if signature is not None:
                self.clarified_signatures.add(signature)

    def create_pose_stamped(self, x, y, theta):
        """Create a PoseStamped message from x, y, theta"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        
        return pose

    def calculate_distance(self, pose1, pose2):
        """Calculate Euclidean distance between two poses"""
        dx = pose1['x'] - pose2['x']
        dy = pose1['y'] - pose2['y']
        return math.sqrt(dx*dx + dy*dy)

    def is_goal_reached(self, target_pose):
        """Check if the robot has reached the target pose"""
        if not self.current_pose:
            return False
        
        distance = self.calculate_distance(self.current_pose, target_pose)
        return distance < self.goal_tolerance

    def navigate_to_waypoint(self, waypoint):
        """Send navigation goal to Nav2"""
        if not self.nav_client.server_is_ready():
            self.get_logger().warn("Nav2 action server not ready, waiting...")
            return False

        # Check if robot pose is available
        if not self.current_pose:
            self.get_logger().warn("Robot pose not available yet, waiting...")
            return False

        # Skip if already at starting position (within tolerance)
        if waypoint['description'] == "Starting position" and self.current_pose:
            current_distance = math.sqrt(
                (self.current_pose['x'] - waypoint['x'])**2 + 
                (self.current_pose['y'] - waypoint['y'])**2
            )
            if current_distance < self.goal_tolerance:
                self.get_logger().info(f"Already at starting position, moving to next waypoint")
                
                # Mark waypoint as completed and move to next immediately
                waypoint_data = waypoint.copy()
                waypoint_data['reached_at'] = datetime.now().isoformat()
                waypoint_data['robot_pose'] = self.current_pose.copy() if self.current_pose else None
                waypoint_data['status'] = 'completed'
                self.visited_waypoints.append(waypoint_data)
                
                # Move to next waypoint index
                self.current_goal_index += 1
                self.save_navigation_data()
                
                # Force state back to IDLE so navigation loop picks up next waypoint immediately
                self.state = NavigationState.IDLE
                return True

        # Create goal pose for Nav2
        goal_pose = self.create_pose_stamped(
            waypoint['x'], waypoint['y'], waypoint['theta']
        )
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(
            f"🎯 Sending navigation goal to Nav2: ({waypoint['x']:.2f}, {waypoint['y']:.2f}) - {waypoint['description']}"
        )
        self.get_logger().info(
            f"📍 Current robot pose: ({self.current_pose['x']:.2f}, {self.current_pose['y']:.2f})"
        )
        
        # Send goal asynchronously to Nav2
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.navigation_feedback_callback
        )
        
        # Add result callback
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        
        self.current_waypoint = waypoint
        self.state = NavigationState.NAVIGATING
        
        return True

    def goal_response_callback(self, future):
        """Callback for goal response from Nav2"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"❌ Goal rejected by Nav2! Waypoint: {self.current_waypoint['description']}")
            self.get_logger().error("Check robot localization in RVIZ and ensure map is loaded")
            self.on_navigation_failed()
            return

        self.current_nav_goal = goal_handle
        self.get_logger().info(f"✅ Goal accepted by Nav2! Navigating to {self.current_waypoint['description']}")
        
        # Get result asynchronously
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Callback for navigation result"""
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f"🎉 Successfully reached waypoint: {self.current_waypoint['description']}")
            self.on_waypoint_reached()
        elif status == 5 and self.state == NavigationState.PAUSED:
            if not self.awaiting_clarification:
                self.get_logger().info("✅ Navigation goal canceled for human clarification.")
        else:
            self.get_logger().warn(f"⚠️ Navigation failed with status {status} for waypoint: {self.current_waypoint['description']}")
            self.on_navigation_failed()

    def navigation_feedback_callback(self, feedback_msg):
        """Callback for navigation feedback during movement"""
        feedback = feedback_msg.feedback
        if feedback and hasattr(feedback, 'distance_remaining'):
            distance = feedback.distance_remaining
            if distance > 0:
                self.get_logger().info(f"🚶 Distance remaining to waypoint: {distance:.2f}m")

    def check_navigation_result(self):
        """Check the result of the current navigation goal"""
        # This method is now handled by callbacks, but kept for compatibility
        pass

    def on_waypoint_reached(self):
        """Handle successful waypoint navigation"""
        if self.current_waypoint:
            # Add to visited waypoints
            waypoint_data = self.current_waypoint.copy()
            waypoint_data['reached_at'] = datetime.now().isoformat()
            waypoint_data['robot_pose'] = self.current_pose.copy() if self.current_pose else None
            waypoint_data['status'] = 'reached'
            
            progress = f"({len(self.visited_waypoints) + 1}/{len(self.predefined_waypoints)})"
            self.get_logger().info(
                f"🎯 Reached waypoint: {self.current_waypoint['description']} {progress}"
            )
            
            # Update waypoint status to completed
            waypoint_data['status'] = 'completed'
            waypoint_data['completed_at'] = datetime.now().isoformat()
            self.visited_waypoints.append(waypoint_data)
            
            # Move to next waypoint
            self.current_goal_index += 1
            self.current_waypoint = None
            self.current_nav_goal = None
            
            # Save progress
            self.save_navigation_data()
            
            # Check if exploration is complete
            if self.current_goal_index >= len(self.predefined_waypoints):
                self.on_exploration_complete()
            else:
                # Wait a moment before moving to next waypoint
                self.get_logger().info(f"⏳ Preparing for next waypoint...")
                time.sleep(2.0)
                self.state = NavigationState.IDLE
                self.get_logger().info(f"📍 Moving to next waypoint in sequence...")

    def on_navigation_failed(self):
        """Handle navigation failure"""
        waypoint_desc = self.current_waypoint['description'] if self.current_waypoint else "unknown"
        self.get_logger().warn(f"❌ Failed to reach waypoint: {waypoint_desc}")
        
        # Add to visited with failure status
        if self.current_waypoint:
            waypoint_data = self.current_waypoint.copy()
            waypoint_data['failed_at'] = datetime.now().isoformat()
            waypoint_data['status'] = 'failed'
            waypoint_data['robot_pose'] = self.current_pose.copy() if self.current_pose else None
            self.visited_waypoints.append(waypoint_data)
        
        # Move to next waypoint (don't stop exploration due to single failure)
        self.current_goal_index += 1
        self.current_waypoint = None
        self.current_nav_goal = None
        
        # Save progress
        self.save_navigation_data()
        
        # Check if exploration is complete
        if self.current_goal_index >= len(self.predefined_waypoints):
            self.on_exploration_complete()
        else:
            self.state = NavigationState.IDLE
            self.get_logger().info(f"🔄 Continuing to next waypoint despite failure...")

    def on_exploration_complete(self):
        """Handle completion of all waypoints"""
        self.state = NavigationState.COMPLETED
        self.exploration_active = False
        
        successful = len([w for w in self.visited_waypoints if w.get('status') != 'failed'])
        failed = len([w for w in self.visited_waypoints if w.get('status') == 'failed'])
        
        self.get_logger().info("🎉 EXPLORATION COMPLETE! 🎉")
        self.get_logger().info("="*50)
        self.get_logger().info(f"✅ Total waypoints visited: {len(self.visited_waypoints)}")
        self.get_logger().info(f"✅ Successful: {successful}")
        self.get_logger().info(f"❌ Failed: {failed}")
        self.get_logger().info("="*50)
        
        # Generate comprehensive exploration report
        self.generate_exploration_report()
        
        self.get_logger().info("🔍 Object detection data may be saved to detected_objects_vlm.json")
        self.get_logger().info("📊 Navigation report saved to exploration_report.json")

    def generate_exploration_report(self):
        """Generate a summary report of the exploration"""
        try:
            report = {
                'exploration_completed_at': datetime.now().isoformat(),
                'total_waypoints': len(self.predefined_waypoints),
                'successful_waypoints': len([w for w in self.visited_waypoints if w.get('status') != 'failed']),
                'failed_waypoints': len([w for w in self.visited_waypoints if w.get('status') == 'failed']),
                'visited_waypoints': self.visited_waypoints
            }
            
            report_file = os.path.join(PACKAGE_DIR, "exploration_report.json")
            with open(report_file, 'w') as f:
                json.dump(report, f, indent=2)
            
            self.get_logger().info(f"Exploration report saved to: {report_file}")
        except Exception as e:
            self.get_logger().error(f"Could not generate exploration report: {e}")

    def start_exploration(self):
        """Start autonomous exploration with Nav2 integration"""
        if self.state != NavigationState.IDLE:
            self.get_logger().warn(f"Cannot start exploration. Current state: {self.state.value}")
            return False
        
        if not self.predefined_waypoints:
            self.get_logger().error("No waypoints defined for exploration")
            return False

        # Check if Nav2 is ready
        if not self.nav_client.server_is_ready():
            self.get_logger().error("❌ Nav2 action server not ready!")
            self.get_logger().error("Please launch navigation2 first:")
            self.get_logger().error("ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$HOME/map_house.yaml")
            return False

        # Wait a bit for robot pose to become available
        self.get_logger().info("🔍 Checking robot pose availability...")
        
        # Wait up to 10 seconds for pose to become available
        for i in range(10):
            if self.current_pose:
                break
            self.get_logger().info(f"⏱️ Waiting for robot pose... ({i+1}/10)")
            time.sleep(1.0)
            rclpy.spin_once(self, timeout_sec=0.1)  # Process callbacks
        
        # Check if robot pose is available after waiting
        if not self.current_pose:
            self.get_logger().error("❌ Robot pose still not available after 10 seconds!")
            self.get_logger().error("Troubleshooting steps:")
            self.get_logger().error("1. Check if /odom topic is publishing: ros2 topic echo /odom")
            self.get_logger().error("2. Verify robot localization in RVIZ")
            self.get_logger().error("3. Set initial pose estimate using '2D Pose Estimate' button")
            self.get_logger().error("4. Check if TurtleBot3 simulation is running properly")
            return False
        
        # Reset exploration state
        self.exploration_active = True
        self.current_goal_index = max(0, self.current_goal_index)  # Resume from where we left off
        self.state = NavigationState.IDLE
        
        self.get_logger().info("🚀 Starting autonomous exploration!")
        self.get_logger().info(f"📋 Total waypoints to visit: {len(self.predefined_waypoints)}")
        self.get_logger().info(f"🎯 Starting from waypoint {self.current_goal_index + 1}/{len(self.predefined_waypoints)}")
        self.get_logger().info(f"📍 Robot starting position: ({self.current_pose['x']:.2f}, {self.current_pose['y']:.2f})")
        
        # Print waypoint sequence
        self.get_logger().info("📋 Waypoint sequence:")
        for i, wp in enumerate(self.predefined_waypoints):
            status = "✅" if i < self.current_goal_index else "⏳"
            self.get_logger().info(f"  {status} {i+1}. ({wp['x']:.2f}, {wp['y']:.2f}) - {wp['description']}")
        
        return True

    def stop_exploration(self):
        """Stop autonomous exploration"""
        self.exploration_active = False
        
        if self.current_nav_goal:
            # Cancel current navigation
            self.current_nav_goal.cancel_goal_async()
        
        self.state = NavigationState.IDLE
        self.get_logger().info("Exploration stopped")

    def navigation_loop(self):
        """Main navigation loop - manages waypoint sequence"""
        if not self.exploration_active:
            return
        
        if self.state == NavigationState.PAUSED:
            return

        if self.state == NavigationState.IDLE:
            # Start navigation to next waypoint
            if self.current_goal_index < len(self.predefined_waypoints):
                waypoint = self.predefined_waypoints[self.current_goal_index]
                next_wp = self.current_goal_index + 1
                total_wp = len(self.predefined_waypoints)
                
                self.get_logger().info(f"📋 Navigation Progress: {next_wp}/{total_wp}")
                self.get_logger().info(f"🎯 Next destination: {waypoint['description']}")
                
                if not self.navigate_to_waypoint(waypoint):
                    # If navigation fails to start, try next waypoint
                    self.get_logger().warn("Failed to start navigation, moving to next waypoint")
                    self.current_goal_index += 1
        
        elif self.state == NavigationState.NAVIGATING:
            # Navigation is handled by callbacks, just monitor
            if self.current_waypoint:
                # Provide periodic updates during navigation
                if hasattr(self, '_last_update_time'):
                    if time.time() - self._last_update_time > 10.0:  # Every 10 seconds
                        self.get_logger().info(f"🚶 Still navigating to: {self.current_waypoint['description']}")
                        self._last_update_time = time.time()
                else:
                    self._last_update_time = time.time()
        
        elif self.state == NavigationState.COMPLETED:
            # Exploration finished - no action needed
            pass

    def publish_status(self):
        """Publish exploration status"""
        status_msg = Bool()
        status_msg.data = self.exploration_active
        self.exploration_status_pub.publish(status_msg)

    # Service interfaces for external control
    def get_exploration_status(self):
        """Get current exploration status"""
        return {
            'active': self.exploration_active,
            'state': self.state.value,
            'current_waypoint_index': self.current_goal_index,
            'total_waypoints': len(self.predefined_waypoints),
            'visited_waypoints': len(self.visited_waypoints),
            'current_waypoint': self.current_waypoint
        }


def main(args=None):
    rclpy.init(args=args)
    
    try:
        navigation_node = AutonomousNavigationNode()
        
        # Add rotation test option - uncomment to test rotation only
        # navigation_node.get_logger().info("🧪 Running rotation test in 3 seconds...")
        # time.sleep(3.0)
        # navigation_node.test_rotation_only()
        # return
        
        # Start exploration automatically (or wait for external command)
        navigation_node.get_logger().info("Starting autonomous exploration in 5 seconds...")
        time.sleep(5.0)
        
        if navigation_node.start_exploration():
            navigation_node.get_logger().info("Autonomous exploration started!")
        else:
            navigation_node.get_logger().error("Failed to start exploration")
        
        rclpy.spin(navigation_node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in navigation node: {e}")
    finally:
        if 'navigation_node' in locals():
            navigation_node.stop_exploration()
        rclpy.shutdown()


if __name__ == '__main__':
    main()