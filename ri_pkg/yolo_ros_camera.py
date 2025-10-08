#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO

class YoloCameraNode(Node):
    def __init__(self):
        super().__init__('yolo_camera_node')

        # YOLOv8 model
        self.model = YOLO('yolov8n.pt')  # can switch to yolov8s.pt
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)
        self.get_logger().info(f'Using device: {self.device.upper()} for YOLO inference')

        # CV bridge
        self.bridge = CvBridge()

        # Subscribe to ROS2 camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # YOLO inference
            results = self.model(frame)

            # Draw bounding boxes + confidence
            annotated_frame = results[0].plot()

            # Show the frame
            cv2.imshow("YOLOv8 ROS Camera Feed", annotated_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info("Shutting down")
                cv2.destroyAllWindows()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
