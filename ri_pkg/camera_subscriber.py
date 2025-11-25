#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Subscribe only to compressed camera topic for real TurtleBot3
        self.compressed_subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.compressed_image_callback,
            1
        )
        
        # Initialize CV bridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()
        
        self.get_logger().info('Camera subscriber node started. Listening to /camera/image_raw/compressed')
        self.get_logger().info('Press "q" in the image window to quit')
        
    def compressed_image_callback(self, msg):
        try:
            # Convert compressed ROS Image message to OpenCV format
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            
            # Display the image 
            cv2.imshow("TurtleBot3 Camera Feed (Compressed)", cv_image)
            
            # Wait for keypress (1ms timeout)
            key = cv2.waitKey(1) & 0xFF
            
            # Exit if 'q' is pressed
            if key == ord('q'):
                self.get_logger().info('Shutting down camera subscriber')
                cv2.destroyAllWindows()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_subscriber = CameraSubscriber()
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()