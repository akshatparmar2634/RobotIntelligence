#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        
        # Create subscription to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Initialize CV bridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()
        
        self.get_logger().info('Camera subscriber node started. Listening to /camera/image_raw')
        self.get_logger().info('Press "q" in the image window to quit')
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Display the image
            cv2.imshow("TurtleBot3 Camera Feed", cv_image)
            
            # Wait for keypress (1ms timeout)
            key = cv2.waitKey(1) & 0xFF
            
            # Exit if 'q' is pressed
            if key == ord('q'):
                self.get_logger().info('Shutting down camera subscriber')
                cv2.destroyAllWindows()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


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