#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraTest(Node):
    def __init__(self):
        super().__init__('camera_test')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image',
            self.image_callback,
            10
        )
        self.get_logger().info('Camera test node started')
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Display
        cv2.imshow('Drone Camera', cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraTest()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
