#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        
        # ArUco setup
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image',
            self.image_callback,
            10
        )
        
        # Publish marker pose
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        
        self.get_logger().info('ArUco detector started')
    
    def image_callback(self, msg):
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dict, parameters=self.aruco_params
        )
        
        # Draw detections
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            self.get_logger().info(f'Detected markers: {ids.flatten()}')
            
            # TODO: Calculate pose from corners (next step)
        
        # Display
        cv2.imshow('ArUco Detection', cv_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
