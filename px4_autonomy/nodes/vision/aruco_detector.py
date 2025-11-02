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

        # Camera parameters (typical for simulated camera)
        self.camera_matrix = np.array([
            [277.0, 0.0, 320.0],
            [0.0, 277.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros((5, 1))

        # Marker size in meters (2m in gazebo)
        self.marker_size = 2.0

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image',
            self.image_callback,
            10
        )

        # Publish marker pose
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)

        self.get_logger().info('ArUco detector with pose estimation started')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None:
            # Draw markers
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Estimate pose for each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                # Draw axis
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs,
                                  rvecs[i], tvecs[i], 1.0)

                # Get position
                x, y, z = tvecs[i][0]
                distance = np.linalg.norm(tvecs[i])

                # Display info on image
                text = f"ID:{marker_id} Dist:{distance:.2f}m"
                cv2.putText(cv_image, text, (10, 30 + i * 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                self.get_logger().info(
                    f'Marker {marker_id}: x={x:.2f}, y={y:.2f}, z={z:.2f}, dist={distance:.2f}m'
                )

                # Publish pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera_link'
                pose_msg.pose.position.x = x
                pose_msg.pose.position.y = y
                pose_msg.pose.position.z = z
                self.pose_pub.publish(pose_msg)

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