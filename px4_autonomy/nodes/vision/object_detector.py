#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = YOLO('/data/system/yolo_models/yolov8n.pt')
        self.get_logger().info('YOLOv8 model loaded')

        # Subscribe to camera
        self.camera_sub = self.create_subscription(
            Image,
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image',
            self.image_callback,
            10
        )

        # Publish detections
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            '/vision/detections',
            10
        )

        self.get_logger().info('Object detector node started')

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO detection
        results = self.model(cv_image, verbose=False)

        # Convert to ROS Detection2DArray
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        if results[0].boxes is not None:
            for box in results[0].boxes:
                detection = Detection2D()
                detection.header = msg.header

                # Get bounding box
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                detection.bbox.center.position.x = (x1 + x2) / 2
                detection.bbox.center.position.y = (y1 + y2) / 2
                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1

                # Get class and confidence
                # Get class and confidence
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(class_id)
                hypothesis.hypothesis.score = confidence

                detection.results.append(hypothesis)
                detection_array.detections.append(detection)

        # Publish
        self.detections_pub.publish(detection_array)
        self.get_logger().debug(f'Published {len(detection_array.detections)} detections')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down object detector')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()