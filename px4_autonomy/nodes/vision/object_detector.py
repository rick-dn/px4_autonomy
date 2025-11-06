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
            '/world/default/model/x500_gimbal_0/link/camera_link/sensor/camera/image',
            # '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image',
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
        # self.get_logger().info('image callback')
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # self.get_logger().info('image callback')

        # Run YOLO detection
        results = self.model(cv_image, verbose=False)

        # Convert to ROS Detection2DArray
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        if results[0].boxes is not None:
            self.get_logger().info('detected')
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

        # Visualization
        for detection in detection_array.detections:
            x = int(detection.bbox.center.position.x)
            y = int(detection.bbox.center.position.y)
            w = int(detection.bbox.size_x)
            h = int(detection.bbox.size_y)

            cv2.rectangle(cv_image, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2), (0, 255, 0), 2)

            if detection.results:
                class_id = detection.results[0].hypothesis.class_id
                score = detection.results[0].hypothesis.score
                cv2.putText(cv_image, f'{class_id} {score:.2f}',
                            (x - w // 2, y - h // 2 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow('YOLO Detection', cv_image)
        cv2.waitKey(1)


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