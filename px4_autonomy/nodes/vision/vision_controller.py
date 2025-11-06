#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess


class VisionController(Node):
    def __init__(self):
        super().__init__('vision_controller')

        self.processes = {}

        # Services to start/stop each node
        self.create_service(Trigger, '/vision/aruco/start', self.start_aruco)
        self.create_service(Trigger, '/vision/aruco/stop', self.stop_aruco)
        self.create_service(Trigger, '/vision/yolo/start', self.start_yolo)
        self.create_service(Trigger, '/vision/yolo/stop', self.stop_yolo)

        self.get_logger().info('Vision controller initialized')

    def start_aruco(self, request, response):
        if 'aruco' not in self.processes:
            self.processes['aruco'] = subprocess.Popen(['ros2', 'run', 'px4_autonomy', 'aruco_detector'])
            response.success = True
            response.message = 'ArUco detector started'
        else:
            response.success = False
            response.message = 'ArUco already running'
        return response

    def stop_aruco(self, request, response):
        if 'aruco' in self.processes:
            self.processes['aruco'].terminate()
            del self.processes['aruco']
            response.success = True
            response.message = 'ArUco detector stopped'
        else:
            response.success = False
            response.message = 'ArUco not running'
        return response

    def start_yolo(self, request, response):
        if 'yolo' not in self.processes:
            self.processes['yolo'] = subprocess.Popen(['ros2', 'run', 'px4_autonomy', 'object_detector'])
            response.success = True
            response.message = 'YOLO detector started'
        else:
            response.success = False
            response.message = 'YOLO already running'
        return response

    def stop_yolo(self, request, response):
        if 'yolo' in self.processes:
            self.processes['yolo'].terminate()
            del self.processes['yolo']
            response.success = True
            response.message = 'YOLO detector stopped'
        else:
            response.success = False
            response.message = 'YOLO not running'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VisionController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()