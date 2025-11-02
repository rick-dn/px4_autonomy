#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import Trigger
import math


class VisualServoLanding(Node):
    def __init__(self):
        super().__init__('visual_servo_landing')

        # Subscribe to ArUco pose
        self.pose_sub = self.create_subscription(
            PoseStamped, '/aruco/pose', self.pose_callback, 10
        )

        # Publish velocity commands
        self.vel_pub = self.create_publisher(Twist, '/velocity/cmd_vel', 10)

        # Service clients
        self.velocity_start = self.create_client(Trigger, '/velocity/start')
        self.velocity_stop = self.create_client(Trigger, '/velocity/stop')
        self.land_client = self.create_client(Trigger, '/vehicle/land')

        # Service to start landing
        self.start_landing_srv = self.create_service(
            Trigger, '/visual_servo/start_landing', self.start_landing_callback
        )

        # State
        self.marker_pose = None
        self.landing_active = False
        self.last_seen_time = None

        # Control parameters
        self.kp_xy = 0.5  # Proportional gain for x,y
        self.kp_z = 0.3  # Proportional gain for descent
        self.center_threshold = 0.2  # meters
        self.landing_altitude = 0.5  # start final descent below this
        self.descent_speed = 0.2  # m/s downward

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Visual servo landing ready')

    def pose_callback(self, msg):
        self.marker_pose = msg
        self.last_seen_time = self.get_clock().now()

    def start_landing_callback(self, request, response):
        """Start the landing sequence"""
        if self.landing_active:
            response.success = False
            response.message = 'Landing already active'
            return response

        self.landing_active = True
        self.get_logger().info('Starting visual servo landing')

        # Activate velocity control
        self.velocity_start.call_async(Trigger.Request())

        response.success = True
        response.message = 'Visual servo landing started'
        return response

    def control_loop(self):
        if not self.landing_active:
            return

        # Check if marker visible
        if self.marker_pose is None:
            self.get_logger().warn('No marker detected, hovering')
            self.send_velocity(0.0, 0.0, 0.0, 0.0)
            return

        # Check marker timeout (lost for >1 second)
        time_since_seen = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
        if time_since_seen > 1.0:
            self.get_logger().warn('Marker lost, hovering')
            self.send_velocity(0.0, 0.0, 0.0, 0.0)
            return

        # Get marker position in camera frame
        x = self.marker_pose.pose.position.x
        y = self.marker_pose.pose.position.y
        z = self.marker_pose.pose.position.z

        # Calculate offset from center
        offset = math.sqrt(x ** 2 + y ** 2)

        self.get_logger().info(f'Marker: x={x:.2f}, y={y:.2f}, z={z:.2f}, offset={offset:.2f}m')

        # Check if centered and close
        if offset < self.center_threshold and z < self.landing_altitude:
            self.get_logger().info('Centered and close - LANDING!')
            self.send_velocity(0.0, 0.0, 0.0, 0.0)
            self.velocity_stop.call_async(Trigger.Request())
            self.land_client.call_async(Trigger.Request())
            self.landing_active = False
            return

        # Calculate velocity commands (proportional control)
        # Note: camera frame - x forward, y left, z into scene
        vx = -self.kp_xy * y  # Camera y → drone forward/back
        vy = -self.kp_xy * x  # Camera x → drone left/right

        # Descend if well-centered
        if offset < self.center_threshold:
            vz = self.descent_speed  # Positive = down in NED
            self.get_logger().info('Centered - descending')
        else:
            vz = 0.0  # Don't descend until centered

        # Send velocity command
        self.send_velocity(vx, vy, vz, 0.0)

    def send_velocity(self, vx, vy, vz, vyaw):
        """Send velocity command"""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = vyaw
        self.vel_pub.publish(msg)


def main():
    rclpy.init()
    node = VisualServoLanding()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()