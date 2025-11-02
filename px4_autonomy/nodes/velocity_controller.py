#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # Services
        self.start_service = self.create_service(
            Trigger, '/velocity/start', self.start_callback)
        self.stop_service = self.create_service(
            Trigger, '/velocity/stop', self.stop_callback)

        # Subscribe to velocity commands
        self.velocity_sub = self.create_subscription(
            Twist, '/velocity/cmd_vel', self.velocity_callback, 10)

        # Parameters
        self.declare_parameter('vx', 0.0)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('vz', 0.0)
        self.declare_parameter('vyaw', 0.0)

        # State
        self.velocity_control_active = False
        self.target_velocity = [0.0, 0.0, 0.0]
        self.target_yaw_rate = 0.0

        # Timers
        self.heartbeat_timer = self.create_timer(0.02, self.publish_heartbeat)
        self.setpoint_timer = self.create_timer(0.1, self.publish_velocity_setpoint)

        self.get_logger().info('Velocity controller initialized')

    def velocity_callback(self, msg):
        """Update velocity from Twist messages"""
        self.target_velocity = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.target_yaw_rate = msg.angular.z

    def publish_heartbeat(self):
        """Publish offboard control mode for velocity"""
        if not self.velocity_control_active:
            return

        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_velocity_setpoint(self):
        """Publish velocity setpoint"""
        if not self.velocity_control_active:
            return

        # Get velocity from parameters or topic
        vx = self.get_parameter('vx').value
        vy = self.get_parameter('vy').value
        vz = self.get_parameter('vz').value
        vyaw = self.get_parameter('vyaw').value

        # Use topic values if available
        if any(self.target_velocity):
            vx, vy, vz = self.target_velocity
            vyaw = self.target_yaw_rate

        msg = TrajectorySetpoint()
        msg.position = [float('nan')] * 3
        msg.velocity = [vx, vy, vz]
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yaw = float('nan')
        msg.yawspeed = vyaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def start_callback(self, request, response):
        """Start velocity control mode"""
        self.velocity_control_active = True
        self.get_logger().info('Velocity control STARTED')
        response.success = True
        response.message = 'Velocity control activated'
        return response

    def stop_callback(self, request, response):
        """Stop velocity control mode"""
        self.velocity_control_active = False
        self.target_velocity = [0.0, 0.0, 0.0]
        self.target_yaw_rate = 0.0
        self.get_logger().info('Velocity control STOPPED')
        response.success = True
        response.message = 'Velocity control deactivated'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down velocity controller')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()