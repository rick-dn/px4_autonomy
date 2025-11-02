#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition


class PositionController:
    """
    Position control module for PX4 offboard control.
    Handles takeoff, goto, and hover commands.
    Publishes trajectory setpoints to PX4.
    """

    def __init__(self, node: Node):


        self.node = node

        # QoS profile for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher to PX4
        self.trajectory_setpoint_pub = self.node.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        # Subscriber from PX4
        self.local_position_sub = self.node.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback, qos_profile)

        # Internal state
        self.current_position = [0.0, 0.0, 0.0]
        self.current_yaw = 0.0
        self.target_position = [0.0, 0.0, 0.0]
        self.target_yaw = 0.0
        self.control_active = False

        # Declare parameters for position controller
        self.node.declare_parameter('takeoff_altitude', 2.5)
        self.node.declare_parameter('goto_x', 0.0)
        self.node.declare_parameter('goto_y', 0.0)
        self.node.declare_parameter('goto_z', 2.5)
        self.node.declare_parameter('goto_yaw', float('nan'))

        # Timer for publishing setpoints
        self.setpoint_timer = self.node.create_timer(0.1, self.publish_setpoint)  # 10Hz

        self.node.get_logger().info('Position controller initialized')

    def local_position_callback(self, msg):
        """Update local position and heading from PX4"""
        self.current_position = [msg.x, msg.y, msg.z]
        self.current_yaw = msg.heading

    def publish_setpoint(self):
        """Publish trajectory setpoint to PX4"""
        if not self.control_active:
            return

        msg = TrajectorySetpoint()
        msg.position = self.target_position
        msg.yaw = self.target_yaw
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def takeoff(self, altitude):
        """Takeoff to specified altitude"""
        self.node.get_logger().info(f'Taking off to {altitude}m')
        self.target_position = [
            self.current_position[0],
            self.current_position[1],
            -abs(altitude)  # NED frame
        ]
        self.target_yaw = self.current_yaw
        self.control_active = True

    def goto_position(self, x=None, y=None, z=None, yaw=None):
        """
        Go to specified position.
        If parameter is None, maintain current value.
        """
        new_x = x if x is not None else self.current_position[0]
        new_y = y if y is not None else self.current_position[1]
        new_z = z if z is not None else -self.current_position[2]  # Convert from NED
        new_yaw = yaw if yaw is not None else self.current_yaw

        self.node.get_logger().info(f'Going to position: x={new_x}, y={new_y}, z={new_z}')
        self.target_position = [new_x, new_y, -abs(new_z)]  # Convert to NED
        self.target_yaw = new_yaw
        self.control_active = True

    def hover(self):
        """Hold current position"""
        self.node.get_logger().info('Hovering at current position')


        self.target_position = self.current_position.copy()
        self.target_yaw = self.current_yaw
        self.control_active = True

    def stop(self):
        """Stop publishing position control setpoints"""
        self.node.get_logger().info('Stopping position control')
        self.control_active = False

    def takeoff_callback(self, request, response):
        """Service callback for takeoff"""
        altitude = self.node.get_parameter('takeoff_altitude').value if hasattr(self, 'get_parameter') else 2.5
        self.takeoff(altitude)
        response.success = True
        response.message = f'Takeoff to {altitude}m initiated'
        return response

    def goto_callback(self, request, response):
        """Service callback for goto position"""
        x = self.node.get_parameter('goto_x').value
        y = self.node.get_parameter('goto_y').value
        z = self.node.get_parameter('goto_z').value
        yaw = self.node.get_parameter('goto_yaw').value
        self.goto_position(x, y, z, yaw)
        response.success = True
        response.message = f'Going to ({x}, {y}, {z})'
        return response

    def hover_callback(self, request, response):
        """Service callback for hover"""

        self.target_position = self.current_position.copy()
        self.target_yaw = self.current_yaw

        self.hover()
        response.success = True
        response.message = 'Hovering'
        return response