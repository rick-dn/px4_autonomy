#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import TrajectorySetpoint, VehicleStatus

class VelocityController:
    """
    Velocity-based control using TrajectorySetpoint messages.
    Pure control logic - no service callbacks, no node inheritance.
    """

    def __init__(self, node: Node):
        """
        Args:
            node: Parent ROS2 node
        """
        self.node = node

        # QoS profile for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher for trajectory setpoints (velocity mode)
        self.trajectory_setpoint_pub = self.node.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        # Subscribe to vehicle status for auto-stop on disarm
        self.vehicle_status_sub = self.node.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile
        )

        # Declare parameters for velocity
        self.node.declare_parameter('vx', 0.0)
        self.node.declare_parameter('vy', 0.0)
        self.node.declare_parameter('vz', 0.0)
        self.node.declare_parameter('vyaw', 0.0)

        # Target velocity (NED frame)
        self.target_velocity = [0.0, 0.0, 0.0]
        self.target_yawspeed = 0.0

        # Control flag
        self.control_active = False

        # Timer for publishing setpoints (10Hz)
        self.setpoint_timer = self.node.create_timer(0.1, self.publish_setpoint)

        self.node.get_logger().info('Velocity Controller initialized')

    def vehicle_status_callback(self, msg):
        """Stop velocity control when disarmed"""
        if msg.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
            if self.control_active:
                self.control_active = False
                self.node.get_logger().info('Velocity control stopped - vehicle disarmed')

    def publish_setpoint(self):
        """Publish velocity setpoint at 10Hz"""
        if not self.control_active:
            return

        msg = TrajectorySetpoint()
        msg.position = [float('nan')] * 3
        msg.velocity = self.target_velocity
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yaw = float('nan')
        msg.yawspeed = self.target_yawspeed
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)

        self.trajectory_setpoint_pub.publish(msg)

    def set_velocity(self, vx, vy, vz, yawspeed=0.0):
        """
        Set velocity command

        Args:
            vx: North velocity (m/s)
            vy: East velocity (m/s)
            vz: Down velocity (m/s, negative = up)
            yawspeed: Yaw rate (rad/s)
        """
        self.target_velocity = [vx, vy, vz]
        self.target_yawspeed = yawspeed
        self.control_active = True
        self.node.get_logger().info(f'Setting velocity: vx={vx}, vy={vy}, vz={vz}, yawspeed={yawspeed}')

    def start_callback(self, request, response):
        """Service callback to start velocity control with parameters"""
        vx = self.node.get_parameter('vx').value
        vy = self.node.get_parameter('vy').value
        vz = self.node.get_parameter('vz').value
        vyaw = self.node.get_parameter('vyaw').value

        self.set_velocity(vx, vy, vz, vyaw)
        response.success = True
        response.message = f"Velocity control started: vx={vx}, vy={vy}, vz={vz}"
        return response

    def stop_callback(self, request, response):
        """Service callback to stop velocity control"""
        self.stop()
        response.success = True
        response.message = "Velocity control stopped"
        return response

    def stop(self):
        """Stop velocity control"""
        self.target_velocity = [0.0, 0.0, 0.0]
        self.target_yawspeed = 0.0
        self.control_active = False
        self.node.get_logger().info('Velocity control stopped')