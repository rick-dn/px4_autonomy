#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition
)
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
import math

class VehicleInterface(Node):
    """
    Centralized flight controller for PX4 offboard control.
    Manages heartbeat, state transitions, and provides high-level flight services.
    """
    def __init__(self):
        super().__init__('vehicle_interface')

        # QoS profile for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers to PX4
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Subscribers from PX4
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback, qos_profile)
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback, qos_profile)

        # State publishers for monitoring
        self.state_pub = self.create_publisher(PoseStamped, '/vehicle/state', 10)

        # Services
        self.arm_service = self.create_service(Trigger, '/vehicle/arm', self.arm_callback)
        self.disarm_service = self.create_service(Trigger, '/vehicle/disarm', self.disarm_callback)
        self.offboard_service = self.create_service(
            Trigger, '/vehicle/set_offboard_mode', self.offboard_callback)
        self.hover_service = self.create_service(Trigger, '/vehicle/hover', self.hover_callback)
        self.takeoff_service = self.create_service(Trigger, '/vehicle/takeoff', self.takeoff_service_callback)
        self.goto_service = self.create_service(Trigger, '/vehicle/goto', self.goto_service_callback)
        self.land_service = self.create_service(Trigger, '/vehicle/land', self.land_callback)

        # Declare parameters for commands
        self.declare_parameter('takeoff_altitude', 2.5)
        self.declare_parameter('goto_x', 0.0)
        self.declare_parameter('goto_y', 0.0)
        self.declare_parameter('goto_z', 2.5)
        self.declare_parameter('goto_yaw', 0.0)

        # Internal state
        self.armed = False
        self.offboard_mode = False
        self.current_position = [0.0, 0.0, 0.0]
        self.current_yaw = 0.0
        self.target_position = [0.0, 0.0, 0.0]
        self.target_yaw = 0.0
        self.connection_active = False

        # Control flags
        self.heartbeat_active = True
        self.position_control_active = False

        # Timers
        self.heartbeat_timer = self.create_timer(0.02, self.publish_heartbeat)  # 50Hz
        self.setpoint_timer = self.create_timer(0.1, self.publish_setpoint)  # 10Hz
        self.state_timer = self.create_timer(0.2, self.publish_state)  # 5Hz

        self.get_logger().info('Vehicle interface initialized')

    def vehicle_status_callback(self, msg):
        """Update vehicle status"""
        was_armed = self.armed
        was_offboard = self.offboard_mode
        
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard_mode = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.connection_active = True

        # Log state changes only
        if self.armed and not was_armed:
            self.get_logger().info('Vehicle ARMED')
        elif not self.armed and was_armed:
            self.get_logger().info('Vehicle DISARMED')
        
        if self.offboard_mode and not was_offboard:
            self.get_logger().info('Offboard mode ACTIVE')
        elif not self.offboard_mode and was_offboard:
            self.get_logger().warn('Offboard mode LOST')

    def local_position_callback(self, msg):
        """Update local position and heading"""
        self.current_position = [msg.x, msg.y, msg.z]
        self.current_yaw = msg.heading
        self.connection_active = True

    def publish_heartbeat(self):
        """Publish offboard control mode heartbeat"""
        if not self.heartbeat_active:
            return

        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_setpoint(self):
        """Publish trajectory setpoint"""
        if not self.position_control_active:
            return

        msg = TrajectorySetpoint()
        msg.position = self.target_position
        msg.yaw = self.target_yaw
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.jerk = [float('nan')] * 3
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_state(self):
        """Publish vehicle state for monitoring"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.current_position[0]
        msg.pose.position.y = self.current_position[1]
        msg.pose.position.z = -self.current_position[2]  # Convert NED to normal
        
        # Convert yaw to quaternion (simplified, yaw only)
        msg.pose.orientation.z = math.sin(self.current_yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.current_yaw / 2.0)
        
        self.state_pub.publish(msg)

    def send_vehicle_command(self, command, **params):
        """Send vehicle command to PX4"""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param7 = params.get('param7', 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm_callback(self, request, response):
        """Service to arm the vehicle"""
        if self.armed:
            response.success = True
            response.message = 'Vehicle already armed'
            return response

        self.get_logger().info('Arming vehicle...')
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        
        response.success = True
        response.message = 'Arm command sent'
        return response

    def disarm_callback(self, request, response):
        """Service to disarm the vehicle"""
        if not self.armed:
            response.success = True
            response.message = 'Vehicle already disarmed'
            return response

        self.get_logger().info('Disarming vehicle...')
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        
        response.success = True
        response.message = 'Disarm command sent'
        return response

    def offboard_callback(self, request, response):
        """Service to switch to offboard mode"""
        if self.offboard_mode:
            response.success = True
            response.message = 'Already in offboard mode'
            return response

        self.get_logger().info('Switching to offboard mode...')
        self.send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        
        response.success = True
        response.message = 'Offboard mode command sent'
        return response

    def hover_callback(self, request, response):
        """Service to hold current position"""
        self.get_logger().info('Holding position')
        self.target_position = self.current_position.copy()
        self.target_yaw = self.current_yaw
        self.position_control_active = True
        
        response.success = True
        response.message = 'Hovering at current position'
        return response

    def takeoff_service_callback(self, request, response):
        """Service to takeoff using parameter"""
        altitude = self.get_parameter('takeoff_altitude').value
        self.takeoff(altitude)
        
        response.success = True
        response.message = f'Takeoff to {altitude}m initiated'
        return response

    def goto_service_callback(self, request, response):
        """Service to goto position using parameters"""
        x = self.get_parameter('goto_x').value
        y = self.get_parameter('goto_y').value
        z = self.get_parameter('goto_z').value
        yaw = self.get_parameter('goto_yaw').value
        
        self.goto_position(x, y, z, yaw)
        
        response.success = True
        response.message = f'Going to position ({x}, {y}, {z})'
        return response

    def takeoff(self, altitude):
        """Takeoff to specified altitude"""
        self.get_logger().info(f'Taking off to {altitude}m')
        self.target_position = [
            self.current_position[0],
            self.current_position[1],
            -abs(altitude)  # NED frame
        ]
        self.target_yaw = self.current_yaw
        self.position_control_active = True

    def goto_position(self, x, y, z, yaw=None):
        """Go to specified position"""
        self.get_logger().info(f'Going to position: x={x}, y={y}, z={z}')
        self.target_position = [x, y, -abs(z)]  # NED frame
        self.target_yaw = yaw if yaw is not None else self.current_yaw
        self.position_control_active = True

    def hover(self):
        """Hold current position"""
        self.get_logger().info('Holding position')
        self.target_position = self.current_position.copy()
        self.target_yaw = self.current_yaw
        self.position_control_active = True

    def land_callback(self, request, response):
        """Service to land using PX4's land command"""
        self.get_logger().info('Landing...')

        # Stop position control
        self.position_control_active = False

        # Send land command
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

        response.success = True
        response.message = 'Land command sent'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = VehicleInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down vehicle interface')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
