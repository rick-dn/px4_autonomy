#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleControlMode

class ModeSwitchNode(Node):
    """
    ROS2 node to switch PX4 drone to offboard mode.
    Checks if already in offboard mode before attempting switch.
    """
    def __init__(self):
        super().__init__('mode_switch_node')

        # Configure QoS profile to match PX4 bridge
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher for VehicleCommand
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # Subscriber for VehicleControlMode
        self.vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.vehicle_control_mode_callback,
            qos_profile
        )
        
        # State tracking
        self.control_mode_received = False
        self.offboard_mode_active = False
        self.should_exit = False
        self.switch_attempts = 0
        self.max_attempts = 10
        
        # Wait for initial status
        self.status_timeout_timer = self.create_timer(5.0, self.check_initial_status_timeout)
        self.mode_switch_timer = None
        
        self.get_logger().info('Mode switch node initialized')
        self.get_logger().info('Waiting for vehicle control mode status...')

    def check_initial_status_timeout(self):
        """Check if we received initial control mode status."""
        if not self.control_mode_received:
            self.get_logger().error('No control mode status received - Is PX4 running? Is XRCE-DDS agent running?')
            self.should_exit = True
            self.status_timeout_timer.cancel()

    def vehicle_control_mode_callback(self, msg):
        """Process control mode status and decide whether to switch to offboard."""
        if not self.control_mode_received:
            self.control_mode_received = True
            self.status_timeout_timer.cancel()
            
            # Check if already in offboard mode
            self.offboard_mode_active = msg.flag_control_offboard_enabled
            
            if self.offboard_mode_active:
                self.get_logger().info('Drone is already in OFFBOARD mode')
                self.should_exit = True
                return
            
            # Not in offboard, start switching attempts
            self.get_logger().info('Drone not in offboard mode - Attempting to switch...')
            self.mode_switch_timer = self.create_timer(1.0, self.mode_switch_attempt_callback)
            
        else:
            # Update offboard status for ongoing attempts
            was_offboard = self.offboard_mode_active
            self.offboard_mode_active = msg.flag_control_offboard_enabled
            
            if self.offboard_mode_active and not was_offboard:
                self.get_logger().info('Successfully switched to OFFBOARD mode')
                self.should_exit = True
                if self.mode_switch_timer:
                    self.mode_switch_timer.cancel()

    def mode_switch_attempt_callback(self):
        """Attempt to switch to offboard mode."""
        if self.switch_attempts >= self.max_attempts:
            self.get_logger().error(f'Failed to switch to offboard mode after {self.max_attempts} attempts')
            self.get_logger().warn('Make sure: 1) Drone is armed, 2) Heartbeat is running')
            self.should_exit = True
            self.mode_switch_timer.cancel()
            return
        
        self.switch_attempts += 1
        self.get_logger().info(f'Mode switch attempt {self.switch_attempts}/{self.max_attempts}')
        self.publish_offboard_mode_command()

    def publish_offboard_mode_command(self):
        """Publish command to switch to offboard mode."""
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # Mode
        msg.param2 = 6.0  # Custom mode - 6 is offboard
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ModeSwitchNode()
    
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        print("Mode switch node shutdown complete")

if __name__ == '__main__':
    main()
