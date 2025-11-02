#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleStatus

class ArmingNode(Node):
    """
    ROS2 node to arm a PX4 drone with pre-flight checks.
    Checks if drone is already armed or in flight before attempting to arm.
    """
    def __init__(self):
        super().__init__('arming_node')

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

        # Subscriber for VehicleStatus
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile
        )
        
        # State tracking
        self.vehicle_status_received = False
        self.armed_status = False
        self.in_air = False
        self.should_exit = False
        self.arm_attempts = 0
        self.max_attempts = 10
        
        # Wait for initial status before doing anything
        self.status_timeout_timer = self.create_timer(5.0, self.check_initial_status_timeout)
        self.arm_timer = None  # Will be created after we get initial status
        
        self.get_logger().info('Arming node initialized')
        self.get_logger().info('Waiting for vehicle status...')

    def check_initial_status_timeout(self):
        """Check if we received initial vehicle status."""
        if not self.vehicle_status_received:
            self.get_logger().error('No vehicle status received - Is PX4 running? Is XRCE-DDS agent running?')
            self.should_exit = True
            self.status_timeout_timer.cancel()

    def vehicle_status_callback(self, msg):
        """Process vehicle status and decide whether to arm."""
        if not self.vehicle_status_received:
            self.vehicle_status_received = True
            self.status_timeout_timer.cancel()
            
            # Check current state
            self.armed_status = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            self.in_air = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF or
                          msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER or
                          msg.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_MISSION or
                          msg.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL or
                          msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)
            
            # Check if already armed
            if self.armed_status:
                if self.in_air:
                    self.get_logger().warn('Drone is already armed and IN FLIGHT - Cannot arm again')
                else:
                    self.get_logger().info('Drone is already ARMED')
                self.should_exit = True
                return
            
            # Not armed, start arming attempts
            self.get_logger().info('Drone is disarmed - Attempting to arm...')
            self.arm_timer = self.create_timer(1.0, self.arm_attempt_callback)
            
        else:
            # Update arming status for ongoing attempts
            was_armed = self.armed_status
            self.armed_status = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            
            if self.armed_status and not was_armed:
                self.get_logger().info('Drone successfully ARMED')
                self.should_exit = True
                if self.arm_timer:
                    self.arm_timer.cancel()

    def arm_attempt_callback(self):
        """Attempt to arm the drone."""
        if self.arm_attempts >= self.max_attempts:
            self.get_logger().error(f'Failed to arm after {self.max_attempts} attempts')
            self.should_exit = True
            self.arm_timer.cancel()
            return
        
        self.arm_attempts += 1
        self.get_logger().info(f'Arm attempt {self.arm_attempts}/{self.max_attempts}')
        self.publish_arm_command()

    def publish_arm_command(self):
        """Publish the arm command."""
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0  # 1.0 to arm
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmingNode()
    
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        print("Arming node shutdown complete")

if __name__ == '__main__':
    main()
