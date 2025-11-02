#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand, VehicleStatus, VehicleLocalPosition

class TakeoffNode(Node):
    """
    ROS2 node to takeoff PX4 drone using AUTO mode.
    Performs pre-flight checks before takeoff.
    """
    def __init__(self):
        super().__init__('takeoff_node')

        # Declare parameters
        self.declare_parameter('takeoff_altitude', 2.5)  # meters relative gain
        self.takeoff_height = self.get_parameter('takeoff_altitude').value

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher for VehicleCommand
        self.command_publisher = self.create_publisher(
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

        # Subscriber for VehicleLocalPosition
        self.local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.local_position_callback,
            qos_profile
        )
        
        # State tracking
        self.status_received = False
        self.position_received = False
        self.armed = False
        self.initial_altitude = None
        self.current_altitude = None
        self.target_altitude = None
        self.takeoff_command_sent = False
        self.should_exit = False
        
        # Timers
        self.status_timeout_timer = self.create_timer(5.0, self.check_initial_status_timeout)
        self.monitor_timer = None
        
        self.get_logger().info('Takeoff node initialized')
        self.get_logger().info(f'Target takeoff height: {self.takeoff_height}m')
        self.get_logger().info('Waiting for vehicle status...')

    def check_initial_status_timeout(self):
        """Check if we received initial status."""
        if not self.status_received:
            self.get_logger().error('No vehicle status received - Is PX4 running? Is XRCE-DDS agent running?')
            self.should_exit = True
            self.status_timeout_timer.cancel()

    def vehicle_status_callback(self, msg):
        """Process vehicle status for pre-takeoff checks."""
        if not self.status_received:
            self.status_received = True
            self.status_timeout_timer.cancel()
            
            # Check if armed
            self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            if not self.armed:
                self.get_logger().error('Drone is NOT armed - Arm the drone first')
                self.should_exit = True
                return
            
            # Check if already in air (common nav states for flight)
            in_air_states = [
                VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF,
                VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
                VehicleStatus.NAVIGATION_STATE_AUTO_MISSION,
                VehicleStatus.NAVIGATION_STATE_OFFBOARD
            ]
            
            if msg.nav_state in in_air_states:
                self.get_logger().warn('Drone appears to be already in flight or taking off')
                self.should_exit = True
                return
            
            self.get_logger().info('Pre-takeoff checks passed: Drone is armed and on ground')
            self.get_logger().info('Waiting for position data...')

    def local_position_callback(self, msg):
        """Process local position for takeoff monitoring."""
        # Capture initial altitude once
        if self.initial_altitude is None and self.status_received and not self.should_exit:
            self.initial_altitude = -msg.z  # NED frame: negative z is up
            # Calculate target altitude (absolute value in NED frame)
            self.target_altitude = -(self.initial_altitude + self.takeoff_height)
            
            self.get_logger().info(f'Initial altitude: {self.initial_altitude:.2f}m')
            self.get_logger().info(f'Target altitude: {self.initial_altitude + self.takeoff_height:.2f}m')
            self.get_logger().info('Sending takeoff command...')
            
            # Send takeoff command
            self.send_takeoff_command()
            self.takeoff_command_sent = True
            
            # Start monitoring progress
            self.monitor_timer = self.create_timer(1.0, self.monitor_progress)
        
        # Update current altitude
        self.current_altitude = -msg.z

    def send_takeoff_command(self):
        """Send takeoff command to PX4."""
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        msg.param7 = self.target_altitude  # Target altitude in NED frame
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.command_publisher.publish(msg)
        self.get_logger().info('Takeoff command sent')

    def monitor_progress(self):
        """Monitor takeoff progress."""
        if self.current_altitude is None:
            return
        
        current_gain = self.current_altitude - self.initial_altitude
        progress = (current_gain / self.takeoff_height) * 100
        
        self.get_logger().info(
            f'Takeoff progress: {progress:.1f}% '
            f'(Current: {self.current_altitude:.2f}m, Gain: {current_gain:.2f}m)'
        )
        
        # Check if takeoff complete (95% of target)
        if current_gain >= self.takeoff_height * 0.95:
            self.get_logger().info('Takeoff complete!')
            self.should_exit = True
            if self.monitor_timer:
                self.monitor_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffNode()
    
    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        print("Takeoff node shutdown complete")

if __name__ == '__main__':
    main()
