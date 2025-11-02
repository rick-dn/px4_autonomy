#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, VehicleStatus, VehicleControlMode

class OffboardModeManager(Node):
    """
    Manages offboard mode by publishing continuous heartbeat signals.
    Monitors connection health and offboard mode status.
    """
    def __init__(self):
        super().__init__('offboard_mode_manager')

        # Configure QoS profile to match PX4 bridge
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publisher for OffboardControlMode (heartbeat)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        # Subscriber for VehicleControlMode (confirms offboard active)
        self.vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.vehicle_control_mode_callback,
            qos_profile
        )

        # Subscriber for VehicleStatus (monitors nav state and arming)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )
        
        # State tracking
        self.offboard_mode_active = False
        self.armed = False
        self.nav_state = None
        self.heartbeat_count = 0
        self.last_status_time = self.get_clock().now()
        self.connection_healthy = False

        # Heartbeat timer (2Hz - required minimum for PX4)
        self.heartbeat_timer = self.create_timer(0.5, self.publish_heartbeat)
        
        # Status monitoring timer (1Hz - check connection health)
        self.monitor_timer = self.create_timer(1.0, self.monitor_connection)
        
        self.get_logger().info('🟡 Offboard mode manager initialized')
        self.get_logger().info('📡 Starting offboard heartbeat at 2Hz...')

    def vehicle_control_mode_callback(self, msg):
        """Monitor if offboard mode is active."""
        was_offboard = self.offboard_mode_active
        self.offboard_mode_active = msg.flag_control_offboard_enabled
        self.last_status_time = self.get_clock().now()
        self.connection_healthy = True
        
        # Log status changes
        if self.offboard_mode_active and not was_offboard:
            self.get_logger().info('✅ Drone entered OFFBOARD mode')
        elif not self.offboard_mode_active and was_offboard:
            self.get_logger().warn('⚠️  Drone exited OFFBOARD mode')

    def vehicle_status_callback(self, msg):
        """Monitor arming state and navigation state."""
        was_armed = self.armed
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state
        self.last_status_time = self.get_clock().now()
        self.connection_healthy = True
        
        # Log arming changes
        if self.armed and not was_armed:
            self.get_logger().info('🔓 Drone ARMED')
        elif not self.armed and was_armed:
            self.get_logger().info('🔒 Drone DISARMED')

    def publish_heartbeat(self):
        """Publish offboard control mode heartbeat."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.offboard_control_mode_publisher.publish(msg)
        self.heartbeat_count += 1
        
        # Log periodic status (every 10 heartbeats = 5 seconds)
        if self.heartbeat_count % 10 == 0:
            status = '✅ ACTIVE' if self.offboard_mode_active else '⏳ WAITING'
            armed_status = 'ARMED' if self.armed else 'DISARMED'
            self.get_logger().info(
                f'💓 Heartbeat #{self.heartbeat_count} | '
                f'Offboard: {status} | {armed_status}'
            )

    def monitor_connection(self):
        """Check if we're still receiving status messages."""
        time_since_status = (self.get_clock().now() - self.last_status_time).nanoseconds / 1e9
        
        # If no status for 3 seconds, connection might be lost
        if time_since_status > 3.0:
            if self.connection_healthy:
                self.get_logger().error(
                    '❌ No vehicle status received for 3s - Connection lost?'
                )
                self.get_logger().warn('   Check: PX4 running? XRCE-DDS agent running?')
                self.connection_healthy = False
        else:
            if not self.connection_healthy:
                self.get_logger().info('✅ Connection restored')
                self.connection_healthy = True

def main(args=None):
    rclpy.init(args=args)
    node = OffboardModeManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Shutting down offboard mode manager...')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Offboard mode manager has been shut down.")

if __name__ == '__main__':
    main()
