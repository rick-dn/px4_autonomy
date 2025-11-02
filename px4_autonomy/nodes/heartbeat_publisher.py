#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode

class HeartbeatPublisher(Node):
    """
    Publishes continuous offboard control mode heartbeat signals to PX4.
    """
    def __init__(self):
        super().__init__('heartbeat_publisher')

        # Publisher for OffboardControlMode (heartbeat)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        # Heartbeat timer (2Hz - required minimum for PX4)
        self.heartbeat_timer = self.create_timer(0.5, self.publish_heartbeat)
        
        self.get_logger().info('Heartbeat publisher initialized')
        self.get_logger().info('Publishing offboard heartbeat at 2Hz')

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

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Suppress traceback on Ctrl+C
    finally:
        # Clean shutdown without logging (context already invalid)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        print("Heartbeat publisher shutdown complete")

if __name__ == '__main__':
    main()
