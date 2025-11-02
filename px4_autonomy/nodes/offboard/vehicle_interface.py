#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, VehicleCommand
from std_srvs.srv import Trigger
from px4_autonomy.nodes.offboard.position_controller import PositionController
from px4_autonomy.nodes.offboard.velocity_controller import VelocityController


class VehicleInterface(Node):
    """
    Service interface for PX4 offboard control.
    Arm, disarm, offboard mode, land commands.
    Position control (takeoff, goto, hover) delegated to PositionController.
    """

    def __init__(self):
        super().__init__('vehicle_interface')

        self.position_controller = PositionController(self)
        self.velocity_controller = VelocityController(self)

        # Publishers to PX4
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Services
        # PX4 services
        self.create_service(Trigger, '/vehicle/arm', self.arm_callback)
        self.create_service(Trigger, '/vehicle/disarm', self.disarm_callback)
        self.create_service(Trigger, '/vehicle/set_offboard_mode', self.offboard_callback)
        self.create_service(Trigger, '/vehicle/land',
                            lambda req, resp: self._landing_sequence(self.land_callback, req, resp))

        self.create_service(Trigger, '/vehicle/rtl',
                            lambda req, resp: self._landing_sequence(self.rtl_callback, req, resp))

        # Position control services delegated to position_controller
        self.create_service(Trigger, '/vehicle/takeoff', self.position_controller.takeoff_callback)
        self.create_service(Trigger, '/vehicle/goto',
                            lambda req, resp: self._switch_mode('POS', self.position_controller.goto_callback, req,
                                                                resp))

        self.create_service(Trigger, '/vehicle/hover',
                            lambda req, resp: self._switch_mode('POS', self.position_controller.hover_callback, req,
                                                                resp))

        # velocity conroller
        self.create_service(Trigger, '/vehicle/velocity_start',
                            lambda req, resp: self._switch_mode('VEL', self.velocity_controller.start_callback, req,
                                                                resp))

        self.create_service(Trigger, '/vehicle/velocity_stop',
                            lambda req, resp: self._switch_mode('POS', self.velocity_controller.stop_callback, req,
                                                                resp))


        # Heartbeat timer
        self.heartbeat_timer = None

        self.get_logger().info('Vehicle interface initialized')

    def publish_heartbeat(self):
        """Publish offboard control mode heartbeat"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

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
        self.get_logger().info('Arming vehicle...')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        response.success = True
        response.message = 'Arm command sent'
        return response

    def disarm_callback(self, request, response):
        """Service to disarm the vehicle"""
        self.get_logger().info('Disarming vehicle...')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        response.success = True
        response.message = 'Disarm command sent'
        return response

    def offboard_callback(self, request, response):
        """Service to switch to offboard mode"""
        self.get_logger().info('Switching to offboard mode...')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.heartbeat_timer = self.create_timer(0.02, self.publish_heartbeat)
        response.success = True
        response.message = 'Offboard mode command sent'
        return response

    def land_callback(self, request, response):
        """Service to land"""
        self.get_logger().info('Landing...')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        response.success = True
        response.message = 'Land command sent'
        return response

    def rtl_callback(self, request, response):
        """Service to return to launch"""
        self.get_logger().info('Returning to launch...')
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        response.success = True
        response.message = 'RTL command sent'
        return response

    def _landing_sequence(self, flight_command_callback, request, response):
        self.position_controller.stop()
        # self.velocity_controller.stop()
        self.heartbeat_timer.cancel()
        return flight_command_callback(request, response)

    def _publish_offboard_mode(self, position=False, velocity=False):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def _switch_mode(self, mode, flight_command_callback, request, response):
        """Switch between position and velocity control modes"""
        current_mode = 'POS' if self.position_controller.control_active else 'VEL'

        if mode == current_mode:
            # Already in this mode, just execute callback
            return flight_command_callback(request, response)

        # Mode switch needed
        if mode == 'POS':
            self.get_logger().info('Switching to position mode...')
            self.velocity_controller.stop()
            self.position_controller.hover()
            self._publish_offboard_mode(position=True, velocity=False)
        elif mode == 'VEL':
            self.get_logger().info('Switching to velocity mode...')
            self.position_controller.stop()
            self._publish_offboard_mode(position=True, velocity=False)

        return flight_command_callback(request, response)


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