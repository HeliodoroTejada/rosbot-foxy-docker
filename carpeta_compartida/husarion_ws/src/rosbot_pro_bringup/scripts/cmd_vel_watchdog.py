#!/usr/bin/env python3
"""Local velocity timeout; firmware must independently handle process/power loss.

Twist has no source timestamp: this detects time since receipt, not network age.
Keep this process beside the base driver, never on the remote navigation PC.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy


class CommandVelocityWatchdog(Node):
    def __init__(self, **kwargs):
        super().__init__('cmd_vel_watchdog', **kwargs)
        self.declare_parameter('command_timeout', 0.5)
        self.timeout = self.get_parameter('command_timeout').value
        self.validate_timeout(self.timeout)
        self.last_command_time = None
        # VOLATILE deliberately does not request commands retained before startup.
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(Twist, '/cmd_vel_safe', qos)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.on_command, qos)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(
            0.02, self.check_timeout, clock=self.steady_clock)
        self.add_on_set_parameters_callback(self.set_parameters_checked)
        self.stop()

    @staticmethod
    def validate_timeout(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError('command_timeout must be a number')
        if not math.isfinite(value) or value <= 0:
            raise ValueError('command_timeout must be finite and greater than zero')

    def set_parameters_checked(self, parameters):
        for parameter in parameters:
            if parameter.name == 'command_timeout':
                try:
                    self.validate_timeout(parameter.value)
                except ValueError as error:
                    return SetParametersResult(successful=False, reason=str(error))
        for parameter in parameters:
            if parameter.name == 'command_timeout':
                self.timeout = parameter.value
        return SetParametersResult(successful=True)

    def on_command(self, command):
        values = (command.linear.x, command.linear.y, command.linear.z,
                  command.angular.x, command.angular.y, command.angular.z)
        if not all(math.isfinite(value) for value in values):
            self.last_command_time = None
            self.stop()
            self.get_logger().warning('Rejected non-finite velocity command')
            return
        self.last_command_time = time.monotonic()
        self.publisher.publish(command)

    def check_timeout(self):
        if (self.last_command_time is None or
                time.monotonic() - self.last_command_time >= self.timeout):
            self.stop()

    def stop(self):
        self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CommandVelocityWatchdog()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            # Best effort only: SIGKILL, process failure and power loss bypass this.
            if rclpy.ok():
                node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
