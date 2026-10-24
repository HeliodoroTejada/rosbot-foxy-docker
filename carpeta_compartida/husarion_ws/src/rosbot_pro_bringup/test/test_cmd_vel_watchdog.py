"""Fake command source/base: no robot driver, /clock, or public velocity topics."""
import importlib.util
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy


spec = importlib.util.spec_from_file_location(
    'cmd_vel_watchdog', Path(__file__).parents[1] / 'scripts' / 'cmd_vel_watchdog.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_timeout_uses_wall_time_and_does_not_replay_latched_command():
    rclpy.init()
    fake_base = Node('watchdog_fake_base')
    executor = SingleThreadedExecutor()
    executor.add_node(fake_base)
    received = []
    fake_base.create_subscription(
        Twist, '/_watchdog_test_output',
        lambda message: received.append((time.monotonic(), message)), 10)
    source = fake_base.create_publisher(
        Twist, '/_watchdog_test_input',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(0.01, deadline - time.monotonic()))

    watchdog = None
    try:
        command = Twist()
        command.linear.x = 0.17
        command.angular.z = -0.23
        source.publish(command)  # Stored before watchdog startup; must not replay.
        spin_for(0.1)
        watchdog = module.CommandVelocityWatchdog(
            parameter_overrides=[Parameter('use_sim_time', value=True),
                                 Parameter('command_timeout', value=0.3)],
            cli_args=['--ros-args', '-r', '/cmd_vel:=/_watchdog_test_input',
                      '-r', '/cmd_vel_safe:=/_watchdog_test_output'])
        executor.add_node(watchdog)
        spin_for(0.5)
        assert watchdog.get_clock().now().nanoseconds == 0  # No /clock publisher.
        assert received, 'Steady timer must publish stop even without /clock'
        assert all(message.linear.x == 0.0 for _, message in received)
        received.clear()
        source.publish(command)
        spin_for(0.15)
        forwarded = [(stamp, msg) for stamp, msg in received if msg.linear.x == 0.17]
        assert forwarded, 'A fresh command must pass through unchanged'
        assert forwarded[-1][1].angular.z == -0.23
        assert not any(msg.linear.x == 0.0 for stamp, msg in received
                       if stamp > forwarded[-1][0])
        spin_for(0.3)
        stops = [stamp for stamp, msg in received
                 if stamp > forwarded[-1][0]
                 and msg.linear.x == 0.0 and msg.angular.z == 0.0]
        assert stops, 'Lost command stream must stop in wall time'
        assert 0.25 < stops[0] - forwarded[-1][0] < 0.4
        assert not watchdog.set_parameters([
            Parameter('command_timeout', value=float('nan'))])[0].successful
        assert watchdog.timeout == 0.3
        received.clear()
        command.linear.x = float('nan')
        source.publish(command)
        spin_for(0.1)
        assert received
        assert all(msg.linear.x == 0.0 for _, msg in received)
        # Check explicit shutdown path can emit zero to the fake base.
        received.clear()
        watchdog.stop()
        spin_for(0.05)
        assert received and received[0][1].linear.x == 0.0
    finally:
        executor.shutdown()
        if watchdog is not None:
            watchdog.destroy_node()
        fake_base.destroy_node()
        rclpy.shutdown()
