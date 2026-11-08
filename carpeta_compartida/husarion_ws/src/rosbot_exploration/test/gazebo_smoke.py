"""Manual headless smoke test using this workspace's actual Gazebo/SLAM/Nav2.

Build rosbot_description and rosbot_exploration, source install/setup.bash, then
run in an isolated domain: python3 .../test/gazebo_smoke.py. No camera rendering
is needed for this LiDAR-based test. It never connects to a physical robot.
"""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import statistics

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from rcl_interfaces.msg import Log


class Probe(Node):
    def __init__(self):
        super().__init__('exploration_smoke_probe')
        self.distance = 0.0
        self.last_position = None
        self.known = []
        self.goals = 0
        self.state = ''
        self.statuses = []
        self.velocity = 0.0
        self.states = Counter()
        self.errors = Counter()
        self.terminals = {}
        self.first_goal_time = None
        self.last_arrival = None
        self.handoffs = []
        self.last_odom_time = None
        self.idle_since = None
        self.max_idle = 0.0
        self.moving_time = 0.0
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map, retained)
        self.create_subscription(Odometry, '/odom', self.odom, 10)
        self.create_subscription(PoseStamped, '/exploration/goal', self.goal, 10)
        self.create_subscription(String, '/exploration/state', self.exploration_state, retained)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status, retained)
        self.create_subscription(Twist, '/cmd_vel', self.cmd, 10)
        self.create_subscription(Log, '/rosout', self.log, 100)

    def map(self, message):
        self.known.append(int(np.count_nonzero(np.asarray(message.data) >= 0)))

    def odom(self, message):
        point = message.pose.pose.position
        if self.last_position:
            self.distance += math.hypot(point.x - self.last_position[0], point.y - self.last_position[1])
        self.last_position = (point.x, point.y)
        now = time.monotonic()
        moving = (abs(message.twist.twist.linear.x) > 0.01 or
                  abs(message.twist.twist.angular.z) > 0.03)
        if self.first_goal_time is not None:
            if moving:
                if self.last_odom_time is not None:
                    self.moving_time += min(now - self.last_odom_time, 0.5)
                if self.idle_since is not None:
                    self.max_idle = max(self.max_idle, now - self.idle_since)
                    self.idle_since = None
            elif self.idle_since is None:
                self.idle_since = now
        self.last_odom_time = now

    def goal(self, message):
        self.goals += 1
        now = time.monotonic()
        if self.first_goal_time is None:
            self.first_goal_time = now
        if self.last_arrival is not None:
            self.handoffs.append(now - self.last_arrival)
            self.last_arrival = None

    def exploration_state(self, message):
        self.state = message.data
        self.states[message.data] += 1
        if message.data == 'SELECTING: Reached viewpoint':
            self.last_arrival = time.monotonic()
        print(self.state, flush=True)

    def status(self, message):
        self.statuses = [s.status for s in message.status_list]
        for item in message.status_list:
            if item.status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                self.terminals[bytes(item.goal_info.goal_id.uuid).hex()] = item.status

    def cmd(self, message):
        self.velocity = abs(message.linear.x) + abs(message.angular.z)

    def log(self, message):
        if message.level >= 40:
            key = 'send_goal failed' if 'send_goal failed' in message.msg else message.name + ': ' + message.msg[:100]
            self.errors[key] += 1

    def metrics(self):
        now = time.monotonic()
        idle = max(self.max_idle, now - self.idle_since if self.idle_since else 0.0)
        elapsed = now - self.first_goal_time if self.first_goal_time else 0.0
        return dict(goals=self.goals, travelled_m=round(self.distance, 3),
                    known_initial=self.known[0] if self.known else 0,
                    known_max=max(self.known) if self.known else 0,
                    moving_fraction=round(self.moving_time / elapsed, 3) if elapsed else 0.0,
                    max_idle_s=round(idle, 3),
                    median_handoff_s=round(statistics.median(self.handoffs), 3) if self.handoffs else None,
                    succeeded=sum(s == GoalStatus.STATUS_SUCCEEDED for s in self.terminals.values()),
                    aborted=sum(s == GoalStatus.STATUS_ABORTED for s in self.terminals.values()),
                    states=dict(self.states), errors=dict(self.errors))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=0,
                        help='Run for this many wall seconds instead of stopping after two goals')
    parser.add_argument('--report-json', type=Path)
    args = parser.parse_args()
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0')
    rclpy.init()
    probe = Probe()
    processes = []
    with tempfile.TemporaryDirectory(prefix='exploration_gazebo_') as directory:
        logs = []

        def start(label, command):
            log = Path(directory) / (label + '.log')
            stream = log.open('w')
            process = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            processes.append((process, stream))
            logs.append(log)
            return process

        try:
            start('gazebo', ['ros2', 'launch', 'rosbot_description', 'rosbot_sim.launch.py',
                             'use_sim_time:=true', 'gui:=false', 'camera:=false'])
            start('slam', ['ros2', 'launch', 'rosbot_description', 'slam_toolbox_sim.launch.py'])
            start('nav2', ['ros2', 'launch', 'rosbot_description', 'rosbot_navigation_sim.launch.py'])
            explorer = start('explore', ['ros2', 'launch', 'rosbot_exploration', 'exploration.launch.py',
                                         'use_sim_time:=true', 'autostart:=true'])
            deadline = time.monotonic() + (args.duration if args.duration else 150.0)
            next_report = 0.0
            while time.monotonic() < deadline:
                rclpy.spin_once(probe, timeout_sec=0.1)
                if time.monotonic() >= next_report:
                    print('state={} goals={} distance={:.2f} known={}'.format(
                        probe.state, probe.goals, probe.distance,
                        probe.known[-1] if probe.known else 0), flush=True)
                    next_report = time.monotonic() + 5
                if not args.duration and probe.goals >= 2 and probe.distance >= 0.5:
                    break
                if any(process.poll() is not None for process, _ in processes):
                    raise AssertionError('A simulation process exited unexpectedly')
            assert probe.goals >= 2 and probe.distance >= 0.5, 'No autonomous progression'
            metrics = probe.metrics()
            explorer.send_signal(signal.SIGINT)
            cancel_deadline = time.monotonic() + 15
            while time.monotonic() < cancel_deadline:
                rclpy.spin_once(probe, timeout_sec=0.1)
                active = any(s in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                                  GoalStatus.STATUS_CANCELING) for s in probe.statuses)
                if explorer.poll() is not None and not active and probe.velocity < 0.001:
                    break
            assert explorer.poll() == 0, 'Exploration did not exit cleanly'
            assert not active and probe.velocity < 0.001, 'Navigation continued after closing exploration'
            assert max(probe.known) > probe.known[0], 'Map did not grow'
            metrics['cancellation_confirmed'] = True
            print('METRICS ' + json.dumps(metrics, sort_keys=True), flush=True)
            if args.report_json:
                args.report_json.write_text(json.dumps(metrics, indent=2) + '\n')
            print('PASS: goals={} travelled={:.2f}m known_cells={} -> {}; closing exploration stopped Nav2'.format(
                metrics['goals'], metrics['travelled_m'], metrics['known_initial'],
                metrics['known_max']), flush=True)
        except BaseException:
            for log in logs:
                print('\n{}:\n{}'.format(log.name, '\n'.join(log.read_text().splitlines()[-30:])), flush=True)
            raise
        finally:
            for process, stream in reversed(processes):
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                stream.close()
            probe.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
