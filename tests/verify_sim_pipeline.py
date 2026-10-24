#!/usr/bin/env python3
"""Exercise the real Foxy/Gazebo pipeline in an isolated ROS domain.

Run inside the configured container after sourcing ROS and the workspace.
The test owns every launch it starts and removes its children on exit.
It never sends goals unless its Gazebo process and rosbot model are present.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import traceback
import uuid
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from gazebo_msgs.srv import GetModelList
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan, Range
from tf2_ros import Buffer, TransformListener


class PipelineCheck(Node):
    def __init__(self, args):
        super().__init__('verify_sim_pipeline', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.args = args
        self.directory = Path(args.output)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.marker = str(uuid.uuid4())
        self.child_env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain),
                              ROS_LOCALHOST_ONLY='1',
                              GAZEBO_MASTER_URI=args.master,
                              RMW_IMPLEMENTATION=args.rmw,
                              ROSBOT_PIPELINE_TEST=self.marker)
        self.processes = {}
        self.handles = []
        self.report = {'domain': args.domain, 'master': args.master,
                       'rmw': args.rmw, 'attempts': [], 'goals': [],
                       'cyclonedds_uri': os.environ.get('CYCLONEDDS_URI'),
                       'started_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())}
        self.counts = {}
        self.latest = {}
        self.stamps = {}
        self.range_frames = set()
        self.map = None
        self.odom = None
        self.clock = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.subscriptions_owned = []
        for topic, kind in [('/scan', LaserScan), ('/odom', Odometry), ('/imu', Imu),
                            ('/range', Range)]:
            self.subscriptions_owned.append(self.create_subscription(
                kind, topic, lambda msg, name=topic: self.capture(name, msg),
                qos_profile_sensor_data))
        self.subscriptions_owned.append(self.create_subscription(
            Clock, '/clock', self.capture_clock, 10))
        self.subscriptions_owned.append(self.create_subscription(
            OccupancyGrid, '/map', self.capture_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)))
        self.service_clients = {}
        self.navigator = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def capture(self, topic, msg):
        self.counts[topic] = self.counts.get(topic, 0) + 1
        self.latest[topic] = msg
        self.stamps[topic] = time.monotonic()
        if topic == '/odom':
            self.odom = msg
        if topic == '/range':
            self.range_frames.add(msg.header.frame_id)

    def capture_map(self, msg):
        self.map = msg
        self.capture('/map', msg)

    def capture_clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.clock is not None and value < self.clock:
            self.report.setdefault('backward_clock_jumps', []).append([self.clock, value])
        self.clock = value

    def spin(self, duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0., until-time.monotonic())))

    def future(self, future, timeout):
        until = time.monotonic() + timeout
        while not future.done() and time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError('ROS request timed out after {} seconds'.format(timeout))
        return future.result()

    def client(self, service, kind):
        if service not in self.service_clients:
            self.service_clients[service] = self.create_client(kind, service)
        return self.service_clients[service]

    def request(self, service, kind, request, timeout=3.):
        client = self.client(service, kind)
        if not client.wait_for_service(timeout_sec=0.2):
            raise TimeoutError('Service unavailable: ' + service)
        return self.future(client.call_async(request), timeout)

    def start(self, label, command):
        log = self.directory / (label + '.log')
        handle = log.open('w')
        self.handles.append(handle)
        proc = subprocess.Popen(command, env=self.child_env, stdout=handle,
                                stderr=subprocess.STDOUT, start_new_session=True)
        self.processes[label] = proc
        self.report.setdefault('logs', {})[label] = str(log)
        print('START {} pid={} log={}'.format(label, proc.pid, log), flush=True)
        return proc

    def stop(self, label):
        proc = self.processes.get(label)
        if proc is None:
            return
        try:
            # Launch is noninteractive and forwards SIGINT to its nodes. Sending
            # it to the whole group would signal every node twice on shutdown.
            proc.send_signal(signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=12)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=5)
        print('STOP {} exit={}'.format(label, proc.returncode), flush=True)
        self.report.setdefault('launcher_exit_codes', {})[label] = proc.returncode

    def sim_guard(self):
        sim = self.processes.get('sim')
        if sim is None or sim.poll() is not None:
            raise RuntimeError('Own simulator is not running; movement forbidden')
        result = self.request('/get_model_list', GetModelList, GetModelList.Request(), 5.)
        if not result.success or 'rosbot' not in result.model_names:
            raise RuntimeError('Own isolated Gazebo did not confirm rosbot; movement forbidden')
        return list(result.model_names)

    def wait_sim(self):
        until = time.monotonic() + 100
        while time.monotonic() < until:
            self.spin(0.5)
            try:
                models = self.sim_guard()
                if self.odom is not None and self.counts.get('/scan', 0) > 3:
                    self.report['models'] = models
                    return
            except TimeoutError:
                pass
        raise TimeoutError('Simulator/model/scan/odom not ready')

    def wait_active(self, label):
        names = ('controller_server', 'planner_server', 'recoveries_server',
                 'bt_navigator', 'waypoint_follower')
        result = {'label': label, 'states': {}, 'started': time.monotonic()}
        self.report['attempts'].append(result)
        until = time.monotonic() + self.args.startup_timeout
        previous = None
        while time.monotonic() < until:
            self.spin(0.25)
            states = {}
            for name in names:
                try:
                    response = self.request('/' + name + '/get_state', GetState,
                                            GetState.Request(), 1.)
                    states[name] = response.current_state.label
                except TimeoutError:
                    states[name] = 'unresponsive'
            result['states'] = states
            if states != previous:
                print('LIFECYCLE {} {}'.format(label, states), flush=True)
                previous = states
            if all(state == 'active' for state in states.values()):
                result['startup_seconds'] = time.monotonic() - result.pop('started')
                return
            proc = self.processes[label]
            if proc.poll() is not None:
                raise RuntimeError(label + ' exited before activation')
        result['startup_seconds'] = time.monotonic() - result.pop('started')
        raise TimeoutError('Nav2 activation timed out: ' + repr(result['states']))

    def check_data(self):
        required_frames = ('base_link', 'laser', 'imu', 'camera_link', 'range_fl',
                           'range_fr', 'range_rl', 'range_rr')
        until = time.monotonic() + 30
        while time.monotonic() < until:
            if (self.map is not None
                    and all(self.tf.can_transform('map', frame, Time()) for frame in required_frames)
                    and all(topic in self.latest for topic in ('/scan', '/odom', '/imu', '/range'))
                    and self.range_frames == {'range_fl', 'range_fr', 'range_rl', 'range_rr'}):
                break
            self.spin(0.25)
        if self.map is None:
            raise AssertionError('No SLAM map')
        assert abs(self.map.info.resolution - 0.05) < 1e-6, self.map.info.resolution
        frames = {}
        for frame in required_frames:
            transform = self.tf.lookup_transform('map', frame, Time())
            t = transform.transform.translation
            frames[frame] = [t.x, t.y, t.z]
        sdf = Path(get_package_share_directory('rosbot_description')) / 'models' / 'rosbot.sdf'
        model = ET.parse(str(sdf)).getroot().find('model')
        sensor_poses = {}
        for frame, link in [('laser', 'laser_link'), ('imu', 'top_cover'),
                            ('camera_link', 'camera_link'), ('range_fl', 'range_fl'),
                            ('range_fr', 'range_fr'), ('range_rl', 'range_rl'),
                            ('range_rr', 'range_rr')]:
            expected = [float(v) for v in model.find(
                "link[@name='{}']".format(link)).findtext('pose', '0 0 0 0 0 0').split()]
            actual = self.tf.lookup_transform('base_link', frame, Time()).transform
            xyz = [actual.translation.x, actual.translation.y, actual.translation.z]
            assert all(abs(a-b) < 1e-5 for a, b in zip(xyz, expected[:3])), (frame, xyz, expected)
            roll, pitch, yaw = [v/2 for v in expected[3:]]
            sr, cr, sp, cp, sy, cy = (math.sin(roll), math.cos(roll), math.sin(pitch),
                                     math.cos(pitch), math.sin(yaw), math.cos(yaw))
            expected_q = [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                          cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]
            q = actual.rotation
            assert abs(sum(a*b for a, b in zip([q.x, q.y, q.z, q.w], expected_q))) > 0.999999
            sensor_poses[frame] = xyz
        sensor_frames = {}
        for topic in ('/scan', '/imu', '/range'):
            if topic not in self.latest:
                raise AssertionError('Missing sensor: ' + topic)
            frame = self.latest[topic].header.frame_id
            self.tf.lookup_transform('base_link', frame, Time())
            sensor_frames[topic] = frame
        assert self.range_frames == {'range_fl', 'range_fr', 'range_rl', 'range_rr'}, self.range_frames
        sensor_frames['/range_all_frames'] = sorted(self.range_frames)
        clock_nodes = ('slam_toolbox', 'controller_server', 'planner_server',
                       'recoveries_server', 'bt_navigator', 'waypoint_follower',
                       'local_costmap/local_costmap', 'global_costmap/global_costmap')
        clocks = {}
        for name in clock_nodes:
            request = GetParameters.Request()
            request.names = ['use_sim_time']
            response = self.request('/' + name + '/get_parameters', GetParameters, request)
            clocks[name] = response.values[0].bool_value
            assert response.values[0].type == 1 and clocks[name], (name, response)
        self.report['data'] = {'map_resolution': self.map.info.resolution,
                               'map_size': [self.map.info.width, self.map.info.height],
                               'frames_in_map': frames, 'sensor_frames': sensor_frames,
                               'sensor_poses_match_sdf': sensor_poses,
                               'clock_parameters': clocks, 'message_counts': dict(self.counts)}
        print('DATA map={} frames={} clocks=OK'.format(
            self.report['data']['map_size'], sensor_frames), flush=True)

    def pose(self):
        t = self.tf.lookup_transform('map', 'base_link', Time()).transform
        q = t.rotation
        return (t.translation.x, t.translation.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))

    def clear(self, x, y, radius=0.25):
        info = self.map.info
        origin = info.origin
        q = origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        dx, dy = x-origin.position.x, y-origin.position.y
        mx = int(math.floor((math.cos(yaw)*dx+math.sin(yaw)*dy)/info.resolution))
        my = int(math.floor((-math.sin(yaw)*dx+math.cos(yaw)*dy)/info.resolution))
        n = int(math.ceil(radius/info.resolution))
        for oy in range(-n, n+1):
            for ox in range(-n, n+1):
                if ox*ox+oy*oy > n*n:
                    continue
                cx, cy = mx+ox, my+oy
                if not (0 <= cx < info.width and 0 <= cy < info.height):
                    return False
                value = self.map.data[cy*info.width+cx]
                if value < 0 or value > 20:
                    return False
        return True

    def segment_clear(self, start, end):
        distance = math.hypot(end[0]-start[0], end[1]-start[1])
        steps = max(1, int(math.ceil(distance/0.025)))
        return all(self.clear(start[0]+(end[0]-start[0])*i/steps,
                              start[1]+(end[1]-start[1])*i/steps)
                   for i in range(steps+1))

    def choose_goal(self, home=None):
        pose = self.pose()
        # Favor forward travel; candidate and entire segment must be observed free.
        angles = (0, math.pi/8, -math.pi/8, math.pi/4, -math.pi/4,
                  math.pi/2, -math.pi/2, math.pi)
        for distance in (0.8, 0.7, 0.6):
            for angle in angles:
                yaw = pose[2] + angle
                goal = (pose[0]+distance*math.cos(yaw),
                        pose[1]+distance*math.sin(yaw), yaw)
                if home is not None and not 0.6 <= math.hypot(
                        goal[0]-home[0], goal[1]-home[1]) <= 0.95:
                    continue
                if self.segment_clear(pose, goal):
                    return goal
        raise RuntimeError('No observed free 0.6-0.8 m segment with 0.25 m clearance')

    def navigate(self, goal):
        self.sim_guard()
        start = self.pose()
        if not self.segment_clear(start, goal):
            raise RuntimeError('Goal segment is not free; movement forbidden')
        if math.hypot(goal[0]-start[0], goal[1]-start[1]) < 0.5:
            raise RuntimeError('Goal too near to verify actual navigation')
        if not self.navigator.wait_for_server(timeout_sec=5):
            raise RuntimeError('Navigation action unavailable')
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = 'map'
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = goal[:2]
        msg.pose.pose.orientation.z = math.sin(goal[2]/2)
        msg.pose.pose.orientation.w = math.cos(goal[2]/2)
        before = self.odom.pose.pose.position
        initial = (before.x, before.y)
        t0 = time.monotonic()
        handle = self.future(self.navigator.send_goal_async(msg), 10)
        record = {'goal': list(goal), 'start_map': list(start), 'accepted': handle.accepted}
        self.report['goals'].append(record)
        if not handle.accepted:
            raise AssertionError('Navigation goal rejected')
        print('GOAL start={} target={}'.format(start, goal), flush=True)
        try:
            response = self.future(handle.get_result_async(), self.args.goal_timeout)
        except TimeoutError:
            self.future(handle.cancel_goal_async(), 5)
            record['status'] = 'timeout'
            raise
        after = self.odom.pose.pose.position
        record.update(status=response.status, result=repr(response.result),
                      duration_seconds=time.monotonic()-t0,
                      odom_displacement=math.hypot(after.x-initial[0], after.y-initial[1]),
                      final_map=list(self.pose()))
        print('RESULT {}'.format(record), flush=True)
        assert response.status == 4, record
        assert record['odom_displacement'] > 0.25, record
        self.spin(1.)

    def run(self):
        self.start('sim', ['ros2', 'launch', 'rosbot_description',
                          'rosbot_sim.launch.py', 'gui:=false', 'camera:=false'])
        self.wait_sim()
        for index in range(self.args.restarts+1):
            label = 'offboard_' + str(index)
            self.start(label, ['ros2', 'launch', 'rosbot_description',
                               'offboard.launch.py', 'use_sim_time:=true'])
            self.wait_active(label)
            self.spin(3)
            self.check_data()
            if index == 0:
                home = self.pose()
                self.navigate(self.choose_goal())
                self.navigate(self.choose_goal(home))
                now = self.pose()
                goal = (home[0], home[1], math.atan2(home[1]-now[1], home[0]-now[0]))
                if math.hypot(home[0]-now[0], home[1]-now[1]) <= 0.5:
                    # A preceding goal may finish within Nav2's 25 cm tolerance
                    # toward home. Select another verified nearby free segment
                    # so the final action still demonstrates real movement.
                    goal = self.choose_goal(home)
                    self.report['return_goal_adjustment'] = {
                        'reason': 'home_within_0.5_m', 'home': list(home),
                        'replacement_goal': list(goal)}
                self.navigate(goal)
                counts = dict(self.counts)
                clock = self.clock
                wall = time.monotonic()
                self.spin(10)
                elapsed = time.monotonic()-wall
                self.report['rates'] = {name: (value-counts.get(name, 0))/elapsed
                                        for name, value in self.counts.items()}
                self.report['real_time_factor'] = (self.clock-clock)/elapsed
            self.stop(label)
            self.spin(2)
        self.report['passed'] = True

    def cleanup(self):
        for label in reversed(list(self.processes)):
            self.stop(label)
        # Gazebo may outlive ros2 launch; target only children carrying this UUID,
        # domain AND master URI. Never signal any pre-existing robot process.
        orphans = []
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit() or int(entry.name) == os.getpid():
                continue
            try:
                env = entry.joinpath('environ').read_bytes().split(b'\0')
                if all(value.encode() in env for value in (
                        'ROSBOT_PIPELINE_TEST='+self.marker,
                        'ROS_DOMAIN_ID='+str(self.args.domain),
                        'GAZEBO_MASTER_URI='+self.args.master)):
                    orphans.append(int(entry.name))
            except (OSError, ProcessLookupError):
                pass
        for pid in orphans:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        time.sleep(0.5)
        for pid in orphans:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self.report['cleanup_orphan_pids'] = orphans
        remaining = []
        for pid in orphans:
            try:
                if ('ROSBOT_PIPELINE_TEST='+self.marker).encode() in Path(
                        '/proc/{}/environ'.format(pid)).read_bytes().split(b'\0'):
                    remaining.append(pid)
            except OSError:
                pass
        self.report['cleanup_remaining_pids'] = remaining
        self.report['cleanup_success'] = not remaining
        self.report['finished_utc'] = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
        for handle in self.handles:
            handle.close()
        errors = []
        signal_exits = []
        for label, filename in self.report.get('logs', {}).items():
            for line in Path(filename).read_text(errors='replace').splitlines():
                match = re.search(r'process has died .*exit code (-?\d+)', line)
                if match:
                    item = {'launch': label, 'exit_code': int(match.group(1)), 'error': line}
                    if item['exit_code'] in (-2, -15):
                        signal_exits.append(item)
                    else:
                        errors.append(item)
        self.report['child_process_errors'] = errors
        self.report['child_signal_exits'] = signal_exits
        self.report['clean_shutdown'] = not errors and not orphans
        self.report['functional_checks_passed'] = self.report.get('passed', False)
        self.report['passed'] = self.report['functional_checks_passed'] and self.report['clean_shutdown']
        self.directory.joinpath('results.json').write_text(json.dumps(self.report, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=197)
    parser.add_argument('--master', default='http://127.0.0.1:11375')
    parser.add_argument('--rmw', default='rmw_fastrtps_cpp')
    parser.add_argument('--output', default='/tmp/rosbot_pipeline_fastdds')
    parser.add_argument('--restarts', type=int, default=2)
    parser.add_argument('--startup-timeout', type=float, default=100)
    parser.add_argument('--goal-timeout', type=float, default=100)
    args = parser.parse_args()
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
                      GAZEBO_MASTER_URI=args.master, RMW_IMPLEMENTATION=args.rmw)
    if args.rmw == 'rmw_cyclonedds_cpp' and not os.environ.get('CYCLONEDDS_URI'):
        # Foxy's Cyclone defaults to only ten local participant slots. This
        # pipeline starts eleven static TF processes plus Gazebo and Nav2.
        os.environ['CYCLONEDDS_URI'] = (
            '<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>50'
            '</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
    rclpy.init()
    check = PipelineCheck(args)
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(130))
    signal.signal(signal.SIGINT, lambda *_: sys.exit(130))
    code = 0
    try:
        check.run()
    except BaseException as error:
        check.report.update(passed=False, error=str(error), traceback=traceback.format_exc())
        traceback.print_exc()
        code = 1
    finally:
        check.cleanup()
        check.navigator.destroy()
        check.destroy_node()
        rclpy.shutdown()
    if not check.report['passed']:
        code = 1
    print('REPORT {}'.format(check.directory / 'results.json'), flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
