"""Frontier exploration client for the Foxy NavigateToPose action.

Run through the `explore` supervisor; it cancels the owned goal even if this
worker is killed while Nav2 is still accepting or executing that goal.
"""
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.expand_topic_name import expand_topic_name
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from unique_identifier_msgs.msg import UUID
from visualization_msgs.msg import Marker, MarkerArray

from .frontier_search import Grid, clearance_mask, path_is_safe, search
from .goal_selection import CompletionGate, GoalMemory
from .lease import clear_lease, write_lease
from .viewpoints import footprint_is_free


DEFAULTS = {
    'autostart': False,
    'map_topic': '/map', 'costmap_topic': '/global_costmap/costmap',
    'map_frame': 'map', 'robot_base_frame': 'base_link',
    'navigate_action': '/navigate_to_pose', 'planner_action': '/compute_path_to_pose',
    'preplan': False, 'behavior_tree': 'auto',
    'lifecycle_nodes': ['/bt_navigator', '/planner_server', '/controller_server'],
    'planner_id': 'GridBased',
    'planner_frequency': 0.5, 'robot_radius': 0.25,
    'use_rectangular_footprint': True,
    'footprint_length': 0.28, 'footprint_width': 0.32, 'footprint_padding': 0.02,
    'free_threshold': 25, 'costmap_max_cost': 80,
    'min_frontier_size': 0.35, 'goal_standoff': 0.45, 'max_standoff': 0.9,
    'distance_weight': 1.0, 'gain_weight': 0.5, 'start_tolerance': 0.3,
    'information_radius': 3.0, 'information_rays': 72, 'information_candidates': 12,
    'candidates_per_frontier': 3, 'turn_weight': 0.35, 'revisit_weight': 1.0,
    'min_goal_distance': 0.5,
    'map_timeout': 10.0, 'costmap_timeout': 5.0, 'transform_timeout': 2.0,
    'planning_timeout': 8.0, 'navigation_timeout': 180.0,
    'path_endpoint_tolerance': 0.5,
    'lookahead_distance': 1.0, 'lookahead_max_age': 5.0,
    'handoff_delay': 0.2, 'failure_backoff': 0.5,
    'goal_invalid_timeout': 1.5,
    'progress_timeout': 30.0, 'progress_distance': 0.1,
    'retry_delay': 30.0, 'retry_radius': 0.5, 'max_attempts': 3,
    'revisit_delay': 10.0, 'completion_confirmations': 3, 'completion_delay': 5.0,
    'blocked_retry_delay': 180.0, 'visit_cooldown': 90.0,
    'scene_change_ratio': 0.1, 'retry_frontier_radius': 0.75,
    'max_map_cells': 2000000, 'cancel_timeout': 8.0,
}


def grid_from_message(message, max_cells):
    info = message.info
    if not info.width or not info.height or info.width * info.height > max_cells:
        raise ValueError('Empty map or max_map_cells exceeded')
    if len(message.data) != info.width * info.height:
        raise ValueError('Occupancy grid dimensions do not match data')
    q = info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    data = np.asarray(message.data, dtype=np.int16).reshape((info.height, info.width)).copy()
    return Grid(data, info.resolution, info.origin.position.x, info.origin.position.y, yaw)


class ExplorationNode(Node):
    def __init__(self, **kwargs):
        super().__init__('exploration', **kwargs)
        for key, value in DEFAULTS.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in DEFAULTS}
        self._validate_parameters()
        if self.cfg['behavior_tree'] == 'auto':
            self.cfg['behavior_tree'] = str(Path(get_package_share_directory('rosbot_exploration')) /
                                            'behavior_trees' / 'explore_w_replanning.xml')
        self.add_on_set_parameters_callback(self._parameters_changed)
        self.enabled = self.cfg['autostart']
        self.state = ''
        self.grid = self.costmap = None
        self.revision = 0
        self.map_received = self.costmap_received = 0.0
        self.next_search = 0.0
        self.search_task = None
        self.search_result = None
        self.next_goal_time = 0.0
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.memory = GoalMemory(self.cfg['retry_radius'], self.cfg['retry_delay'],
                                 self.cfg['max_attempts'], self.cfg['revisit_delay'],
                                 self.cfg['blocked_retry_delay'], self.cfg['visit_cooldown'],
                                 self.cfg['scene_change_ratio'], self.cfg['retry_frontier_radius'])
        self.completion = CompletionGate(self.cfg['completion_confirmations'],
                                         self.cfg['completion_delay'])
        self.target = None
        self.robot_yaw = 0.0
        self.operation = None  # planning or navigating, including pending acceptance
        self.goal_handle = None
        self.cancel_requested = False
        self.cancel_started = None
        self.operation_token = 0
        self.started_at = self.last_progress = 0.0
        self.best_distance = float('inf')
        self.best_remaining = float('inf')
        self.target_invalid_since = None
        self.lease_path = os.environ.get('ROSBOT_EXPLORATION_LEASE_FILE', '')
        self.active_uuid = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = ActionClient(self, NavigateToPose, self.cfg['navigate_action'])
        self.planner = (ActionClient(self, ComputePathToPose, self.cfg['planner_action'])
                        if self.cfg['preplan'] else None)
        self.lifecycle = {}
        self.lifecycle_next = 0.0
        for name in self.cfg['lifecycle_nodes']:
            self.lifecycle[name] = [self.create_client(GetState, name.rstrip('/') + '/get_state'),
                                    None, 0.0, False]
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self.cfg['map_topic'], self._map, retained)
        # Nav2 publishes full costmaps periodically; volatile also accepts durable publishers.
        self.create_subscription(OccupancyGrid, self.cfg['costmap_topic'], self._costmap, 1)
        self.state_pub = self.create_publisher(String, '~/state', retained)
        self.markers_pub = self.create_publisher(MarkerArray, '~/frontiers', retained)
        self.goal_pub = self.create_publisher(PoseStamped, '~/goal', 1)
        self.candidate_pub = self.create_publisher(String, '~/candidates', retained)
        for command in ('start', 'pause', 'resume', 'stop'):
            self.create_service(Trigger, '~/' + command,
                                lambda request, response, cmd=command: self._control(cmd, response))
        self.timer = self.create_timer(0.1, self._tick,
                                      clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._state('WAITING' if self.enabled else 'IDLE', 'Use ~/start to explore')

    def _validate_parameters(self):
        integers = ('max_attempts', 'completion_confirmations', 'max_map_cells',
                    'information_rays', 'information_candidates', 'candidates_per_frontier')
        for key in integers:
            if not isinstance(self.cfg[key], int) or isinstance(self.cfg[key], bool) or self.cfg[key] < 1:
                raise ValueError(key + ' must be a positive integer')
        for key, default in DEFAULTS.items():
            if isinstance(default, float):
                value = self.cfg[key]
                if not math.isfinite(value) or value < 0:
                    raise ValueError(key + ' must be finite and non-negative')
                if value == 0 and key not in ('gain_weight', 'completion_delay', 'handoff_delay',
                                              'footprint_padding', 'turn_weight', 'revisit_weight'):
                    raise ValueError(key + ' must be positive')
            elif isinstance(default, str) and not self.cfg[key] and key != 'behavior_tree':
                raise ValueError(key + ' must not be empty')
        if not 0 <= self.cfg['free_threshold'] < 100 or not 0 <= self.cfg['costmap_max_cost'] < 100:
            raise ValueError('Occupancy thresholds must be in [0, 100)')
        if self.cfg['goal_standoff'] < self.cfg['robot_radius']:
            raise ValueError('goal_standoff must be >= robot_radius')
        if self.cfg['max_standoff'] < self.cfg['goal_standoff']:
            raise ValueError('max_standoff must be >= goal_standoff')
        if not 8 <= self.cfg['information_rays'] <= 720 or self.cfg['information_candidates'] > 64:
            raise ValueError('Visibility budget: 8..720 rays, at most 64 candidates')
        if not 0 < self.cfg['scene_change_ratio'] <= 1:
            raise ValueError('scene_change_ratio must be in (0, 1]')

    def _parameters_changed(self, parameters):
        if any(p.name in DEFAULTS for p in parameters):
            return SetParametersResult(successful=False, reason='Restart exploration to change its configuration')
        return SetParametersResult(successful=True)

    def _state(self, state, reason=''):
        message = state + (': ' + reason if reason else '')
        if message != self.state:
            self.state = message
            self.state_pub.publish(String(data=message))
            self.get_logger().info(message)

    def _map(self, message):
        try:
            if message.header.frame_id.lstrip('/') != self.cfg['map_frame'].lstrip('/'):
                raise ValueError('Map frame differs from map_frame')
            self.grid = grid_from_message(message, self.cfg['max_map_cells'])
            self.map_received = time.monotonic()
            self.revision += 1
        except ValueError as error:
            self.grid = None
            self.get_logger().warning(str(error))

    def _costmap(self, message):
        try:
            if message.header.frame_id.lstrip('/') != self.cfg['map_frame'].lstrip('/'):
                raise ValueError('Global costmap frame differs from map_frame')
            self.costmap = grid_from_message(message, self.cfg['max_map_cells'])
            self.costmap_received = time.monotonic()
        except ValueError as error:
            self.costmap = None
            self.get_logger().warning(str(error))

    def _control(self, command, response):
        if command in ('pause', 'stop'):
            self.enabled = False
            self.completion.reset()
            self._cancel()
            if self.search_task is not None:
                self.search_task[0].cancel()
            self.search_task = None
            self.search_result = None
            self._state('PAUSED' if command == 'pause' else 'STOPPED')
        elif self.operation is not None or self.cancel_requested:
            response.success = False
            response.message = 'Wait until the current objective has stopped'
            return response
        else:
            if command == 'start':
                self.memory.clear()
            self.enabled = True
            self.completion.reset()
            self.next_search = 0.0
            self.next_goal_time = 0.0
            self.search_result = None
            self._state('WAITING')
        response.success = True
        response.message = 'Cancellation requested' if self.operation else self.state
        return response

    def _poll_lifecycle(self, now):
        if now < self.lifecycle_next:
            return
        self.lifecycle_next = now + 1.0
        for name, entry in self.lifecycle.items():
            client, pending, _, _ = entry
            if pending is not None and not pending.done():
                continue
            if not client.service_is_ready():
                entry[3] = False
                continue
            future = client.call_async(GetState.Request())
            entry[1] = future

            def received(done, record=entry):
                try:
                    record[3] = done.result().current_state.id == State.PRIMARY_STATE_ACTIVE
                    record[2] = time.monotonic()
                except Exception:
                    record[3] = False
            future.add_done_callback(received)

    def _ready(self, now):
        if self.grid is None or now - self.map_received > self.cfg['map_timeout']:
            raise ValueError('Waiting for a current SLAM map')
        if self.costmap is None or now - self.costmap_received > self.cfg['costmap_timeout']:
            raise ValueError('Waiting for the global costmap')
        if (not self.navigator.server_is_ready() or
                (self.planner is not None and not self.planner.server_is_ready())):
            raise ValueError('Waiting for Nav2 action servers')
        if any(not entry[3] or now - entry[2] > 4.0 for entry in self.lifecycle.values()):
            raise ValueError('Waiting for active Nav2 lifecycle nodes')
        try:
            transform = self.tf_buffer.lookup_transform(self.cfg['map_frame'],
                                                        self.cfg['robot_base_frame'], Time())
        except TransformException as error:
            raise ValueError('Waiting for robot TF: ' + str(error)) from error
        stamp = Time.from_msg(transform.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds * 1e-9
        if stamp.nanoseconds == 0 or age > self.cfg['transform_timeout'] or age < -0.5:
            raise ValueError('Robot TF is stale or uses a different clock')
        q = transform.transform.rotation
        self.robot_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return transform.transform.translation.x, transform.transform.translation.y

    def _tick(self):
        now = time.monotonic()
        self._poll_lifecycle(now)
        if self.cancel_requested:
            if self.cancel_started and now - self.cancel_started > self.cfg['cancel_timeout']:
                # Exit to the independent supervisor; it retries the exact owned UUID.
                raise RuntimeError('Nav2 did not finish cancellation; handing over to supervisor')
            return
        if not self.enabled:
            return
        try:
            robot = self._ready(now)
        except ValueError as error:
            self.completion.reset()
            self._cancel()
            self._state('WAITING', str(error))
            return
        self._harvest_search(now)
        self.memory.observe_pose(robot[0], robot[1], now)
        if self.operation:
            timeout = self.cfg['planning_timeout'] if self.operation == 'planning' else self.cfg['navigation_timeout']
            if now - self.started_at > timeout:
                self._fail('Objective timed out')
            elif self.operation == 'navigating':
                distance = math.hypot(robot[0] - self.target.x, robot[1] - self.target.y)
                if distance < self.best_distance - self.cfg['progress_distance']:
                    self.best_distance, self.last_progress = distance, now
                if now - self.last_progress > self.cfg['progress_timeout']:
                    self._fail('No progress towards the frontier')
                elif not self._target_valid(self.target):
                    if self.target_invalid_since is None:
                        self.target_invalid_since = now
                    elif now - self.target_invalid_since >= self.cfg['goal_invalid_timeout']:
                        self._fail('Viewpoint remained invalid in map/costmap')
                else:
                    self.target_invalid_since = None
                    if distance <= self.cfg['lookahead_distance']:
                        # Prepare the next viewpoints without sending another
                        # action or preempting the current navigation task.
                        if self.search_result is None or self.search_result[1] < self.revision:
                            self._schedule_search((self.target.x, self.target.y), now, prefetch=True)
            return
        if now < self.next_goal_time:
            return
        if self.search_result is not None:
            result, revision, source_grid, origin, finished, prefetched = self.search_result
            self.search_result = None
            if (now - finished > self.cfg['lookahead_max_age'] or
                    math.hypot(robot[0] - origin[0], robot[1] - origin[1]) >
                    self.cfg['lookahead_distance'] + self.cfg['start_tolerance']):
                self.next_search = 0.0
                return
            self._markers(result.frontiers, source_grid)
            candidates = [f for f in result.frontiers if self.memory.allowed(f, now) and
                          math.hypot(f.x - robot[0], f.y - robot[1]) >= self.cfg['min_goal_distance'] and
                          self._target_valid(f)]
            self.candidate_pub.publish(String(data=json.dumps([
                dict(x=float(f.x), y=float(f.y), yaw=f.yaw, information_m2=f.information_gain,
                     distance_m=f.distance, turn_rad=f.turn, revisit=f.revisit, score=f.score)
                for f in candidates])))
            if candidates:
                self.completion.reset()
                self.target = candidates[0]
                if self.cfg['preplan']:
                    self._plan()
                else:
                    self._navigate()
            elif prefetched:
                # Completion is established from the actual robot pose, never
                # from an anticipated arrival point or an outdated candidate.
                self.next_search = 0.0
            elif (result.frontiers or result.excluded) and self.memory.temporarily_blocked(now):
                self.completion.reset()
                self._state('WAITING_RETRY', 'Viewpoints are temporarily excluded')
            elif result.frontiers or result.excluded:
                self.completion.reset()
                self._state('BLOCKED', 'Candidates rejected; start resets failed attempts')
            elif self.completion.update(True, revision, now):
                self.enabled = False
                self._state('COMPLETED', 'No reachable frontiers remain')
            else:
                self._state('CHECKING_COMPLETION', 'Waiting for confirmation from new maps')
            return
        self._schedule_search(robot, now)

    def _schedule_search(self, robot, now, prefetch=False):
        if self.search_task is None and now >= self.next_search:
            self.next_search = now + 1.0 / self.cfg['planner_frequency']
            options = {key: self.cfg[key] for key in (
                'robot_radius', 'free_threshold', 'min_frontier_size', 'goal_standoff',
                'max_standoff', 'distance_weight', 'gain_weight', 'start_tolerance',
                'min_goal_distance', 'information_radius', 'information_rays', 'information_candidates',
                'candidates_per_frontier', 'turn_weight', 'revisit_weight',
                'use_rectangular_footprint', 'footprint_length', 'footprint_width', 'footprint_padding')}
            self.memory.update_scene(self.grid, self.costmap, now)
            options['excluded_positions'] = self.memory.exclusions(now)
            options['excluded_frontiers'] = self.memory.frontier_exclusions(now)
            options['recent_positions'] = self.memory.recent_positions(now)
            options['robot_yaw'] = self.target.yaw if prefetch else self.robot_yaw
            if prefetch:
                options['excluded_positions'].append((self.target.x, self.target.y, self.cfg['retry_radius']))
            options['costmap'] = self.costmap
            options['costmap_max_cost'] = self.cfg['costmap_max_cost']
            self.search_task = (self.pool.submit(search, self.grid, robot, **options),
                                self.revision, self.grid, robot, prefetch)

    def _harvest_search(self, now):
        if self.search_task is None or not self.search_task[0].done():
            return
        future, revision, source_grid, origin, prefetched = self.search_task
        self.search_task = None
        try:
            result = future.result()
        except ValueError as error:
            self.completion.reset()
            if self.operation is None:
                self._state('WAITING', str(error))
            return
        self.search_result = (result, revision, source_grid, origin, now, prefetched)

    def _target_valid(self, target):
        row, col = self.grid.cell(target.x, target.y)
        if not self.grid.contains(row, col):
            return False
        if self.cfg['use_rectangular_footprint']:
            if not footprint_is_free(self.grid, target.x, target.y, target.yaw,
                                     self.cfg['footprint_length'], self.cfg['footprint_width'],
                                     self.cfg['footprint_padding'], self.cfg['free_threshold']):
                return False
        else:
            radius = int(math.ceil(self.cfg['robot_radius'] / self.grid.resolution + 1))
            h, w = self.grid.data.shape
            r0, r1, c0, c1 = row - radius, row + radius + 1, col - radius, col + radius + 1
            if r0 < 0 or c0 < 0 or r1 > h or c1 > w:
                return False
            free = ((self.grid.data[r0:r1, c0:c1] >= 0) &
                    (self.grid.data[r0:r1, c0:c1] <= self.cfg['free_threshold']))
            if not clearance_mask(free, self.grid.resolution, self.cfg['robot_radius'])[radius, radius]:
                return False
        cr, cc = self.costmap.cell(target.x, target.y)
        return (self.costmap.contains(cr, cc) and
                0 <= self.costmap.data[cr, cc] <= self.cfg['costmap_max_cost'])

    def _pose(self):
        pose = PoseStamped()
        pose.header.frame_id = self.cfg['map_frame']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = self.target.x, self.target.y
        pose.pose.orientation.z = math.sin(self.target.yaw / 2)
        pose.pose.orientation.w = math.cos(self.target.yaw / 2)
        return pose

    def _plan(self):
        goal = ComputePathToPose.Goal()
        goal.pose, goal.planner_id = self._pose(), self.cfg['planner_id']
        self._send(self.planner, goal, 'planning')
        self._state('PLANNING')

    def _navigate(self):
        goal = NavigateToPose.Goal()
        goal.pose = self._pose()
        goal.behavior_tree = self.cfg['behavior_tree']
        self.goal_pub.publish(goal.pose)
        self._send(self.navigator, goal, 'navigating')
        self._state('EXPLORING')

    def _send(self, client, goal, operation):
        self.operation_token += 1
        token = self.operation_token
        self.operation, self.goal_handle = operation, None
        self.started_at = self.last_progress = time.monotonic()
        self.best_distance = float('inf')
        self.best_remaining = float('inf')
        self.target_invalid_since = None
        self.active_uuid = list(uuid.uuid4().bytes)
        action = self.cfg['navigate_action'] if operation == 'navigating' else self.cfg['planner_action']
        action = expand_topic_name(action, self.get_name(), self.get_namespace())
        write_lease(self.lease_path, action, self.active_uuid)
        future = client.send_goal_async(goal, goal_uuid=UUID(uuid=self.active_uuid),
                                        feedback_callback=lambda msg: self._feedback(msg, token))
        future.add_done_callback(lambda done: self._accepted(done, token))

    def _feedback(self, message, token):
        if token == self.operation_token and self.operation == 'navigating':
            remaining = message.feedback.distance_remaining
            if math.isfinite(remaining) and 0 <= remaining < self.best_remaining - self.cfg['progress_distance']:
                self.best_remaining = remaining
                self.last_progress = time.monotonic()

    def _accepted(self, future, token):
        if token != self.operation_token:
            return
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            if self.cancel_requested or not self.enabled:
                self._clear_operation()
            else:
                self._finished(False, 'Objective rejected')
            return
        result = self.goal_handle.get_result_async()
        result.add_done_callback(lambda done: self._result(done, token))
        if self.cancel_requested or not self.enabled:
            self._cancel()

    def _result(self, future, token):
        if token != self.operation_token:
            return
        response = future.result()
        success = response.status == GoalStatus.STATUS_SUCCEEDED
        if self.cancel_requested or not self.enabled:
            self._clear_operation()
            return
        if self.operation == 'planning' and success:
            try:
                self._ready(time.monotonic())
            except ValueError as error:
                self._clear_operation()
                self._state('WAITING', str(error))
                return
            path = response.result.path
            points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
            free = (self.grid.data >= 0) & (self.grid.data <= self.cfg['free_threshold'])
            safe = clearance_mask(free, self.grid.resolution, self.cfg['robot_radius'])
            wrong_frame = (path.header.frame_id != self.cfg['map_frame'] or
                           any(p.header.frame_id not in ('', self.cfg['map_frame']) for p in path.poses))
            wrong_end = (not points or math.hypot(points[-1][0] - self.target.x,
                                                 points[-1][1] - self.target.y) >
                         self.cfg['path_endpoint_tolerance'])
            if wrong_frame:
                self._finished(False, 'Plan rejected: expected frame ' + self.cfg['map_frame'])
                return
            if wrong_end:
                self._finished(False, 'Plan rejected: endpoint exceeds path_endpoint_tolerance')
                return
            if not self._target_valid(self.target):
                self._finished(False, 'Plan rejected: viewpoint changed in the latest map/costmap')
                return
            if not path_is_safe(points, self.grid, safe):
                self._finished(False, 'Plan rejected: strict known-free clearance check failed')
                return
            self._clear_operation()
            self._navigate()
            return
        self._finished(success, 'Reached viewpoint' if success else 'Nav2 aborted the objective')

    def _finished(self, success, reason):
        if self.target:
            self.memory.record(self.target, time.monotonic(), success, self.grid, self.costmap)
        self._clear_operation()
        self.next_search = 0.0
        self.next_goal_time = time.monotonic() + (self.cfg['handoff_delay'] if success
                                                 else self.cfg['failure_backoff'])
        self._state('SELECTING', reason)

    def _fail(self, reason):
        self.memory.record(self.target, time.monotonic(), grid=self.grid, costmap=self.costmap)
        self._cancel()
        self._state('CANCELING', reason)

    def _cancel(self):
        if self.operation is None:
            return
        if not self.cancel_requested:
            self.cancel_requested = True
            self.cancel_started = time.monotonic()
            if self.goal_handle is not None and self.goal_handle.accepted:
                self.goal_handle.cancel_goal_async()
        elif self.goal_handle is not None and self.goal_handle.accepted:
            # Also covers pause arriving before the goal response.
            self.goal_handle.cancel_goal_async()

    def _clear_operation(self):
        if self.active_uuid is not None:
            clear_lease(self.lease_path, self.active_uuid)
        self.active_uuid = None
        self.operation = self.goal_handle = None
        self.cancel_requested = False
        self.cancel_started = None

    def _markers(self, frontiers, source_grid):
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.cfg['map_frame']
        clear.header.stamp = self.get_clock().now().to_msg()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        seen = set()
        for frontier in frontiers:
            if id(frontier.cells) in seen:
                continue
            seen.add(id(frontier.cells))
            marker = Marker()
            marker.header.frame_id = self.cfg['map_frame']
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type = 'frontiers', len(seen), Marker.POINTS
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = source_grid.resolution
            marker.color.r, marker.color.g, marker.color.a = 0.2, 1.0, 1.0
            # Bound RViz traffic for large maps.
            for row, col in frontier.cells[::max(1, len(frontier.cells) // 500)]:
                x, y = source_grid.world(row, col)
                marker.points.append(Point(x=x, y=y, z=0.05))
            markers.markers.append(marker)
        self.markers_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    stopping = False

    def stop(_signal, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGHUP, stop)
    node = None
    try:
        node = ExplorationNode()
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        # The parent retains an independent context and the persisted UUID. Let
        # it cancel after this process exits, without ROS signal-handler races.
        if node is not None:
            node.enabled = False
            node.pool.shutdown(wait=False)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
