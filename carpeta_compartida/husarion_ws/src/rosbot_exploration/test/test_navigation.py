"""Exercise real Foxy actions, TF and process shutdown without moving a robot."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import pytest
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster

from rosbot_exploration.exploration_node import ExplorationNode


def wait_for(predicate, timeout=12.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.03)
    raise AssertionError('Condition did not become true before timeout')


class FakeStack(Node):
    def __init__(self):
        super().__init__('fake_navigation')
        self.running = True
        self.nav_started = threading.Event()
        self.nav_requested = threading.Event()
        self.nav_canceled = threading.Event()
        self.nav_count = 0
        self.plan_count = 0
        self.behavior_trees = []
        self.finish_navigation = threading.Event()
        self.reject_plans = False
        self.complete_navigation = False
        self.wrong_plan_frame = False
        self.accept_delay = 0.0
        self.active = True
        self.freeze_data = False
        self.freeze_map = False
        self.path_end_offset = 0.0
        self.obstacle = None
        self.closed_map = False
        self.position = (2.5, 2.5)
        self.group = ReentrantCallbackGroup()
        self.state_services = [self.create_service(GetState, '/' + name + '/get_state', self.lifecycle,
                                            callback_group=self.group)
                         for name in ('bt_navigator', 'planner_server', 'controller_server')]
        self.nav = ActionServer(self, NavigateToPose, '/navigate_to_pose',
                                self.navigate, goal_callback=self.accept,
                                cancel_callback=lambda _: CancelResponse.ACCEPT,
                                callback_group=self.group)
        self.planner = ActionServer(self, ComputePathToPose, '/compute_path_to_pose', self.plan,
                                    cancel_callback=lambda _: CancelResponse.ACCEPT,
                                    callback_group=self.group)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', retained)
        self.cost_pub = self.create_publisher(OccupancyGrid, '/global_costmap/costmap', 1)
        self.tf = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish, callback_group=self.group)

    def lifecycle(self, request, response):
        response.current_state.id = State.PRIMARY_STATE_ACTIVE if self.active else State.PRIMARY_STATE_INACTIVE
        return response

    def publish(self):
        if self.freeze_data:
            return
        message = OccupancyGrid()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.resolution = 0.1
        message.info.width = message.info.height = 50
        message.info.origin.orientation.w = 1.0
        data = np.full((50, 50), 100 if self.closed_map else -1, dtype=np.int8)
        data[5:45, 5:45] = 0
        if self.obstacle is not None:
            x, y = self.obstacle
            data[int(y / 0.1), int(x / 0.1)] = 100
        message.data = data.ravel().tolist()
        if not self.freeze_map:
            self.map_pub.publish(message)
        self.cost_pub.publish(message)
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x, transform.transform.translation.y = self.position
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)

    def accept(self, request):
        self.nav_requested.set()
        if self.accept_delay:
            time.sleep(self.accept_delay)
        return GoalResponse.ACCEPT

    def plan(self, handle):
        self.plan_count += 1
        result = ComputePathToPose.Result()
        if self.reject_plans:
            handle.abort()
            return result
        result.path.header.frame_id = 'map'
        if self.wrong_plan_frame:
            result.path.header.frame_id = 'odom'
        for x, y in (self.position, (handle.request.pose.pose.position.x,
                                    handle.request.pose.pose.position.y)):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation.w = 1.0
            result.path.poses.append(pose)
        if self.path_end_offset:
            last = result.path.poses[-1].pose.position
            dx, dy = self.position[0] - last.x, self.position[1] - last.y
            distance = (dx * dx + dy * dy) ** 0.5
            last.x += dx * self.path_end_offset / distance
            last.y += dy * self.path_end_offset / distance
        handle.succeed()
        return result

    def navigate(self, handle):
        self.nav_count += 1
        self.behavior_trees.append(handle.request.behavior_tree)
        self.nav_started.set()
        if self.complete_navigation:
            self.closed_map = True
            handle.succeed()
            return NavigateToPose.Result()
        while self.running and not handle.is_cancel_requested and not self.finish_navigation.is_set():
            time.sleep(0.02)
        if handle.is_cancel_requested:
            handle.canceled()
            self.nav_canceled.set()
        elif self.finish_navigation.is_set():
            self.finish_navigation.clear()
            self.position = (handle.request.pose.pose.position.x, handle.request.pose.pose.position.y)
            handle.succeed()
        else:
            handle.abort()
        return NavigateToPose.Result()

    def close(self):
        self.running = False
        self.nav.destroy()
        self.planner.destroy()
        self.destroy_node()


@pytest.fixture
def stack():
    # Fail instead of accidentally testing against the workspace's real domain.
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0')
    rclpy.init(args=[])
    fake = FakeStack()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(fake)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield fake, executor
    finally:
        fake.running = False
        executor.shutdown()
        thread.join(timeout=5)
        fake.close()
        rclpy.shutdown()


def add_explorer(executor, **options):
    values = dict(autostart=True, planner_frequency=5.0, completion_delay=0.1,
                  completion_confirmations=2, retry_delay=0.3)
    values.update(options)
    node = ExplorationNode(parameter_overrides=[Parameter(key, value=value)
                                                for key, value in values.items()],
                           use_global_arguments=False)
    executor.add_node(node)
    return node


def close_explorer(node, executor):
    node.enabled = False
    node._cancel()
    if node.operation:
        wait_for(lambda: node.operation is None)
    executor.remove_node(node)
    node.pool.shutdown(wait=True)
    node.navigator.destroy()
    if node.planner is not None:
        node.planner.destroy()
    node.destroy_node()


def test_goal_is_sent_only_after_nav2_activation_and_can_be_paused(stack):
    fake, executor = stack
    fake.active = False
    node = add_explorer(executor)
    try:
        time.sleep(1.2)
        assert not fake.nav_started.is_set()
        fake.active = True
        wait_for(fake.nav_started.is_set)
        from std_srvs.srv import Trigger
        node._control('pause', Trigger.Response())
        wait_for(fake.nav_canceled.is_set)
        wait_for(lambda: node.operation is None)
        count = fake.nav_count
        time.sleep(0.5)
        assert fake.nav_count == count
        assert not node.enabled
        node._control('resume', Trigger.Response())
        wait_for(lambda: fake.nav_count == count + 1)
    finally:
        close_explorer(node, executor)


def test_enclosed_room_completes_without_navigation(stack):
    fake, executor = stack
    fake.closed_map = True
    node = add_explorer(executor)
    try:
        wait_for(lambda: node.state.startswith('COMPLETED'))
        assert not fake.nav_started.is_set()
        assert not node.enabled
    finally:
        close_explorer(node, executor)


def test_rejected_paths_never_reach_navigator_and_attempts_are_bounded(stack):
    fake, executor = stack
    fake.reject_plans = True
    node = add_explorer(executor, preplan=True, max_attempts=2, retry_radius=10.0)
    try:
        wait_for(lambda: fake.plan_count == 2 and node.state.startswith('WAITING_RETRY'))
        assert not fake.nav_started.is_set()
        assert node.memory.attempts[0].failures == 2
        time.sleep(0.5)
        assert fake.plan_count == 2
    finally:
        close_explorer(node, executor)


def test_lost_data_cancels_current_objective(stack):
    fake, executor = stack
    node = add_explorer(executor, map_timeout=0.6, costmap_timeout=0.6)
    try:
        wait_for(fake.nav_started.is_set)
        fake.freeze_data = True
        wait_for(fake.nav_canceled.is_set)
        wait_for(lambda: node.state.startswith('WAITING'))
    finally:
        close_explorer(node, executor)


def test_successful_viewpoint_rechecks_map_and_completes(stack):
    fake, executor = stack
    fake.complete_navigation = True
    node = add_explorer(executor, revisit_delay=0.2)
    try:
        wait_for(lambda: node.state.startswith('COMPLETED'))
        assert fake.nav_count == 1
    finally:
        close_explorer(node, executor)


def test_plan_in_wrong_frame_is_rejected(stack):
    fake, executor = stack
    fake.wrong_plan_frame = True
    node = add_explorer(executor, preplan=True, max_attempts=1, retry_radius=10.0)
    try:
        wait_for(lambda: fake.plan_count == 1 and node.state.startswith('WAITING_RETRY'))
        assert not fake.nav_started.is_set()
    finally:
        close_explorer(node, executor)


def test_default_uses_navigate_task_without_extra_planning_action(stack):
    fake, executor = stack
    node = add_explorer(executor)
    try:
        wait_for(fake.nav_started.is_set)
        assert fake.plan_count == 0
        assert Path(fake.behavior_trees[0]).is_file()
        assert 'server_timeout="1000"' in Path(fake.behavior_trees[0]).read_text()
    finally:
        close_explorer(node, executor)


def test_optional_preplan_accepts_endpoint_within_navfn_tolerance(stack):
    fake, executor = stack
    fake.path_end_offset = 0.4
    node = add_explorer(executor, preplan=True)
    try:
        wait_for(fake.nav_started.is_set)
        assert fake.plan_count >= 1
    finally:
        close_explorer(node, executor)


def test_next_goal_is_prefetched_without_preempting_or_waiting_for_new_map(stack):
    fake, executor = stack
    node = add_explorer(executor, lookahead_distance=3.0)
    try:
        wait_for(fake.nav_started.is_set)
        wait_for(lambda: node.search_result is not None and node.search_result[-1])
        assert fake.nav_count == 1
        fake.freeze_map = True
        revision = node.revision
        started = time.monotonic()
        fake.finish_navigation.set()
        wait_for(lambda: fake.nav_count == 2, timeout=2.0)
        assert time.monotonic() - started < 1.5
        assert node.revision == revision
        assert not fake.nav_canceled.is_set()
    finally:
        close_explorer(node, executor)


def test_transient_goal_obstacle_does_not_cancel_but_persistent_obstacle_does(stack):
    fake, executor = stack
    node = add_explorer(executor, goal_invalid_timeout=0.6)
    try:
        wait_for(fake.nav_started.is_set)
        fake.obstacle = (node.target.x, node.target.y)
        wait_for(lambda: node.target_invalid_since is not None)
        fake.obstacle = None
        wait_for(lambda: node.target_invalid_since is None)
        assert not fake.nav_canceled.is_set()
        fake.obstacle = (node.target.x, node.target.y)
        wait_for(fake.nav_canceled.is_set)
    finally:
        close_explorer(node, executor)


@pytest.mark.parametrize('shutdown', ['ctrl_c', 'terminal_close', 'worker_kill', 'pending_acceptance'])
def test_supervisor_cancels_owned_goal_on_exit(stack, tmp_path, shutdown):
    fake, _ = stack
    if shutdown == 'pending_acceptance':
        fake.accept_delay = 2.0
    log = tmp_path / 'explore.log'
    with log.open('w') as output:
        process = subprocess.Popen([
            sys.executable, '-m', 'rosbot_exploration.exploration_supervisor',
            '--ros-args', '-p', 'autostart:=true', '-p', 'planner_frequency:=5.0',
        ], stdout=output, stderr=subprocess.STDOUT)
        try:
            wait_for(fake.nav_requested.is_set if shutdown == 'pending_acceptance'
                     else fake.nav_started.is_set, timeout=18)
            if shutdown == 'worker_kill':
                children = Path('/proc/{0}/task/{0}/children'.format(process.pid)).read_text().split()
                assert len(children) == 1
                os.kill(int(children[0]), signal.SIGKILL)
            else:
                process.send_signal(signal.SIGHUP if shutdown == 'terminal_close' else signal.SIGINT)
            process.wait(timeout=15)
            assert fake.nav_canceled.is_set(), log.read_text()
            assert process.returncode == (1 if shutdown == 'worker_kill' else 0), log.read_text()
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=15)
