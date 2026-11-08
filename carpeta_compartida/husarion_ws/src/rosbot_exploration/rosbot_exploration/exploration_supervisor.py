"""Run exploration and cancel only its owned Nav2 goal on process exit.

The worker records the UUID *before* send_goal_async. This parent survives a
worker crash/SIGKILL and also handles Ctrl+C with its own ROS context intact.
No velocities and no cancel-all request are sent.
"""
import json
import os
import signal
import subprocess
import sys
import tempfile
import time


def cancel_owned_goal(lease_path, ros_args=None, timeout=8.0):
    try:
        with open(lease_path, encoding='utf-8') as stream:
            lease = json.load(stream)
    except FileNotFoundError:
        return True
    goal_id = list(bytes.fromhex(lease['uuid']))
    if len(goal_id) != 16 or not lease['action'].startswith('/'):
        raise ValueError('Invalid exploration goal lease')

    import rclpy
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from action_msgs.srv import CancelGoal
    from rclpy.node import Node
    from rclpy.qos import qos_profile_action_status_default

    handlers = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    rclpy.init(args=ros_args)
    for sig, handler in handlers.items():
        signal.signal(sig, handler)
    node = Node('exploration_supervisor', use_global_arguments=False)
    client = node.create_client(CancelGoal, lease['action'].rstrip('/') + '/_action/cancel_goal')
    terminal = False

    def status(message):
        nonlocal terminal
        for item in message.status_list:
            if list(item.goal_info.goal_id.uuid) == goal_id:
                terminal = item.status in (GoalStatus.STATUS_SUCCEEDED,
                                           GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)

    node.create_subscription(GoalStatusArray, lease['action'].rstrip('/') + '/_action/status',
                             status, qos_profile_action_status_default)
    deadline, next_request, pending = time.monotonic() + timeout, 0.0, None
    node.get_logger().info('Canceling exploration objective on ' + lease['action'])
    try:
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if terminal:
                return True
            if pending is not None and pending.done():
                response = pending.result()
                if response.return_code == CancelGoal.Response.ERROR_GOAL_TERMINATED:
                    return True
                pending = None
            if pending is None and client.service_is_ready() and time.monotonic() >= next_request:
                request = CancelGoal.Request()
                request.goal_info.goal_id.uuid = goal_id
                pending = client.call_async(request)
                next_request = time.monotonic() + 0.5
        node.get_logger().error('Could not confirm Nav2 cancellation before timeout')
        return False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    args = sys.argv[1:] if args is None else args
    stopping = False

    def stop(_signal, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGHUP, stop)
    with tempfile.TemporaryDirectory(prefix='rosbot_exploration_') as directory:
        lease_path = os.path.join(directory, 'goal.json')
        environment = dict(os.environ, ROSBOT_EXPLORATION_LEASE_FILE=lease_path)
        worker = subprocess.Popen([sys.executable, '-m',
                                   'rosbot_exploration.exploration_node', *args], env=environment)
        stop_deadline = None
        while worker.poll() is None:
            if stopping and stop_deadline is None:
                worker.terminate()
                stop_deadline = time.monotonic() + 4.0
            if stop_deadline is not None and time.monotonic() >= stop_deadline:
                worker.kill()
            time.sleep(0.05)
        # Replace ROS-installed handlers after cleanup init too, so repeated
        # Ctrl+C cannot shut down the context used to cancel the goal.
        success = cancel_owned_goal(lease_path, args)
        code = worker.returncode
        return_code = (0 if stopping else code) if success else 1
        if return_code:
            raise SystemExit(return_code if return_code > 0 else 1)


if __name__ == '__main__':
    main()
