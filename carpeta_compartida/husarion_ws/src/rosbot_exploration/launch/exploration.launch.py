"""Add exploration to an already running SLAM + Nav2 stack."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = get_package_share_directory('rosbot_exploration')
    params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'autostart': LaunchConfiguration('autostart')},
        convert_types=True)
    explorer = Node(
        package='rosbot_exploration', executable='explore', output='screen',
        parameters=[params],
        # Allow the supervisor to finish its independent cancellation handshake.
        sigterm_timeout='15', sigkill_timeout='5')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('autostart', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'exploration.yaml')),
        RegisterEventHandler(OnProcessExit(
            target_action=explorer,
            on_exit=[EmitEvent(event=Shutdown(reason='Exploration process exited'))])),
        explorer,
    ])
