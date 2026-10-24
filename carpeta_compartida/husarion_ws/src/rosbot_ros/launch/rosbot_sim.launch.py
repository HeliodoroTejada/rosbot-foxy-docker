"""Gazebo supplies sensors and odometry; SLAM/Nav2 can run on another machine."""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start_gazebo(context):
    robot = get_package_share_directory('rosbot_description')
    env = {}
    for variable, directory in (
        ('GAZEBO_MODEL_PATH', os.path.join(robot, 'models')),
        ('GAZEBO_RESOURCE_PATH', os.path.join(robot, 'worlds')),
        ('GAZEBO_PLUGIN_PATH', os.path.join(get_package_prefix('gazebo_ros'), 'lib')),
    ):
        env[variable] = os.pathsep.join(filter(None, (directory, os.environ.get(variable))))
    server = ['gzserver', LaunchConfiguration('world').perform(context),
              '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
              '-s', 'libgazebo_ros_force_system.so']
    client = ['gzclient']
    if LaunchConfiguration('verbose').perform(context) == 'true':
        server.append('--verbose')
        client.append('--verbose')
    if LaunchConfiguration('gdb').perform(context) == 'true':
        server = ['gdb', '-ex', 'run', '--args'] + server
    # Foxy's gazebo_ros launchers use shell=True. Signal the actual Gazebo
    # processes directly so Ctrl+C cannot leave a server/client behind a shell.
    return [
        ExecuteProcess(cmd=server, output='screen', additional_env=env,
                       on_exit=Shutdown(reason='Gazebo server stopped')),
        ExecuteProcess(cmd=client, output='screen', additional_env=env,
                       condition=IfCondition(LaunchConfiguration('gui')),
                       on_exit=Shutdown(reason='Gazebo window closed')),
    ]


def generate_launch_description():
    robot = get_package_share_directory('rosbot_description')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(robot, 'worlds', 'willow_garage.world')),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gui', default_value='false', choices=['true', 'false'],
                              description='Open Gazebo client; server-only is sufficient for Nav2/RViz'),
        DeclareLaunchArgument('camera', default_value='false', choices=['true', 'false'],
                              description='Render RGB-D sensor only when it is needed'),
        DeclareLaunchArgument('verbose', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('gdb', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=start_gazebo),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(robot, 'launch', 'rosbot_spawn.launch.py')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'camera': LaunchConfiguration('camera')}.items()),
    ])
