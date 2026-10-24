"""Run mapping and Nav2 on a PC, against existing simulator or robot drivers."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def launch_stack(context):
    share = get_package_share_directory('rosbot_description')
    time_value = LaunchConfiguration('use_sim_time').perform(context).lower()
    if time_value not in ('true', 'false'):
        raise ValueError('use_sim_time must be true or false')
    use_sim_time = time_value == 'true'
    nav_params = LaunchConfiguration('nav_params').perform(context)
    if not nav_params:
        profile = 'nav2_params_sim.yaml' if use_sim_time else 'nav2_params_pro.yaml'
        nav_params = os.path.join(share, 'config', profile)
    slam_params = LaunchConfiguration('slam_params').perform(context)
    for path in (nav_params, slam_params):
        if not os.path.isfile(path):
            raise ValueError('Parameter file does not exist: ' + path)

    # Foxy may prioritize node-specific YAML over later wildcard dictionaries.
    # Rewrite the file itself so SLAM and Nav2 always share the same clock.
    configured_slam = RewrittenYaml(
        source_file=slam_params,
        param_rewrites={'use_sim_time': time_value, 'mode': 'mapping'},
        convert_types=True)

    return [
        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', output='screen',
            parameters=[configured_slam,
                        {'use_sim_time': use_sim_time, 'mode': 'mapping'}]),
        # This backend accepts explicit clock and parameters for both environments.
        # It starts only Nav2; no Gazebo, hardware driver or robot TF publisher.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, 'launch', 'rosbot_navigation_sim.launch.py')),
            launch_arguments={
                'use_sim_time': time_value,
                'params': nav_params,
                'autostart': LaunchConfiguration('autostart'),
            }.items()),
    ]


def generate_launch_description():
    share = get_package_share_directory('rosbot_description')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='true for existing Gazebo /clock; false for real robot'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Activate Nav2 automatically after configuration'),
        DeclareLaunchArgument(
            'nav_params',
            default_value='',
            description='Override Nav2 YAML; empty selects sim/pro profile from use_sim_time'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(share, 'config', 'offboard_mapping.yaml'),
            description='SLAM mapping parameters; clock and mode are set by this launch'),
        OpaqueFunction(function=launch_stack),
    ])
