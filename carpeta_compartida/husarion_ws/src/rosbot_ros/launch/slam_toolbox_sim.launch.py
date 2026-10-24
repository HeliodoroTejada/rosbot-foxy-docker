import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rosbot_description = get_package_share_directory('rosbot_description')
    # In Foxy, a node-specific YAML key can win over a later /** dictionary.
    # Rewrite the node-specific key so hardware and simulation select one clock.
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True)

    slam_toolbox_node = launch_ros.actions.Node(
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation clock; set false for a physical robot'
        ),
        DeclareLaunchArgument(
            name='params_file',
            default_value=os.path.join(rosbot_description, 'config',
                                       'slam_toolbox_sim.yaml'),
            description='Full path to the SLAM parameters file'
        ),
        slam_toolbox_node,
    ])

if __name__ == '__main__':
    generate_launch_description()
