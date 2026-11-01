"""Project Astra depth and CameraInfo into an XYZ PointCloud2 for Nav2."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/depth/camera_info'),
        DeclareLaunchArgument('cloud_topic', default_value='/camera/points'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='astra_camera', executable='depth_to_cloud_node',
            name='astra_depth_to_cloud',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            # The legacy Astra driver publishes metric depth with zero
            # distortion and identity rectification in its CameraInfo.
            remappings=[
                ('image_rect', LaunchConfiguration('depth_topic')),
                ('camera_info', LaunchConfiguration('camera_info_topic')),
                ('points', LaunchConfiguration('cloud_topic')),
            ]),
    ])
