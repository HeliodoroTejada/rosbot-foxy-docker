"""Project Astra depth and CameraInfo into an XYZ PointCloud2 for Nav2."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/depth/camera_info'),
        DeclareLaunchArgument('cloud_topic', default_value='/camera/points'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('pixel_stride', default_value='2',
                             description='Sample every Nth depth pixel in both axes (1=full)'),
        DeclareLaunchArgument('max_rate', default_value='5.0',
                             description='Maximum cloud generation rate in Hz (0=unlimited)'),
        Node(
            package='astra_camera', executable='depth_to_cloud_node',
            name='astra_depth_to_cloud',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'pixel_stride': ParameterValue(LaunchConfiguration('pixel_stride'), value_type=int),
                'max_rate': ParameterValue(LaunchConfiguration('max_rate'), value_type=float),
            }],
            # The legacy Astra driver publishes metric depth with zero
            # distortion and identity rectification in its CameraInfo.
            remappings=[
                ('image_rect', LaunchConfiguration('depth_topic')),
                ('camera_info', LaunchConfiguration('camera_info_topic')),
                ('points', LaunchConfiguration('cloud_topic')),
            ]),
    ])
