"""Spawn the simulated robot and publish sensor frames from the same SDF."""
import math
import os
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory('rosbot_description')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    camera = LaunchConfiguration('camera').perform(context).lower() == 'true'
    model_file = os.path.join(share, 'models', 'rosbot.sdf')
    tree = ET.parse(model_file)
    model = tree.getroot().find('model')
    nodes = []

    def static(name, xyz, rpy, parent, child):
        roll, pitch, yaw = rpy
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=name, output='log',
            arguments=[str(value) for value in (*xyz, yaw, pitch, roll, parent, child)],
            parameters=[{'use_sim_time': use_sim_time}]))

    def link_frame(link_name, frame_name):
        link = model.find("link[@name='{}']".format(link_name))
        pose = [float(value) for value in link.findtext('pose', '0 0 0 0 0 0').split()]
        static('sim_base_to_' + frame_name, pose[:3], pose[3:], 'base_link', frame_name)

    # Link poses in this SDF are relative to the model; base_link has zero pose.
    for link, frame in [('camera_link', 'camera_link'), ('laser_link', 'laser'),
                        ('top_cover', 'top'), ('top_cover', 'imu')]:
        link_frame(link, frame)
    for frame in ('range_fl', 'range_fr', 'range_rl', 'range_rr'):
        link_frame(frame, frame)
    static('sim_base_to_dummy', (0, 0, 0), (0, 0, 0), 'base_link', 'dummy')
    for kind in ('depth', 'rgb'):
        static('sim_camera_to_' + kind, (0, 0, 0), (-math.pi / 2, 0, -math.pi / 2),
               'camera_link', 'camera_' + kind + '_frame')

    if not camera:
        for link in model.findall('link'):
            for sensor in link.findall('sensor'):
                if sensor.get('type') in ('depth', 'camera', 'depth_camera'):
                    link.remove(sensor)
        handle, model_file = tempfile.mkstemp(prefix='rosbot_no_camera_', suffix='.sdf')
        os.close(handle)
        tree.write(model_file, encoding='utf-8', xml_declaration=True)

        def cleanup(_context, *args, **kwargs):
            if os.path.exists(model_file):
                os.unlink(model_file)
            return []

        nodes.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))

    nodes.append(Node(
        package='gazebo_ros', executable='spawn_entity.py', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-spawn_service_timeout', '60', '-entity', 'rosbot',
                   '-x', '0', '-y', '0', '-z', '0.03', '-file', model_file]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('camera', default_value='false', choices=['true', 'false'],
                              description='Enable RGB-D rendering; not required for laser SLAM/Nav2'),
        OpaqueFunction(function=setup),
    ])
