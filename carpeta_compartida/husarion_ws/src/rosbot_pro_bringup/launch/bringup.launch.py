"""Alternative to rosbot_pro.launch.py: do not run both together."""
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULTS = {
    'start_base': 'true', 'start_lidar': 'true', 'start_camera': 'false',
    'command_watchdog': 'true', 'command_timeout': '0.5',
    'camera_width': '640', 'camera_height': '480', 'camera_fps': '30',
    'publish_wheel_frames': 'true',
    'serial_port': '/dev/ttyS4', 'serial_baud': '525000',
    'lidar_port': '/dev/ttyUSB0', 'lidar_baud': '256000',
    'lidar_mode': 'Sensitivity',
    'laser_x': '0.0', 'laser_y': '0.0', 'laser_z': '0.08',
    'laser_yaw': str(-math.pi),
    'camera_x': '0.0', 'camera_y': '0.0', 'camera_z': '0.18',
    'camera_yaw': '0.0', 'camera_pitch': '0.0', 'camera_roll': '0.0',
    'twist_source': 'pose',
}


def setup(context):
    cfg = {key: LaunchConfiguration(key).perform(context) for key in DEFAULTS}
    enabled = lambda key: cfg[key].lower() in ('true', '1', 'yes')
    nodes = []

    def static(name, xyz, ypr, parent, child):
        # Foxy positional order: x y z yaw pitch roll parent child.
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=name, output='screen',
            arguments=[str(v) for v in (*xyz, *ypr, parent, child)],
            parameters=[{'use_sim_time': False}]))

    if enabled('start_base'):
        timeout = float(cfg['command_timeout'])
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('command_timeout must be finite and greater than zero')
        if enabled('command_watchdog'):
            nodes.append(Node(
                package='rosbot_pro_bringup', executable='cmd_vel_watchdog.py',
                output='screen', parameters=[{'use_sim_time': False,
                                              'command_timeout': timeout}]))
        nodes += [
            Node(package='rosbot_description', executable='rosserial_node.py',
                 output='screen', parameters=[{'use_sim_time': False,
                 'port': cfg['serial_port'], 'baud': int(cfg['serial_baud'])}],
                 remappings=[('/cmd_vel', '/cmd_vel_safe')]
                 if enabled('command_watchdog') else []),
            Node(package='rosbot_pro_bringup', executable='pose_to_odom.py',
                 output='screen', parameters=[{'use_sim_time': False,
                 'twist_source': cfg['twist_source']}]),
        ]
    if enabled('start_lidar'):
        nodes.append(Node(
            package='rplidar_ros', executable='rplidar_composition',
            name='rplidar', output='screen', parameters=[{
                'use_sim_time': False, 'serial_port': cfg['lidar_port'],
                'serial_baudrate': int(cfg['lidar_baud']), 'frame_id': 'laser',
                'inverted': False, 'angle_compensate': True,
                'scan_mode': cfg['lidar_mode']}]))
    static('base_to_laser', tuple(float(cfg['laser_' + k]) for k in ('x', 'y', 'z')),
           (float(cfg['laser_yaw']), 0, 0), 'base_link', 'laser')
    static('base_to_camera', tuple(float(cfg['camera_' + k]) for k in ('x', 'y', 'z')),
           tuple(float(cfg['camera_' + k]) for k in ('yaw', 'pitch', 'roll')),
           'base_link', 'camera_link')
    for kind in ('depth', 'color'):
        static('camera_to_' + kind + '_optical', (0, 0, 0),
               (-math.pi/2, 0, -math.pi/2),
               'camera_link', 'openni_' + kind + '_optical_frame')
    static('base_to_top', (0, 0, 0), (0, 0, 0), 'base_link', 'top')
    if enabled('publish_wheel_frames'):
        for name, xyz in (
            ('front_left_wheel', (0.08, 0.1, 0)),
            ('front_right_wheel', (0.08, -0.1, 0)),
            ('rear_left_wheel', (-0.08, 0.1, 0)),
            ('rear_right_wheel', (-0.08, -0.1, 0)),
        ):
            static('base_to_' + name, xyz, (0, 0, 0), 'base_link', name)
    if enabled('start_camera'):
        for key in ('camera_width', 'camera_height', 'camera_fps'):
            if int(cfg[key]) <= 0:
                raise ValueError(key + ' must be a positive integer')
        # No name override: this executable creates astra_camera and astra_camera_.
        nodes.append(Node(
            package='astra_camera', executable='astra_camera_node', output='screen',
            arguments=['-I', '-w', cfg['camera_width'], '-h', cfg['camera_height'],
                       '-f', cfg['camera_fps'], '-dw', cfg['camera_width'],
                       '-dh', cfg['camera_height'], '-df', cfg['camera_fps']],
            remappings=[('image', '/camera/color/image_raw'),
                        ('depth', '/camera/depth/image_raw'),
                        ('depth_camera_info', '/camera/depth/camera_info')],
            parameters=[{'use_sim_time': False}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in DEFAULTS.items()],
        OpaqueFunction(function=setup),
    ])
