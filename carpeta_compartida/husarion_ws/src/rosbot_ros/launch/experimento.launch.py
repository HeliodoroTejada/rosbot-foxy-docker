"""Coordinate simulation or real offboard navigation with the saved RViz view."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = get_package_share_directory('rosbot_description')
    simulation = LaunchConfiguration('modo').perform(context) == 'simulacion'
    clock = 'true' if simulation else 'false'
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    if not os.path.isfile(rviz_config):
        raise ValueError('No existe la configuración de RViz: ' + rviz_config)

    def include(filename, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', filename)),
            launch_arguments=arguments.items())

    navigation = include('offboard.launch.py', {
        'use_sim_time': clock,
        'nav_params': LaunchConfiguration('nav_params'),
        'slam_params': LaunchConfiguration('slam_params'),
    })
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': simulation}],
        # La cámara física usa nombres distintos; se conserva el archivo guardado.
        remappings=[] if simulation else [
            ('/camera/image_raw', '/camera/color/image_raw'),
            ('/camera/camera_info', '/camera/color/camera_info'),
        ])
    stop_on_rviz_exit = RegisterEventHandler(OnProcessExit(
        target_action=rviz,
        on_exit=[EmitEvent(event=Shutdown(reason='RViz se ha cerrado'))]))

    if not simulation:
        return [stop_on_rviz_exit, navigation, rviz]

    # Esperar la inserción real del modelo, sin un sleep de duración arbitraria.
    def after_spawn(event, _context):
        if not event.cmd or os.path.basename(event.cmd[0]) != 'spawn_entity.py':
            return []
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='No se pudo insertar el ROSbot'))]
        return [navigation, rviz]

    return [
        stop_on_rviz_exit,
        RegisterEventHandler(OnProcessExit(on_exit=after_spawn)),
        include('rosbot_sim.launch.py', {
            'use_sim_time': clock,
            'gui': LaunchConfiguration('gui'),
            'camera': LaunchConfiguration('camera'),
            'world': LaunchConfiguration('world'),
        }),
    ]


def generate_launch_description():
    share = get_package_share_directory('rosbot_description')
    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='simulacion',
                              choices=['simulacion', 'offboard']),
        DeclareLaunchArgument('rviz_config', description='Ruta a config/rviz_config.rviz'),
        DeclareLaunchArgument('nav_params', default_value=''),
        DeclareLaunchArgument('slam_params',
                              default_value=os.path.join(share, 'config', 'offboard_mapping.yaml')),
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('camera', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('world',
                              default_value=os.path.join(share, 'worlds', 'willow_garage.world')),
        OpaqueFunction(function=setup),
    ])
