import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _interfaz_por_defecto():
    """Nombre de la interfaz con la ruta por defecto (la Wi-Fi), o '' si no se sabe."""
    try:
        with open('/proc/net/route') as f:
            rutas = [l.split() for l in f.readlines()[1:]]
        por_defecto = [r for r in rutas if len(r) > 6 and r[1] == '00000000']
        return min(por_defecto, key=lambda r: int(r[6]))[0] if por_defecto else ''
    except OSError:
        return ''


def _cyclone_real(robot_ip):
    """Perfil de CycloneDDS del robot (launch_rosbot.sh) más lo que necesita el PC.

    - Interfaz fija: en WSL hay varias (Wi-Fi, Tailscale...) y Cyclone puede elegir mal.
    - ParticipantIndex auto: puertos 7410+ en vez de aleatorios, para que pasen el
      firewall (regla UDP 7400-7600).
    - Peer: saluda al robot directamente por su IP además de por multicast.
    """
    interfaz = _interfaz_por_defecto()
    general = '<NetworkInterfaceAddress>%s</NetworkInterfaceAddress>' % interfaz if interfaz else ''
    peers = '<Peers><Peer address="%s"/></Peers>' % robot_ip if robot_ip else ''
    return ('<CycloneDDS><Domain><General>%s<MaxMessageSize>1400B</MaxMessageSize>'
            '<FragmentSize>1280B</FragmentSize></General><Discovery>'
            '<ParticipantIndex>auto</ParticipantIndex>%s'
            '<MaxAutoParticipantIndex>50</MaxAutoParticipantIndex></Discovery>'
            '</Domain></CycloneDDS>' % (general, peers))


def _nodo(context):
    modo = LaunchConfiguration('modo').perform(context)
    if modo not in ('sim', 'real'):
        raise ValueError("modo debe ser 'sim' o 'real'")
    entorno = {}
    if modo == 'real':
        # Mismo DDS que rosbot.service en el robot (launch_rosbot.sh).
        robot_ip = LaunchConfiguration('robot_ip').perform(context)
        entorno = {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp', 'ROS_LOCALHOST_ONLY': '0',
                   'CYCLONEDDS_URI': os.environ.get('CYCLONEDDS_URI', _cyclone_real(robot_ip))}
    return [Node(
        package='rosbot_telegram_ai', executable='telegram_ai_node', name='telegram_ai',
        output='screen', additional_env=entorno,
        parameters=[LaunchConfiguration('params_file').perform(context), {
            'use_sim_time': modo == 'sim',
            # En el robot real el lidar va girado 180º (laser_yaw en bringup.launch.py).
            'lidar_yaw': -math.pi if modo == 'real' else 0.0,
            'env_file': LaunchConfiguration('env_file').perform(context),
        }],
    )]


def generate_launch_description():
    share = get_package_share_directory('rosbot_telegram_ai')
    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='sim', description="'sim' o 'real'"),
        DeclareLaunchArgument('robot_ip', default_value='10.105.34.33',
                              description="IP del robot en modo real ('' = sólo multicast)"),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'telegram_ai.yaml')),
        DeclareLaunchArgument('env_file',
                              default_value=os.path.join(os.getcwd(), '.env'),
                              description='Fichero con TELEGRAM_BOT_TOKEN y ANTHROPIC_API_KEY'),
        OpaqueFunction(function=_nodo),
    ])
