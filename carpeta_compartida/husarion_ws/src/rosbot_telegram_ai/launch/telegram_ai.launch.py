import math
import os
import socket
import subprocess

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
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


def _resolver_robot(nombre):
    """IP del robot a partir de su nombre ('ubuntu'). '' si no se encuentra.

    Primero con el resolvedor normal; en WSL, si falla, se lo pregunta a Windows,
    que sí encuentra los equipos de la red local por nombre.
    """
    for candidato in (nombre, nombre + '.local'):
        try:
            return socket.gethostbyname(candidato)
        except OSError:
            pass
    if os.path.isdir('/mnt/c/Windows'):
        orden = ("([System.Net.Dns]::GetHostAddresses('%s') | "
                 "? AddressFamily -eq 'InterNetwork' | Select -First 1).IPAddressToString" % nombre)
        try:
            salida = subprocess.run(['powershell.exe', '-NoProfile', '-Command', orden],
                                    capture_output=True, text=True, timeout=15).stdout.strip()
            socket.inet_aton(salida)
            return salida
        except (OSError, subprocess.SubprocessError, ValueError):
            pass
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
        if robot_ip == 'auto':
            robot_ip = _resolver_robot(LaunchConfiguration('robot_host').perform(context))
            print('[telegram_ai.launch] IP del robot: %s' % (robot_ip or 'no encontrada (sólo multicast)'))
        entorno = {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp', 'ROS_LOCALHOST_ONLY': '0',
                   'CYCLONEDDS_URI': os.environ.get('CYCLONEDDS_URI', _cyclone_real(robot_ip))}
    # Se aplica a todos los procesos de este launch (bot y explorador).
    acciones = [SetEnvironmentVariable(k, v) for k, v in entorno.items()]
    acciones.append(Node(
        package='rosbot_telegram_ai', executable='telegram_ai_node', name='telegram_ai',
        output='screen',
        parameters=[LaunchConfiguration('params_file').perform(context), {
            'use_sim_time': modo == 'sim',
            # En el robot real el lidar va girado 180º (laser_yaw en bringup.launch.py).
            'lidar_yaw': -math.pi if modo == 'real' else 0.0,
            'env_file': LaunchConfiguration('env_file').perform(context),
        }],
    ))

    # Explorador por fronteras (paquete rosbot_exploration del equipo), parado hasta
    # que el bot llame a /exploration/start. No se usa su launch porque cierra todo el
    # launch si el explorador termina; así un fallo suyo no tumba el bot.
    if LaunchConfiguration('exploracion').perform(context) == 'true':
        try:
            share_explo = get_package_share_directory('rosbot_exploration')
        except PackageNotFoundError:
            print('[telegram_ai.launch] rosbot_exploration no está compilado: sin exploración')
        else:
            acciones.append(Node(
                package='rosbot_exploration', executable='explore', output='screen',
                parameters=[os.path.join(share_explo, 'config', 'exploration.yaml'),
                            {'use_sim_time': modo == 'sim', 'autostart': False}],
                sigterm_timeout='15', sigkill_timeout='5'))
    return acciones


def generate_launch_description():
    share = get_package_share_directory('rosbot_telegram_ai')
    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='sim', description="'sim' o 'real'"),
        DeclareLaunchArgument('robot_ip', default_value='auto',
                              description="IP del robot en modo real ('auto' = buscarla por "
                                          "su nombre, '' = sólo multicast)"),
        DeclareLaunchArgument('robot_host', default_value='ubuntu',
                              description='Nombre del robot en la red (para robot_ip:=auto)'),
        DeclareLaunchArgument('exploracion', default_value='true', choices=['true', 'false'],
                              description='Lanzar también el explorador (rosbot_exploration)'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'telegram_ai.yaml')),
        DeclareLaunchArgument('env_file',
                              default_value=os.path.join(os.getcwd(), '.env'),
                              description='Fichero con TELEGRAM_BOT_TOKEN y ANTHROPIC_API_KEY'),
        OpaqueFunction(function=_nodo),
    ])
