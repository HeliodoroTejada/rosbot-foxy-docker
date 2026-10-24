#!/usr/bin/env bash
set -eo pipefail

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
paquetes=(rosbot_description)
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    printf '%s\n' 'Uso: ./compilar.sh [--hardware] [opciones adicionales de colcon build]' \
        'Sin opciones compila rosbot_description, suficiente para la simulación.' \
        '--hardware añade bringup, RPLIDAR y Astra (descarga el SDK OpenNI2 fijado).' \
        'Ejemplo: ./compilar.sh --hardware --parallel-workers 2'
    exit 0
fi
if [[ "${1:-}" == --hardware ]]; then
    paquetes+=(rosbot_pro_bringup rplidar_ros astra_camera)
    shift
fi
if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
    printf 'Ejecuta este script dentro del contenedor ubuntu20_foxy o en un equipo con ROS Foxy.\n' >&2
    exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
cd -- "$WS" || exit 1
exec colcon build --symlink-install --packages-select "${paquetes[@]}" "$@"
