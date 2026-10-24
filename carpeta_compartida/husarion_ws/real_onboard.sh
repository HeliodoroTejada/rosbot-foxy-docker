#!/usr/bin/env bash
set -eo pipefail

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    printf '%s\n' 'Uso en el ROBOT: ./real_onboard.sh [argumentos del bringup]' \
        'Arranca base, odometría, láser, cámara y watchdog local. No abre ventanas.' \
        'Ejemplo: ./real_onboard.sh serial_port:=/dev/ttyS4 lidar_port:=/dev/ttyUSB0' \
        'Para desactivar la cámara: ./real_onboard.sh start_camera:=false' \
        'Usa el mismo ROS_DOMAIN_ID y RMW que real_offboard.sh en el PC.'
    exit 0
fi

# shellcheck source=scripts/entorno_experimento.bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/scripts/entorno_experimento.bash"
preparar_experimento onboard
exec ros2 launch rosbot_pro_bringup bringup.launch.py \
    start_camera:=true command_watchdog:=true command_timeout:=0.5 "$@"
