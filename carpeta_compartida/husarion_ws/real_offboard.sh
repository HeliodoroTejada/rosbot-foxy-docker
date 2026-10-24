#!/usr/bin/env bash
set -eo pipefail

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    printf '%s\n' 'Uso en el PC: ./real_offboard.sh [argumentos ROS launch]' \
        'Arranca SLAM, Nav2 y RViz con config/rviz_config.rviz y reloj real.' \
        'Ejemplo: ./real_offboard.sh nav_params:=/ruta/nav2_calibrado.yaml' \
        'Arranca real_onboard.sh en el robot con el mismo ROS_DOMAIN_ID y RMW.'
    exit 0
fi

# shellcheck source=scripts/entorno_experimento.bash
source "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/scripts/entorno_experimento.bash"
preparar_experimento offboard
exec ros2 launch rosbot_description experimento.launch.py \
    modo:=offboard rviz_config:="$RVIZ_EXPERIMENTO_CONFIG" "$@"
