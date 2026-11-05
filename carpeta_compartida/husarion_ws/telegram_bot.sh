#!/usr/bin/env bash
# Lanza el bot de Telegram (rosbot_telegram_ai) contra el robot real.
# Uso: ./telegram_bot.sh [argumentos extra de ros2 launch, p. ej. robot_ip:=10.105.34.33]
set -eo pipefail

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash

# En la distro WSL el paquete se compila en ~/tg_ws; en el contenedor, en el propio workspace.
if [[ -f "$HOME/tg_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$HOME/tg_ws/install/setup.bash"
else
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
fi
export PATH="$HOME/.local/bin:$PATH"

cd -- "$WS"
# El YAML del código fuente: así los cambios (p. ej. allowed_chat_ids) valen sin recompilar.
exec ros2 launch rosbot_telegram_ai telegram_ai.launch.py modo:=real \
    params_file:="$WS/src/rosbot_telegram_ai/config/telegram_ai.yaml" "$@"
