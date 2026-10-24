#!/usr/bin/env bash
set -eo pipefail
# Cada comando tendrá su propia sesión/grupo, independiente de esta terminal.
set +m

if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    printf '%s\n' 'Uso: ./simulacion.sh' \
        'Ejecuta los cuatro comandos de Gazebo, navegación, SLAM y RViz.' \
        'Gazebo abre su ventana y la cámara. RViz carga config/rviz_config.rviz.' \
        'Ctrl+C cierra los cuatro comandos y sus procesos hijos.'
    exit 0
fi
if (( $# )); then
    printf 'Uso: %s [--help]\n' "$0" >&2
    exit 2
fi

WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for archivo in /opt/ros/foxy/setup.bash /usr/share/gazebo/setup.bash \
               "$WS/install/setup.bash" "$WS/config/rviz_config.rviz"; do
    if [[ ! -f "$archivo" ]]; then
        printf 'No existe %s. Ejecuta el script en el entorno Foxy con el workspace compilado.\n' "$archivo" >&2
        exit 1
    fi
done
command -v setsid >/dev/null || { printf 'Falta setsid (paquete util-linux).\n' >&2; exit 1; }
if [[ -z "${DISPLAY:-}" ]]; then
    printf 'Falta DISPLAY para abrir Gazebo y RViz.\n' >&2
    exit 1
fi

# Cargar los mismos entornos que en los comandos manuales. Foxy no admite set -u.
# shellcheck disable=SC1091
source /opt/ros/foxy/setup.bash
# shellcheck disable=SC1091
source /usr/share/gazebo/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
cd -- "$WS" || exit 1
NAV_PARAMS="$(ros2 pkg prefix rosbot_description)/share/rosbot_description/config/nav2_params_sim.yaml"
if [[ ! -f "$NAV_PARAMS" ]]; then
    printf 'No existe el perfil de navegación: %s\n' "$NAV_PARAMS" >&2
    exit 1
fi

pids=()
nombres=()

# Invocada desde el manejador de EXIT.
# shellcheck disable=SC2317
hay_procesos() {
    local pid
    for pid in "${pids[@]}"; do
        if kill -0 -- "-$pid" 2>/dev/null; then
            return 0
        fi
    done
    return 1
}

# shellcheck disable=SC2317
cerrar() {
    local resultado=$? i pid limite senal
    trap - EXIT
    trap '' INT TERM
    if (( ${#pids[@]} == 0 )); then
        exit "$resultado"
    fi
    printf '\nCerrando Gazebo, navegación, SLAM y RViz...\n'

    for i in "${!pids[@]}"; do
        pid="${pids[$i]}"
        # ros2 launch propaga SIGINT a sus nodos. Evitar enviársela dos veces.
        # ros2 run espera que la terminal también interrumpa a su ejecutable.
        if [[ "${nombres[$i]}" != RViz ]] && kill -0 "$pid" 2>/dev/null; then
            kill -INT "$pid" 2>/dev/null || true
        else
            kill -INT -- "-$pid" 2>/dev/null || true
        fi
    done

    limite=$((SECONDS + 12))
    while hay_procesos && (( SECONDS < limite )); do
        sleep 0.2
    done
    # Si un nodo se bloquea, terminar únicamente los grupos de este script.
    for senal in TERM KILL; do
        hay_procesos || break
        for pid in "${pids[@]}"; do
            kill -"$senal" -- "-$pid" 2>/dev/null || true
        done
        if [[ "$senal" == TERM ]]; then
            limite=$((SECONDS + 5))
            while hay_procesos && (( SECONDS < limite )); do
                sleep 0.2
            done
        fi
    done
    for pid in "${pids[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    printf 'Simulación detenida.\n'
    exit "$resultado"
}

trap cerrar EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

arrancar() {
    local nombre="$1"
    shift
    printf '\nIniciando %s...\n' "$nombre"
    setsid "$@" &
    pids+=("$!")
    nombres+=("$nombre")
}

arrancar Gazebo ros2 launch rosbot_description rosbot_sim.launch.py \
    use_sim_time:=true gui:=true camera:=true
arrancar Navegación ros2 launch rosbot_description rosbot_navigation_sim.launch.py \
    use_sim_time:=true "params:=$NAV_PARAMS"
arrancar SLAM ros2 launch rosbot_description slam_toolbox_sim.launch.py
arrancar RViz ros2 run rviz2 rviz2 -d config/rviz_config.rviz \
    --ros-args -p use_sim_time:=true

printf '\nLos cuatro comandos están lanzados. Pulsa Ctrl+C para cerrar todo.\n'
resultado=0
wait -n "${pids[@]}" || resultado=$?
printf '\nUno de los comandos ha terminado (código %s); se cierra el resto.\n' "$resultado"
exit "$resultado"
