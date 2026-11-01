#!/usr/bin/env bash
set -eo pipefail
set +m

ROSBOT_WS="${ROSBOT_WS:-/home/husarion/husarion_ws}"

for setup in /opt/ros/foxy/setup.bash "$ROSBOT_WS/install/setup.bash"; do
    if [[ ! -r "$setup" ]]; then
        printf 'No se encuentra: %s\n' "$setup" >&2
        exit 1
    fi
done

source /opt/ros/foxy/setup.bash
source "$ROSBOT_WS/install/setup.bash"
set -u
cd "$ROSBOT_WS"

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONUNBUFFERED=1

# Perfil DDS que estamos probando; no modifica archivos.
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain>
    <General>
      <MaxMessageSize>1400B</MaxMessageSize>
      <FragmentSize>1280B</FragmentSize>
    </General>
    <Discovery>
      <MaxAutoParticipantIndex>50</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>'

command -v setsid >/dev/null || {
    printf 'Falta el comando setsid.\n' >&2
    exit 1
}

pids=()

enviar_senal() {
    local pid
    for pid in "${pids[@]}"; do
        # Señalar el grupo completo creado por setsid.
        kill -s "$1" -- "-$pid" 2>/dev/null || true
    done
}

esperar_grupos() {
    local limite=$((SECONDS + $1))
    local pid activos

    while (( SECONDS < limite )); do
        activos=0

        for pid in "${pids[@]}"; do
            if kill -0 -- "-$pid" 2>/dev/null; then
                activos=1
            fi
        done

        if (( activos == 0 )); then
            return 0
        fi

        sleep 0.2
    done

    return 1
}

detener() {
    trap - EXIT
    trap '' INT TERM

    printf '\nDeteniendo los lanzamientos ROS y sus procesos...\n'

    # Primero, solicitar una parada limpia equivalente a Ctrl+C.
    enviar_senal INT

    if ! esperar_grupos 10; then
        printf 'Quedan procesos: enviando SIGTERM...\n'
        enviar_senal TERM

        if ! esperar_grupos 5; then
            printf 'Quedan procesos: forzando el cierre con SIGKILL...\n'
            enviar_senal KILL
        fi
    fi

    local pid
    for pid in "${pids[@]}"; do
        wait "$pid" 2>/dev/null || true
    done

    printf 'Parada finalizada.\n'
}

trap detener EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

lanzar() {
    printf '\nIniciando: ros2 launch %s %s\n' "$1" "$2"
    setsid ros2 launch "$@" &
    pids+=("$!")
}

printf 'Workspace: %s | Dominio: %s | RMW: %s\n' \
    "$ROSBOT_WS" "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION"

lanzar rosbot_pro_bringup bringup.launch.py
lanzar rosbot_description slam_toolbox.launch.py
lanzar rosbot_description rosbot_navigation_pro.launch.py

# Si termina un lanzamiento, detener el resto y devolver un fallo.
# Esto permite que systemd reinicie el conjunto en el futuro.
estado=0
wait -n "${pids[@]}" || estado=$?

printf 'Ha terminado uno de los lanzamientos (estado %s).\n' "$estado" >&2

if (( estado == 0 )); then
    estado=1
fi

exit "$estado"