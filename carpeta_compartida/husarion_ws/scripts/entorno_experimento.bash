# Entorno común de los scripts de arranque. Se carga con source.
preparar_experimento() {
    local modo="$1"
    local directorio_scripts
    directorio_scripts="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
    EXPERIMENTO_WS="$(cd -- "$directorio_scripts/.." && pwd)"

    if [[ ! -f /opt/ros/foxy/setup.bash ]]; then
        printf 'No se encuentra ROS Foxy. Ejecuta este script dentro del contenedor ubuntu20_foxy o en el robot con Foxy instalado.\n' >&2
        return 1
    fi
    if [[ ! -f "$EXPERIMENTO_WS/install/setup.bash" ]]; then
        printf 'Falta compilar el workspace: cd "%s" && colcon build\n' "$EXPERIMENTO_WS" >&2
        return 1
    fi

    # Los setup de ROS/Gazebo no son compatibles con set -u en Foxy.
    set +u
    if [[ "$modo" == simulacion ]]; then
        if [[ ! -f /usr/share/gazebo/setup.bash ]]; then
            printf 'No se encuentra Gazebo en /usr/share/gazebo/setup.bash.\n' >&2
            return 1
        fi
        # shellcheck disable=SC1091
        source /usr/share/gazebo/setup.bash
    fi
    # shellcheck disable=SC1091
    source /opt/ros/foxy/setup.bash
    # shellcheck disable=SC1091
    source "$EXPERIMENTO_WS/install/setup.bash"
    set -u

    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    if [[ "$modo" == simulacion ]]; then
        export ROS_LOCALHOST_ONLY=1
    else
        export ROS_LOCALHOST_ONLY=0
    fi
    if [[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp && -z "${CYCLONEDDS_URI:-}" ]]; then
        export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
    fi

    if [[ "$modo" != onboard ]]; then
        if [[ -z "${DISPLAY:-}" ]]; then
            printf 'Falta DISPLAY: abre una terminal con acceso a la pantalla para Gazebo/RViz.\n' >&2
            return 1
        fi
        RVIZ_EXPERIMENTO_CONFIG="$EXPERIMENTO_WS/config/rviz_config.rviz"
        if [[ ! -f "$RVIZ_EXPERIMENTO_CONFIG" ]]; then
            printf 'No existe la configuración RViz: %s\n' "$RVIZ_EXPERIMENTO_CONFIG" >&2
            return 1
        fi
    fi
    cd -- "$EXPERIMENTO_WS" || return 1
    printf 'Modo: %s | ROS_DOMAIN_ID=%s | RMW=%s\n' "$modo" "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION"
    printf 'Ctrl+C detiene este lanzamiento.\n'
}
