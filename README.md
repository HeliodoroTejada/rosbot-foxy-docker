# ROSbot con Ubuntu 20.04, ROS 2 Foxy y Docker

Workspace para arrancar la simulación ROSbot en Gazebo Classic, con SLAM Toolbox, Nav2 y RViz. Incluye también los scripts separados del robot real y del PC externo.

## Requisitos

- PC Linux con Docker Engine y Docker Compose v2, y permiso para usar Docker.
- Sesión gráfica X11 o XWayland, variable `DISPLAY` y acceso a `/tmp/.X11-unix`.
- Git y acceso al repositorio privado; Internet para construir la imagen por primera vez.

La configuración se ha probado en Linux x86_64. No se garantiza el mismo funcionamiento en macOS, Windows/Docker Desktop, equipos ARM ni servidores sin pantalla. Ubuntu 20.04, Foxy y Gazebo Classic son versiones antiguas que se conservan por compatibilidad con este robot.

## Clonar y lanzar la simulación

```bash
git clone git@github.com:HeliodoroTejada/rosbot-foxy-docker.git
cd rosbot-foxy-docker

# Permitir al usuario del contenedor acceder a la pantalla del host.
xhost +si:localuser:"$(id -un)"

# Crear la imagen con el mismo UID/GID del usuario del host.
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose up -d --build

# Compilar desde las fuentes del clon. No se distribuyen binarios compilados.
docker exec -it ubuntu20_foxy bash -lc 'cd ~/carpeta_compartida/husarion_ws && ./compilar.sh'

# Abrir Gazebo con cámara, Nav2, SLAM y RViz.
docker exec -it ubuntu20_foxy bash -lc 'cd ~/carpeta_compartida/husarion_ws && ./simulacion.sh'
```

`compilar.sh` compila por defecto sólo `rosbot_description`, suficiente para la simulación: los sensores y la base simulados son plugins de Gazebo. No se necesitan los drivers USB físicos para simular.

**Ctrl+C** en la terminal de simulación cierra los cuatro comandos y sus procesos hijos. Si un nodo se bloquea, el script escala a SIGTERM y SIGKILL únicamente sobre sus grupos. No hace falta ejecutar manualmente más lanzadores ni recompilar para editar el `.sh`.

RViz carga `carpeta_compartida/husarion_ws/config/rviz_config.rviz`; su vista Map muestra `/map` de SLAM. El escenario Willow Garage está incluido con su licencia, por lo que no necesita descargarse al arrancar.

Si ya existe un contenedor llamado `ubuntu20_foxy`, conservar sus datos antes de recrearlo. No mantener dos instancias del pipeline en el mismo dominio ROS. La carpeta compartida persiste en el host; los archivos guardados únicamente dentro de la capa del contenedor no se conservan al recrearlo.

## Uso diario

Desde la raíz del repositorio:

```bash
docker compose up -d
docker exec -it ubuntu20_foxy bash
```

Dentro del contenedor:

```bash
cd ~/carpeta_compartida/husarion_ws
./simulacion.sh
```

Después de modificar fuentes que requieran compilación, ejecutar `./compilar.sh` de nuevo. Utiliza `--symlink-install` para que configuraciones y recursos instalados apunten al workspace del equipo actual.

Para parar el contenedor y retirar el permiso gráfico:

```bash
docker compose stop
xhost -si:localuser:"$(id -un)"
```

## Estructura y archivos generados

```text
Dockerfile, compose.yaml      Imagen Foxy y contenedor con red host/X11
carpeta_compartida/husarion_ws/
  compilar.sh                 Compilación local de simulación o hardware
  simulacion.sh               Los cuatro comandos manuales con salida por Ctrl+C
  real_onboard.sh             Drivers, odometría y watchdog en el robot
  real_offboard.sh            SLAM, Nav2 y RViz en el PC
  config/rviz_config.rviz     Configuración guardada de RViz
  scripts/                    Entorno común de los lanzadores reales
  src/                        rosbot_description, bringup, Astra y RPLIDAR
```

`build/`, `install/`, `log/`, `node_modules/`, cachés, binarios y `.env` están excluidos de Git. Se generan en cada ordenador; no se copian entre arquitecturas ni se necesitan para clonar. Las mallas y texturas del robot/mundo son recursos fuente necesarios y sí se incluyen.

## Robot real y ordenador externo

Compilar en cada equipo para su propia arquitectura:

```bash
cd ~/carpeta_compartida/husarion_ws
./compilar.sh --hardware
```

Esta opción descarga y compila el SDK OpenNI2 fijado por el paquete Astra; requiere Internet. No ejecuta drivers ni mueve motores. En el robot, que debe tener acceso a sus dispositivos seriales/USB, ejecutar `./real_onboard.sh`. En el PC con pantalla, ejecutar `./real_offboard.sh`. En ambos equipos usar el mismo `ROS_DOMAIN_ID` y `RMW_IMPLEMENTATION`.

El Compose de simulación no expone puertos seriales/USB físicos. El uso del contenedor en hardware exige configurar sus dispositivos y permisos reales; alternativamente ejecutar el onboard en el sistema Foxy del robot. Puertos, TF físicas, firmware y frenado necesitan validación sobre el equipo real.

Los scripts reales habilitan la comunicación fuera de localhost y usan reloj real. Sincronizar los relojes de robot y PC; mantener una sola fuente de `odom → base_link` y otra de `map → odom`. El watchdog local entrega ceros si caducan los comandos durante 0,5 s; no sustituye el timeout del firmware ni la parada física. El driver Astra actual no completa CameraInfo RGB ni `/camera/points` para las vistas de cámara/nube de RViz.

## Configuración y límites conocidos

Sólo se utiliza `compose.yaml`. Los valores por defecto están en ese archivo: dominio ROS 20, Fast DDS, simulación limitada a localhost y renderizado por software. No hace falta crear un `.env`. Para cambiar una variable, exportarla antes de crear el contenedor; por ejemplo:

```bash
export ROS_DOMAIN_ID=30
docker compose up -d
```

El renderizado por software evita depender de la GPU del ordenador. El acceso directo a una GPU requiere adaptar dispositivos, permisos y driver en `compose.yaml` para ese host.

La automatización y la compilación reproducible no corrigen por sí solas los fallos conocidos de este stack Foxy: se han observado bloqueos de Nav2 al reiniciar, errores de shader GLSL y fallos de cierre de RViz. Comprobar `/map`, TF y estados lifecycle antes de enviar objetivos; no se declara estabilidad prolongada ni validación física del robot.

Los comandos y opciones de los scripts están en [ARRANQUE.md](carpeta_compartida/husarion_ws/ARRANQUE.md).

## Origen y licencias

Se conservan los avisos y licencias incluidos en las fuentes. Cada paquete mantiene su declaración en `package.xml`.

| Componente | Origen y licencia conservados |
| --- | --- |
| `rosbot_ros` (`rosbot_description`) | Husarion `rosbot_description`, rama Foxy, con adaptaciones locales; MIT en su `package.xml`. |
| `ros_astra_camera` (`astra_camera`) | Driver Orbbec Astra; BSD en su `package.xml` y avisos originales. |
| `rplidar_ros` | Driver RPLIDAR; [licencia BSD](carpeta_compartida/husarion_ws/src/rplidar_ros/LICENSE). |
| `rosbot_pro_bringup` | Bringup y adaptaciones locales; MIT en su `package.xml`. |
| Willow Garage | OSRF `gazebo_models`, revisión `8163eb4b5e7e21985c6591d1c0bfb56468c0093f`; [licencia CC BY 3.0](carpeta_compartida/husarion_ws/src/rosbot_ros/models/willowgarage/LICENSE) y [procedencia](carpeta_compartida/husarion_ws/src/rosbot_ros/models/willowgarage/UPSTREAM.md). |
| OpenNI2 Orbbec | El driver Astra descarga `ob-tim-liu/OpenNI2`, commit `bbabd58c85d7c8f945f4f679bf73b653bd9df3e7`, al compilar hardware. Sus avisos se conservan en el SDK descargado; sus binarios no se incluyen en Git. |

ROS, Gazebo y las dependencias instaladas por Docker conservan las licencias de sus respectivos paquetes.

## Validación del clon limpio

El 24 de octubre de 2026 se reconstruyó la imagen y se extrajeron únicamente los archivos del árbol Git a un directorio temporal, sin `build/`, `install/` ni `log/`. En un contenedor nuevo, `./compilar.sh` terminó correctamente; los cuatro comandos arrancaron, SLAM publicó `/map` y Ctrl+C cerró todos los procesos hijos. Las compilaciones y los registros de esa prueba quedaron fuera del repositorio. Esto comprueba reconstrucción y arranque, no estabilidad prolongada de Nav2 ni hardware.
