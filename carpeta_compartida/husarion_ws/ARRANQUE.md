# Arranque del workspace

La instalación Docker y el primer clon se explican en [README.md](../../README.md). Los scripts se ejecutan dentro de Foxy y localizan el workspace a partir de su propia ruta.

## Compilar desde cero

```bash
./compilar.sh             # Sólo rosbot_description: simulación
./compilar.sh --hardware  # Añade bringup, RPLIDAR y Astra
```

No se distribuyen `build/`, `install/` ni `log/`; se generan aquí y están ignorados por Git. La opción de hardware descarga OpenNI2 y requiere Internet. No copiar compilaciones entre arquitecturas.

## Simulación completa

```bash
./simulacion.sh
```

Lanza directamente los cuatro comandos de Gazebo con ventana/cámara, Nav2 con `nav2_params_sim.yaml`, `slam_toolbox_sim.launch.py` y RViz con `config/rviz_config.rviz`. No usa el lanzamiento integrado `experimento.launch.py`. Todos usan tiempo simulado.

Ctrl+C cierra los cuatro comandos y sus hijos. Tras 12 s sin respuesta escala a SIGTERM y, después de otros 5 s, a SIGKILL, sólo sobre sus grupos. Si uno termina, se cierra el resto. Las variables ROS de la terminal se conservan; la configuración por defecto de Compose usa dominio 20, Fast DDS y localhost.

La vista Map muestra `/map` de SLAM con durabilidad Transient Local. Para aplicar cambios del archivo con RViz abierto, usar **File → Open Config**. Un robot parado sólo mapea su entorno visible.

## Experimento real

La cámara se activa por defecto en `bringup.launch.py`. Junto al driver se
lanza el conversor de `astra_camera/launch/depth_to_cloud.launch.py`, que usa
las funciones de proyección de `depth_image_proc` mediante el nodo
`astra_camera/depth_to_cloud_node` para publicar `/camera/points` (`PointCloud2`) a partir de
`/camera/depth/image_raw` y `/camera/depth/camera_info`. Nav2 consume esa nube
en el costmap local. La salida conserva el frame óptico de la profundidad.

Para reducir la carga del robot, el conversor toma por defecto un píxel de cada
dos en cada eje (`pixel_stride=2`) y genera como máximo 5 nubes/s (`max_rate=5.0`).
Con profundidad de 640×480, publica 320×240 puntos: 76.800 puntos y unos 1,23 MB
por nube, hasta unos 6,14 MB/s de datos por suscriptor remoto, antes de las
cabeceras de transporte. Descarta los fotogramas excedentes antes de copiar o
proyectar; no genera primero una nube completa. Ajusta los intrínsecos de la
proyección y conserva el timestamp y el frame originales. El muestreo no filtra
el ruido y reduce el detalle de obstáculos pequeños: validar su detección en
el robot antes de dar por ajustada la navegación.

El bringup expone `camera_cloud_pixel_stride` y `camera_cloud_max_rate` al
arrancar. Sus valores predeterminados se aplican también desde `launch_rosbot.sh`,
sin editar ese script. Para probar sólo el conversor con un driver ya abierto:

```bash
ros2 launch astra_camera depth_to_cloud.launch.py pixel_stride:=2 max_rate:=5.0
```

Para recuperar la salida original, usar `pixel_stride:=1 max_rate:=0.0` en ese
launch, o `camera_cloud_pixel_stride:=1 camera_cloud_max_rate:=0.0` en el bringup.
Estos límites se configuran al arrancar. No limitan la captura del driver (que
sigue a 640×480 y 30 FPS solicitados), ni cambian LiDAR, SLAM o parámetros de Nav2.
La cámara simulada de Gazebo publica su propia nube y no pasa por este conversor.
Comprobar CPU y continuidad de TF con la nueva configuración: la reducción no
garantiza por sí sola resolver toda la saturación observada.

Para actualizar un robot que ya tiene este workspace compilado:

```bash
sudo apt-get update
sudo apt-get install ros-foxy-depth-image-proc
cd ~/husarion_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
colcon build --packages-select astra_camera rosbot_pro_bringup \
  --symlink-install --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`launch_rosbot.sh` sigue arrancando el bringup, SLAM y navegación como antes;
no necesita un comando adicional para la conversión. `start_camera:=false`
desactiva tanto la cámara como el conversor. La conversión usa los parámetros
de calibración publicados por el driver; su precisión y las TF deben validarse
con la cámara física.

En el robot, con el workspace compilado allí:

```bash
./real_onboard.sh serial_port:=/dev/ttyS4 lidar_port:=/dev/ttyUSB0
# Cámara opcional:
./real_onboard.sh start_camera:=false
```

En el PC externo con pantalla:

```bash
./real_offboard.sh
# Perfil calibrado opcional:
./real_offboard.sh nav_params:=/ruta/nav2_calibrado.yaml
```

Onboard ejecuta base, odometría, láser, cámara y watchdog; offboard ejecuta SLAM asíncrono, Nav2 y RViz con la configuración guardada y reloj real. No se envían velocidades ni objetivos automáticamente. Parar offboard antes que onboard.

Los scripts reales cargan `scripts/entorno_experimento.bash`, usan por defecto dominio 20 y Fast DDS y habilitan la red. Usar el mismo dominio/RMW y relojes sincronizados en ambos equipos. Puertos, TF, calibración y frenado deben comprobarse en hardware; el Compose base no da acceso USB/serial al contenedor.

## Detalles

`./simulacion.sh --help`, `./real_onboard.sh --help`, `./real_offboard.sh --help` y `./compilar.sh --help` muestran ayuda. Gazebo/RViz requieren `DISPLAY` y acceso X11. Los scripts no recrean el contenedor ni activan la GPU automáticamente.

Se han observado bloqueos al reiniciar Nav2 y errores gráficos/de cierre en Foxy; los scripts no se presentan como solución a esos fallos. Red, cámara física y watchdog se detallan en [README.md](../../README.md#robot-real-y-ordenador-externo).
