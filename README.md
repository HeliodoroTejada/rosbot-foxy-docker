## Contenedor
# Construir, SOLO PRIMERA VEZ
```bash
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose up -d --build
```
# Correr contenedor
```bash
docker start ubuntu20_foxy_rosbot	==> Arrancar contenedor
docker stop ubuntu20_foxy_rosbot	==> Parar contenedor

docker exec -it ubuntu20_foxy_rosbot bash	==> Abrir terminal (ROS 2 Foxy se carga automáticamente)
```

## Compilar desde cero

```bash
cd ~/carpeta_compartida/husarion_ws
colcon build --symlink-install
colcon build --symlink-install --packages-select
```

## Permisos para abrir ventanas gráficas

Si no tienes permisos para abrir ventanas gráficas desde el contenedor, ejecuta
este comando en una terminal del **host**, fuera del contenedor, y vuelve a abrir
la aplicación gráfica:

```bash
xhost +SI:localuser:$(id -un)
```

## Simulación completa

```bash
cd ~/carpeta_compartida/husarion_ws
./simulacion.sh
```


## Robot Real
Dentro del rosbot se lanza con un servicio todos los nodos de Ros al arrancar el robot. Para abrir Rviz en el contenedor en el portatil:

Dar permisos en el host para que el contenedor abra ventanas graficas, se ejecuta en un terminal fuera del contenedor
```bash
xhost +SI:localuser:$(id -un)
```
Entrar al contenedor si no lo hemos hecho
```bash
docker start ubuntu20_foxy_rosbot	==> Arrancar contenedor si no lo hemos hecho
docker exec -it ubuntu20_foxy_rosbot bash	==> Abrir terminal si no lo hemos hecho
```
Lanzar configuracion de Rviz
```bash
ros2 run rviz2 rviz2 \
  -d ~/carpeta_compartida/husarion_ws/config/rviz_config.rviz \
  --ros-args -p use_sim_time:=false
```
Se le deben comandar topics del tipo 2D Goal Pose
Esperar un poco hasta que salgan los puntos del lidar en rojo para comandarle el primer punto

## Alternativas de DWB en RViz

El workspace incluye `dwb_core` de Nav2 Foxy 0.4.7 con una corrección para que
el display **MarkerArray** de `/marker` funcione sin mantener un suscriptor
auxiliar de `/evaluation`. La procedencia y las pruebas están en
[dwb_core/UPSTREAM.md](carpeta_compartida/husarion_ws/src/dwb_core/UPSTREAM.md).

Después de incorporar este paquete, compilarlo dentro del contenedor:

```bash
cd ~/carpeta_compartida/husarion_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select dwb_core \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Relanzar la navegación para que cargue la biblioteca corregida. Los scripts
`simulacion.sh` y `launch_rosbot.sh` ya cargan este overlay. Para el robot real,
compilar también el paquete en `~/husarion_ws` del robot.

En `controller_server.ros__parameters.FollowPath`, activar
`publish_trajectories: True`. En RViz, usar **MarkerArray → `/marker`** para las
alternativas y **Path → `/plan`** para el camino global. Durante simulación,
RViz debe usar `use_sim_time:=true`; para el robot físico, `false`.
