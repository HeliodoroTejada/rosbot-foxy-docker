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

# Simulación completa
## Lanzar Entorno
```bash
cd ~/carpeta_compartida/husarion_ws
./simulacion.sh
```

## Lanzar Nodo Exploracion Simulacion

```bash
cd ~/carpeta_compartida/husarion_ws
source install/setup.sh
ros2 launch rosbot_exploration exploration.launch.py use_sim_time:=true autostart:=true
```


# Robot Real
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
## Lanzar configuracion de Rviz
```bash
ros2 run rviz2 rviz2 \
  -d ~/carpeta_compartida/husarion_ws/config/rviz_config.rviz \
  --ros-args -p use_sim_time:=false
```
Se le deben comandar topics del tipo 2D Goal Pose. Esperar un poco hasta que salgan los puntos del lidar en rojo para comandarle el primer punto

## Lanzar Nodo Exploracion Rosbot
```bash
cd ~/carpeta_compartida/husarion_ws
source install/setup.sh
ros2 launch rosbot_exploration exploration.launch.py use_sim_time:=false autostart:=true
```


