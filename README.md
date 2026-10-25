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
