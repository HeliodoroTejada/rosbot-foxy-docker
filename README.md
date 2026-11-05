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

## Bot de Telegram con IA (`rosbot_telegram_ai`)

Nodo que habla por Telegram usando Claude: puede mirar la cámara ("¿qué ves?"),
leer el lidar y mover el robot publicando en `/cmd_vel` ("da dos vueltas en círculo").
Por defecto usa Claude Code con tu cuenta de Claude (`backend: claude_code`).

1. Crea un bot con [@BotFather](https://t.me/BotFather) y copia su token.
2. Crea `~/carpeta_compartida/husarion_ws/.env` (está en `.gitignore`, no se sube):
   ```bash
   TELEGRAM_BOT_TOKEN=123456:ABC...
   ```
3. Si el contenedor se construyó antes de este cambio, instala Claude Code dentro:
   `curl -fsSL https://claude.ai/install.sh | bash`
4. Inicia sesión una vez dentro del contenedor: ejecuta `claude`, sigue el enlace
   de inicio de sesión con tu cuenta y sal con `/exit`.
   (Para usar una API key en vez de tu cuenta: `backend: api` en el YAML,
   `ANTHROPIC_API_KEY=...` en el `.env` y `python3 -m pip install --user anthropic`.)
5. Compila y lanza (con la simulación o el robot ya arrancados):
   ```bash
   cd ~/carpeta_compartida/husarion_ws
   colcon build --symlink-install --packages-select rosbot_telegram_ai
   source install/setup.bash
   ros2 launch rosbot_telegram_ai telegram_ai.launch.py modo:=sim   # o modo:=real
   ```
6. Escribe `/id` al bot, pon ese número en `allowed_chat_ids` de
   `src/rosbot_telegram_ai/config/telegram_ai.yaml` y vuelve a lanzar. Sólo esos chats pueden mover el robot.

Comandos: `/foto`, `/parar` (inmediato, sin pasar por la IA), `/reset` (olvida la conversación).
Límites de velocidad, duración y distancia de parada por obstáculo en el mismo YAML.