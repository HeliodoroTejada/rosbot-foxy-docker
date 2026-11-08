# DWB Core de Foxy: corrección de publicación de trayectorias

Este directorio incorpora únicamente `nav2_dwb_controller/dwb_core` de
[Navigation2 0.4.7](https://github.com/ros-navigation/navigation2/tree/0.4.7/nav2_dwb_controller/dwb_core).
Conserva las licencias y los encabezados originales BSD-3-Clause.

Archivo de origen:
`https://codeload.github.com/ros-navigation/navigation2/tar.gz/refs/tags/0.4.7`

SHA-256 del archivo de origen:
`a91d5bbe3b99bacbfa2aae53254cf497e3372c12db1b28228df0e173bf62cfdc`

## Cambio local

`src/publisher.cpp`: la publicación de `/marker` depende de sus propios
suscriptores y de `FollowPath.publish_trajectories`, sin exigir un consumidor
de `/evaluation`. Se conserva la publicación de evaluaciones cuando está
habilitada y tiene suscriptores.

`test/publisher_test.cpp`: pruebas ROS de publicación independiente de ambos
tópicos, flags y evaluaciones nulas. Se ejecutan en dominio 174 y localhost.
Se añade la dependencia de pruebas `ament_cmake_gtest` al manifiesto.

## Compilación y uso

Usar un entorno ROS 2 Foxy con las dependencias DWB/Nav2 0.4.7 instaladas.
Los encabezados originales coinciden con los del paquete binario inspeccionado.
No se recompila ni actualiza el resto de Nav2.

Desde el workspace:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select dwb_core \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select dwb_core --event-handlers console_direct+
colcon test-result --test-result-base build/dwb_core --verbose
```

Relanzar la navegación después de compilar y cargar el overlay. Tanto
`simulacion.sh` como `launch_rosbot.sh` ya cargan `install/setup.bash`.
En el robot físico hay que compilar y cargar también este paquete: los cambios
del contenedor no modifican la instalación del robot.

Con `publish_trajectories: true`, RViz usa un display MarkerArray en `/marker`.
No hace falta ejecutar `ros2 topic echo /evaluation` ni añadir un nodo auxiliar.

## Validación realizada en el contenedor

La prueba `MarkersWithoutEvaluationSubscriber` falla con el código original y
pasa con la corrección. Pasan los seis casos del publicador, las dos pruebas
originales de utilidades y los controles de estilo y análisis del paquete.

También se comprobó Gazebo + SLAM + Nav2 + RViz en dominio 175, limitado a
localhost: el controlador carga `build/dwb_core/libdwb_core.so` del workspace y
publica 419 marcadores por mensaje. RViz muestra las alternativas siendo el
único suscriptor de `/marker`, con cero suscriptores de `/evaluation`.
