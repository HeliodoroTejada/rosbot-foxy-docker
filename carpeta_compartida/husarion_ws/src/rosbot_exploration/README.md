# Exploración autónoma del ROSbot (ROS 2 Foxy)

Paquete independiente que añade exploración por fronteras a SLAM Toolbox y Nav2
ya arrancados. Detecta límites entre espacio libre y desconocido y elige una
posición conocida libre, alcanzable y con margen para el robot. Envía una tarea
`NavigateToPose`; Nav2 planifica, controla y recupera esa tarea. No publica velocidades.

Compara varias posiciones por frontera mediante una estimación de información
visible, coste de desplazamiento/giro y proximidad al recorrido reciente. Los
descartes se asocian también a regiones de frontera y se revisan si cambia la escena.

El flujo sigue el patrón de tareas de
[BasicNavigator de Nav2 Humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_simple_commander/nav2_simple_commander/robot_navigator.py):
esperar aceptación, observar feedback, esperar el resultado y cancelar el objetivo
propio cuando corresponda. Utiliza las interfaces de Foxy, sin dependencias de Humble.

## Compilar

En el entorno Foxy y desde `husarion_ws`:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --packages-select rosbot_exploration
source install/setup.bash
```

Requiere los paquetes de Nav2 ya usados por este workspace y `python3-numpy`.
En el robot hay que compilar el paquete allí y cargar su `install/setup.bash`.

## Simulación

En una terminal, arrancar la simulación existente. Usar un dominio distinto del
robot físico si ambos están encendidos:

```bash
export ROS_DOMAIN_ID=21
./simulacion.sh
```

En otra terminal del mismo entorno, workspace y dominio:

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=21
ros2 launch rosbot_exploration exploration.launch.py use_sim_time:=true autostart:=true
```

**Ctrl+C en la terminal de exploración cancela su objetivo de Nav2 y cierra el
explorador.** Gazebo, SLAM y Nav2 continúan abiertos. Esperar a que termine el
mensaje de cancelación antes de enviar un objetivo manual desde RViz.

Después de actualizar el paquete, recompilarlo y reiniciar esta terminal de
exploración para cargar los cambios. Los scripts de simulación y robot no cambian.

## Robot real

Mantener el arranque habitual con `launch_rosbot.sh`. En otra terminal del robot:

```bash
source /opt/ros/foxy/setup.bash
source ~/husarion_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch rosbot_exploration exploration.launch.py use_sim_time:=false autostart:=true
```

Es preferible ejecutarlo en el robot, junto a Nav2. Si se ejecuta en un PC, debe
usar el mismo dominio, middleware y relojes que el robot; la cancelación depende
de que siga existiendo comunicación con Nav2.

## Control sin cerrar la terminal

El lanzamiento sin `autostart:=true` espera una orden explícita:

```bash
ros2 service call /exploration/start std_srvs/srv/Trigger '{}'
ros2 service call /exploration/pause std_srvs/srv/Trigger '{}'
ros2 service call /exploration/resume std_srvs/srv/Trigger '{}'
ros2 service call /exploration/stop std_srvs/srv/Trigger '{}'
```

`pause` y `stop` cancelan el objetivo. `resume` conserva la memoria de intentos;
`start` reinicia intentos y recorrido reciente. Esperar a completar una cancelación antes de reanudar.
Mientras explora, reservar los objetivos de Nav2 para este nodo. Detener la
exploración antes de enviar destinos desde RViz u otro cliente.

```bash
ros2 topic echo /exploration/state
```

Estados principales: `IDLE`, `WAITING`, `PLANNING`, `EXPLORING`, `CANCELING`,
`PAUSED`, `STOPPED`, `WAITING_RETRY`, `BLOCKED`, `CHECKING_COMPLETION`, `COMPLETED`.
En RViz se puede añadir un `MarkerArray` en `/exploration/frontiers` y un `Pose`
en `/exploration/goal`. No se modifica vuestra configuración RViz.
`/exploration/candidates` publica JSON con los candidatos válidos, ganancia
esperada en m², distancia estimada, coste de giro, penalización de revisita y puntuación.

## Configuración

Copiar y ajustar `config/exploration.yaml`, y pasar su ruta:

```bash
ros2 launch rosbot_exploration exploration.launch.py \
  params_file:=/ruta/exploration.yaml use_sim_time:=true autostart:=true
```

Los parámetros se aplican al arrancar. Los nombres de tópicos, acciones y nodos
lifecycle se configuran en ese YAML. El frame del mapa y del costmap debe
coincidir con `map_frame`.

Parámetros principales:

| Parámetro | Valor inicial | Función |
|---|---:|---|
| `robot_radius` | 0.25 m | Radio del modo circular y de la preplanificación estricta |
| `use_rectangular_footprint` | true | Comprueba la huella rectangular orientada en los destinos |
| `footprint_length` / `footprint_width` | 0.28 / 0.32 m | Dimensiones de la huella actual de Nav2 |
| `footprint_padding` | 0.02 m | Margen adicional alrededor del rectángulo |
| `min_frontier_size` | 0.35 m | Descarta fronteras pequeñas |
| `goal_standoff` | 0.45 m | Separación preferida respecto a la frontera |
| `max_standoff` | 0.9 m | Radio de búsqueda de una posición libre |
| `min_goal_distance` | 0.5 m | Desplazamiento mínimo; debe superar la tolerancia XY de Nav2 |
| `planner_frequency` | 0.5 Hz | Frecuencia máxima de búsqueda |
| `information_radius` | 3.0 m | Horizonte local de la estimación de visibilidad |
| `information_rays` | 72 | Rayos por candidato, en 360° |
| `information_candidates` | 12 | Máximo de candidatos evaluados con rayos por búsqueda |
| `candidates_per_frontier` | 3 | Alternativas espaciales por componente de frontera |
| `gain_weight` | 0.5 | Exponente de la ganancia estimada en la puntuación |
| `turn_weight` / `revisit_weight` | 0.35 / 1.0 | Penalizan giros y destinos próximos al recorrido reciente |
| `preplan` | false | Activa la comprobación estricta adicional con `ComputePathToPose` |
| `behavior_tree` | auto | Árbol del paquete seleccionado para cada tarea de navegación |
| `lookahead_distance` | 1.0 m | Prepara el siguiente destino al aproximarse al actual |
| `lookahead_max_age` | 5 s | Antigüedad máxima de los candidatos anticipados |
| `handoff_delay` | 0.2 s | Pausa mínima entre un resultado satisfactorio y el siguiente objetivo |
| `failure_backoff` | 0.5 s | Espera mínima después de un objetivo rechazado o abortado |
| `goal_invalid_timeout` | 1.5 s | Confirmación antes de cancelar por cambios en el destino |
| `progress_timeout` | 30 s | Cancela un objetivo sin acercamiento suficiente |
| `navigation_timeout` | 180 s | Duración máxima de un objetivo |
| `retry_delay` | 30 s | Exclusión temporal tras un fallo |
| `max_attempts` | 3 | Fallos/visitas antes de aplicar una exclusión prolongada |
| `blocked_retry_delay` | 180 s | Exclusión prolongada de regiones fallidas; nunca es permanente |
| `visit_cooldown` | 90 s | Descanso de posiciones visitadas reiteradamente |
| `retry_frontier_radius` | 0.75 m | Agrupa intentos fallidos sobre la misma región de frontera |
| `scene_change_ratio` | 0.1 | Cambio mínimo de conocimiento local para reconsiderar un descarte |
| `completion_confirmations` | 3 | Mapas distintos sin candidatos alcanzables |

La búsqueda usa conectividad aproximada con un disco inscrito en el rectángulo
acolchado. Comprueba después todas las celdas que intersectan la huella orientada
en cada posición candidata, incluidos su interior y los contactos en esquinas.
Esto admite pasos que el círculo de 50 cm de diámetro descartaba; no sustituye
la comprobación de trayectorias y maniobras de giro de Nav2. Mantener dimensiones
coherentes con la huella configurada en Nav2. La cámara continúa interviniendo
a través del controlador y costmap local existentes.

La visibilidad se estima con rayos que se detienen ante obstáculos conocidos y
los límites del mapa. Se integra el área desconocida potencialmente observable,
con pesos en m² y pasos radiales de media celda. Es una estimación: no predice
paredes aún desconocidas. Sólo se evalúan los candidatos más prometedores para
mantener un coste acotado y conservar la anticipación de destinos.

La puntuación (menor es mejor) es:

```text
(0.8 + distance_weight * distancia + turn_weight * giros + revisit_weight * revisita)
    / max(area_visible, 0.25) ** gain_weight
```

La distancia es una aproximación por conectividad, no una llamada adicional al
planificador. El coste de giro considera orientación actual, dirección al destino
y orientación final. El recorrido reciente utiliza muestras separadas 30 cm y
una penalización que decae con el tiempo.

Los fallos excluyen posiciones y regiones de frontera, evitando probar desde
otro punto la misma zona bloqueada. Antes de buscar se comparan parches métricos
locales del mapa y costmap con los guardados al fallar. La aparición de conocimiento
nuevo o la retirada neta de obstáculos puede reactivar la región; los timestamps,
redimensionados del mapa y pequeños desplazamientos de paredes no bastan. Las
exclusiones prolongadas también vencen por tiempo. Las visitas satisfactorias
nunca producen una exclusión permanente.

Con `preplan: true` se añade una comprobación conservadora del camino antes de
navegar: todos sus segmentos deben permanecer en espacio conocido con el radio
indicado por `robot_radius`. Puede descartar rutas válidas para la huella
rectangular de Nav2, especialmente si el robot ya está cerca de una pared. Su
`path_endpoint_tolerance` predeterminado es 0,5 m, igual que Navfn en este workspace.
Los rechazos distinguen frame, extremo del camino, cambios del destino y margen.

El siguiente destino se calcula en segundo plano al acercarse al actual. Sólo
se envía cuando finaliza la acción anterior; no hay preempciones periódicas ni
esperas obligatorias por un nuevo mapa después de cada llegada. Antes de usar
un candidato anticipado se comprueban su antigüedad, distancia real, memoria de
intentos y validez en el mapa/costmap actuales. La finalización se confirma desde
la posición real y con varios mapas distintos.

Un cambio transitorio de mapa/costmap en el destino no cancela inmediatamente
la tarea. Nav2 sigue evitando obstáculos con sus propios sensores y controlador;
si el destino continúa inválido durante `goal_invalid_timeout`, se cancela y se
elige otro.

## Árbol de comportamiento de exploración

`behavior_trees/explore_w_replanning.xml` usa los plugins Foxy ya cargados por el
workspace: planificación periódica a 1 Hz, seguimiento, limpieza y recuperaciones.
Su organización de recuperaciones alternadas sigue el patrón de
[Nav2 Humble](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml).
Amplía a 1.000 ms la espera de aceptación de acciones internas. En
[Nav2 0.4.7](https://github.com/ros-navigation/navigation2/blob/0.4.7/nav2_bt_navigator/src/bt_navigator.cpp)
el valor heredado es de 10 ms, y agotar esa espera provoca `send_goal failed`.

El archivo se pasa por `NavigateToPose.Goal.behavior_tree`. No se editan parámetros,
lanzamientos ni binarios de Nav2. Está preparado para los identificadores actuales
`GridBased` y `FollowPath`. Con `behavior_tree: ''` se utiliza el árbol predeterminado
de Nav2; también se puede indicar otro XML compatible.

El XML debe existir en el equipo que ejecuta Nav2. `auto` funciona al ejecutar
explorador y Nav2 juntos. Para un explorador remoto, configurar `behavior_tree`
con la ruta instalada en el equipo de Nav2.

La finalización describe el espacio alcanzable con esos filtros; no garantiza
descubrir habitaciones cerradas o todas las zonas desconocidas del mundo. El
paquete no guarda el mapa ni añade una vuelta automática al inicio.

## Cierre y supervisión

El ejecutable público es `explore`, usado por el launch. Arranca un proceso
explorador y conserva un supervisor independiente. Antes de enviar cada acción,
el explorador registra su UUID en un archivo temporal privado. Al cerrar con
Ctrl+C/SIGTERM o morir el proceso explorador, el supervisor cancela **ese UUID**
y espera la confirmación de Nav2. No cancela objetivos ajenos.

Si se mata también al supervisor con SIGKILL, se pierde alimentación o se corta
la red con Nav2, este mecanismo no puede garantizar la parada. El watchdog de
velocidades existente no sustituye la cancelación si Nav2 continúa navegando.
Ante un timeout de cancelación se informa del error y se devuelve código de fallo.

## Pruebas

```bash
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 colcon test \
  --packages-select rosbot_exploration --event-handlers console_direct+
colcon test-result --verbose
```

Las pruebas ROS usan servidores de acciones y mapas sintéticos en un dominio
aislado; no envían objetivos a un robot físico. La validación con Gazebo y el
robot real son comprobaciones adicionales.

La prueba manual `test/gazebo_smoke.py` arranca los lanzamientos actuales de
Gazebo sin interfaz gráfica ni cámara, SLAM, Nav2 y el explorador. Requiere tener
compilados `rosbot_description` y `rosbot_exploration`. Comprueba desplazamiento,
varios destinos, crecimiento del mapa y cancelación al cerrar la exploración:

```bash
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 python3 \
  src/rosbot_exploration/test/gazebo_smoke.py
```

Para medir continuidad durante tres minutos y guardar métricas:

```bash
ROS_DOMAIN_ID=181 ROS_LOCALHOST_ONLY=1 python3 \
  src/rosbot_exploration/test/gazebo_smoke.py \
  --duration 180 --report-json /tmp/exploration_metrics.json
```

El informe incluye distancia, mapa conocido, fracción de tiempo en movimiento,
mayor pausa, mediana del tiempo entre destinos, giros acumulados, superficie nueva
por metro/minuto, resultados y errores de Nav2. La fracción en movimiento incluye
rotaciones; por sí sola no mide eficiencia de cobertura.

Las pruebas se ejecutan en el contenedor Foxy con Nav2 0.4.7 y SLAM Toolbox 2.4.1,
sin red y con el repositorio montado en solo lectura. El comportamiento en el
robot físico y la cobertura completa del mundo requieren validación adicional.

### Medición previa de continuidad

Dos ejecuciones consecutivas de 180 s en `willow_garage.world`, con los mismos
lanzamientos y parámetros de SLAM/Nav2, sin interfaz gráfica ni cámara:

| Métrica | Versión inicial | Con anticipación, antes de selección por información |
|---|---:|---:|
| Distancia recorrida antes del cierre | 18,19 m | 35,59 m |
| Fracción de tiempo en movimiento desde el primer objetivo | 54,0 % | 97,7 % |
| Mayor pausa sin movimiento | 54,26 s | 0,40 s |
| Mediana entre llegada y siguiente objetivo | 3,21 s | 0,24 s |
| Objetivos enviados / alcanzados | 11 / 10 | 17 / 13 |
| Celdas conocidas máximas | 120.431 | 144.811 |
| Rechazos de la preplanificación adicional | 42 | 0 (desactivada por defecto) |
| Cancelación y parada al cerrar | Confirmadas | Confirmadas |

Ambas ejecuciones terminaron sin objetivos abortados de navegación; la anterior
registró tres errores de Navfn y la actual no registró errores en `/rosout`.
Las métricas se toman antes de solicitar el cierre; la distancia puede crecer
ligeramente durante el frenado. Son mediciones de esta comparación, no una
garantía para otros mundos, cargas o el robot físico.

La validación previa pasó **26 pruebas**: geometría, conectividad, exclusiones,
cancelación por pérdida de datos, parada con Ctrl+C/SIGHUP, muerte del trabajador,
cierre durante aceptación, preplanificación opcional, anticipación sin preempción
y cambios transitorios/persistentes del destino.

El cálculo de distancias sobre una malla de 719×481 celdas pasó de 1,5492 s a
0,1646 s en una medición aislada, con matrices de salida idénticas. No añade
dependencias nuevas. La optimización sustituye accesos escalares de NumPy dentro
del recorrido por índices enteros y listas de Python.

### Validación de selección por información y huella rectangular

`colcon test` pasa **37 pruebas**, sin errores ni omisiones. Las nuevas pruebas
comprueban oclusión por paredes, unidades métricas de información, penalizaciones
de giro/revisita, huella girada y obstáculos en su interior, un pasillo de 45 cm
en el detector de candidatos, reactivación al retirar obstáculos, agrupación de
fallos por región y estabilidad ante redimensionados/desplazamientos pequeños.

En otra comparación consecutiva de 180 s por versión, en el mismo mundo y sin
interfaz gráfica ni cámara, se obtuvieron estas métricas antes del cierre:

| Métrica | Con anticipación | Con información, memoria y rectángulo |
|---|---:|---:|
| Superficie conocida nueva | 315,01 m² | 452,81 m² |
| Superficie nueva por metro recorrido | 8,37 m²/m | 12,27 m²/m |
| Superficie nueva por minuto desde el primer objetivo | 108,03 m²/min | 153,64 m²/min |
| Distancia recorrida | 37,64 m | 36,89 m |
| Giro acumulado | 29,46 rad | 31,10 rad |
| Giro por 100 m² nuevos | 9,35 rad | 6,87 rad |
| Tiempo en movimiento | 98,4 % | 96,0 % |
| Mayor pausa | 0,30 s | 0,90 s |
| Mediana entre llegada y siguiente objetivo | 0,24 s | 0,24 s |
| Objetivos enviados / alcanzados | 12 / 11 | 24 / 22 |
| Abortos / errores en rosout | 0 / 0 | 0 / 0 |
| Cancelación y parada al cerrar | Confirmadas | Confirmadas |

En esta ejecución mejoró la superficie descubierta por metro y por minuto. El
giro total aumentó ligeramente; el giro relativo a la superficie descubierta se
redujo. Las métricas describen una ejecución por versión y no garantizan el mismo
resultado en otros mundos, con cámara o en el robot físico. La comprobación de
pasos estrechos es geométrica; las maniobras completas dependen de Nav2.
