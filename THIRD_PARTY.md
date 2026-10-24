# Componentes y recursos de terceros

Se conservan los avisos originales y las licencias incluidas en las fuentes. Las licencias declaradas por cada paquete se pueden consultar en su `package.xml`; este repositorio no sustituye esas declaraciones por una licencia única.

| Componente | Origen indicado en las fuentes | Licencia declarada/conservada |
| --- | --- | --- |
| `rosbot_ros` (`rosbot_description`) | README original: Husarion `rosbot_description`, rama Foxy; contiene adaptaciones locales | MIT en `package.xml`; conservar avisos de cada archivo |
| `ros_astra_camera` (`astra_camera`) | Driver Orbbec Astra incluido en el workspace | BSD en `package.xml`; conservar avisos originales |
| `rplidar_ros` | Driver RPLIDAR incluido en el workspace | BSD; archivo `src/rplidar_ros/LICENSE` |
| Willow Garage | OSRF `gazebo_models`, revisión `8163eb4b5e7e21985c6591d1c0bfb56468c0093f` | CC BY 3.0; `models/willowgarage/LICENSE` y `UPSTREAM.md` |
| OpenNI2 Orbbec | Descarga opcional durante compilación de Astra desde `ob-tim-liu/OpenNI2`, commit `bbabd58c85d7c8f945f4f679bf73b653bd9df3e7` | Avisos del SDK descargado; no se incluye su compilación en Git |

Las rutas `src/` y `models/` de la tabla se refieren a los paquetes bajo `carpeta_compartida/husarion_ws/`. Docker obtiene ROS, Gazebo y las dependencias de sus repositorios; sus licencias corresponden a los paquetes instalados. `rosbot_pro_bringup` conserva su declaración MIT y las adaptaciones locales mantienen las atribuciones existentes.
