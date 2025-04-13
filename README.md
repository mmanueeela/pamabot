# 🤖 Pamabot
# Pamabot: Sistema de Navegación Autónoma con ROS 2 y Nav2
Pamabot es un sistema de navegación autónoma desarrollado en ROS 2 para un robot simulado (tipo TurtleBot3) con capacidad para navegar en entornos personalizados como una farmacia simulada. Este proyecto utiliza el stack de navegación Nav2 e incluye mapas, lanzadores personalizados y configuración avanzada en RViz.


## 🚀 Lanzamiento Rápido
### 1. Requisitos
- ROS 2 Galactic o superior  
- `turtlebot3` instalado  
- `nav2_bringup` y dependencias  
- Gazebo y RViz configurados  

### 2. Compilación
```bash
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
```
### 3. Lanzamiento del entorno simulado
```bash
ros2 launch my_nav2_system my_tb3_sim_nav2.launch.py
```
Esto lanzará
- El mundo de simulación en Gazebo
- El modelo URDF personalizado del robot
- RViz con la configuración `pamabot_world.rviz`
- El stack de navegación Nav2

## 🗺️ Mapas y Configuración
El proyecto incluye mapas en formato `.pgm` y `.yaml`, así como múltiples configuraciones de **RViz** listas para usar:

- `farmaciaMapa.pgm/.yaml`: Mapa del entorno tipo farmacia  
- `pamabot_world.rviz`: Configuración para visualización en RViz  
- Parámetros de navegación ajustados en `my_nav2_params.yaml`  

## 🌲 Behavior Trees
Se utiliza un árbol de comportamiento básico (`simple_nav.xml`) para controlar la navegación del robot de forma modular y escalable.

## 🤖 URDF
El modelo **URDF** del robot se encuentra en la carpeta `urdf/` y ha sido adaptado para funcionar correctamente con los sensores requeridos por **Nav2** (como el **LIDAR** para `/scan`).

## 🌐 Interfaz Web
El sistema también incluye una **página web** que permite controlar el movimiento del robot de forma remota. Esta interfaz facilita la interacción con el robot desde cualquier navegador compatible.

## 📌 Notas
- Asegúrate de tener correctamente configuradas las variables de entorno para `TURTLEBOT3_MODEL`.  
- El archivo **RViz** se puede personalizar para mostrar `/map`, `/scan`, `/tf`, etc.  
- Los lanzadores están preparados para integrarse fácilmente con otros nodos **ROS 2**.

# 🧑‍💻 Autor
Desarrollado por [Manuela Zazzaro, Mari Dapcheva, Pablo Meana, Tan Qianzi]

📧 Contacto: manussupv@gmail.com




