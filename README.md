# 🤖 PAMABOT — Robot inteligente de farmacia con ROS 2 y Deep Learning

**PAMABOT** es un sistema robótico inteligente desarrollado con ROS 2, capaz de asistir en entornos farmacéuticos automatizando tareas como la identificación de pacientes, la lectura de sus recetas médicas y la entrega personalizada de medicamentos mediante navegación autónoma y visión por computador.

> 💊 ¡Imagina un robot que reconoce al paciente, detecta su medicación en tiempo real y se la lleva directamente sin errores!  
> Con PAMABOT, esa visión es ya una realidad.

---

## 🧠 ¿Qué hace PAMABOT?

- **📷 Reconocimiento de cliente (por SIP):**  
  Detecta al cliente mediante lectura de su código SIP (por QR o código de barras) y lo identifica mediante una búsqueda web.

- **💊 Gestión de recetas médicas:**  
  Accede a una base de datos web para asociar automáticamente al paciente con su medicamento recetado.

- **🧠 Reconocimiento del medicamento:**  
  Utiliza Deep Learning (modelo YOLOv8 entrenado) para identificar visualmente el medicamento correcto entre varios.

- **🚗 Entrega del medicamento:**  
  El robot navega de forma autónoma dentro de la farmacia, localiza al cliente y le entrega el medicamento correspondiente.

- **📸 Captura y procesamiento de imágenes en tiempo real:**  
  Se usan cámaras y nodos ROS 2 para capturar y analizar imágenes desde el entorno farmacéutico.

- **🌐 Interfaz Web integrada:**  
  Permite monitorear al robot en vivo, visualizar las cámaras, ver el estado de las entregas, y enviar comandos desde un navegador web.

---

## 🧩 Arquitectura del sistema

- **ROS 2 (Galactic)**: Comunicación y gestión de nodos robóticos.  
- **YOLOv8**: Detección de medicamentos.  
- **OpenCV + pyzbar**: Lectura de QR/SIP del cliente.  
- **Navegación Nav2**: Movimiento autónomo del robot en la farmacia.  
- **Gazebo**: Simulación 3D del entorno.  
- **Interfaz Web (HTML + JS)**: Comunicación visual en tiempo real.  
- **Python / OpenAI ChatGPT**: Soporte para lógica de decisiones y desarrollo asistido.

---

## 📦 Estructura del repositorio

| Paquete | Descripción |
|--------|-------------|
| `pamabot` | Nodo principal integrador del sistema. |
| `pamabot_capture_image` | Captura imágenes del entorno. |
| `pamabot_interface` | Lógica para conectarse con el SIP y obtener datos del cliente. |
| `pamabot_vision` | Reconocimiento de medicamentos mediante YOLO. |
| `pamabot_queue_detector` | Detecta la presencia del cliente en la cola (visión). |
| `pamabot_my_nav2_system` | Configura y lanza el sistema de navegación autónoma. |
| `pamabot_my_first_action` | Define acciones personalizadas del robot. |
| `pamabot_my_world` | Mundo simulado para pruebas con Gazebo. |
| `pamabot_web` | Interfaz web para control y visualización. |
| `web_video_server` | Servidor de streaming de vídeo del robot. |

---

## 👩‍💻 Autor
Proyecto de Robotica 2025 de GTI.
Desarrollado por Manuela Zazzaro, Mari Dapcheva, Pablo Meana y Tan Qianzi. 

## 🚀 Instalación

```bash
# Clona el repositorio
git clone https://github.com/mmanueeela/pamabot.git
cd pamabot

# Instala las dependencias
rosdep install --from-paths src --ignore-src -r -y

# Compila el workspace
colcon build

# Fuente del entorno
source install/setup.bash
