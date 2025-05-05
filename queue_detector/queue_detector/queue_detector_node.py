"""
Módulo queue_detector.py

Este nodo de ROS 2 se encarga de detectar si un modelo específico (por ejemplo, 'bobeye') 
entra dentro de una zona de detección definida en un entorno de simulación de Gazebo. 
Cuando se detecta la presencia en la zona, se publica un mensaje en el topic '/queue_detected'.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
import math

class QueueDetector(Node):
    """
    Nodo que detecta si un modelo específico entra en una zona definida en Gazebo.
    
    Suscribe a '/gazebo/model_states' y publica en '/queue_detected' un valor booleano 
    indicando si el modelo objetivo ('bobeye') se encuentra dentro del radio de detección.
    """
    def __init__(self):
        """
        Constructor del nodo QueueDetector. Inicializa las suscripciones, el publicador y 
        los parámetros de la zona de detección.
        """
        super().__init__('queue_detector')

        self.target_x = 3.905034
        self.target_y = 4.927921
        self.detection_radius = 2.0

        self.last_state = None  # Último estado de detección

        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_callback,
            10
        )

        self.pub_state = self.create_publisher(Bool, '/queue_detected', 10)

    def model_callback(self, msg):
        """
        Callback que procesa los estados de los modelos en Gazebo. Si el modelo 'bobeye' 
        se encuentra dentro del área de detección, publica True; en caso contrario, False.
        
        Se publica solo si el estado cambia respecto al anterior.
        
        :param msg: Mensaje de tipo ModelStates con la información de todos los modelos.
        """
        detected = False

        for i, name in enumerate(msg.name):
            model_x = msg.pose[i].position.x
            model_y = msg.pose[i].position.y

            # Imprimir el nombre y la posición de cada modelo
            self.get_logger().info(f"[DEBUG] Modelo: {name}, Posición: ({model_x:.2f}, {model_y:.2f})")

            # Verificar si el modelo 'bobeye' entra en la zona de detección
            if 'bobeye' in name:
                distance = math.sqrt((model_x - self.target_x)**2 + (model_y - self.target_y)**2)
                self.get_logger().info(f"[DEBUG] → Distancia de 'bobeye' al centro: {distance:.2f} m")
                if distance <= self.detection_radius:
                    self.get_logger().info(f"✅ El modelo '{name}' ha entrado en la zona de detección de la cola.")
                    detected = True
                    break

        # Publicar solo si el estado ha cambiado
        if detected != self.last_state:
            msg_out = Bool()
            msg_out.data = detected
            self.pub_state.publish(msg_out)
            state_str = "True" if detected else "False"
            self.get_logger().info(f"📤 Publicado nuevo estado de detección: {state_str}")
            self.last_state = detected

def main(args=None):
    """
    Función principal que inicia el nodo QueueDetector y mantiene el spin activo.
    """
    rclpy.init(args=args)
    node = QueueDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
