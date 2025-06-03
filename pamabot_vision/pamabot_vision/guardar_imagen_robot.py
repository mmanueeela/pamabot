import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        
        # Ruta para guardar las imágenes en pamabot_vision/images
        self.folder_path = os.path.expanduser('~/turtlebot3_ws/src/pamabot/pamabot_vision/images')
        os.makedirs(self.folder_path, exist_ok=True)
        self.save_path = os.path.join(self.folder_path, 'ultima_imagen.jpg')
        
        # Configurar QoS para recibir imágenes BEST_EFFORT
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Suscripción al topic del TurtleBot
        self.subscription = self.create_subscription(
            Image,
            'image',  # Cambia si usas otro topic
            self.image_callback,
            qos_profile
        )
        
        self.get_logger().info(f"📸 Nodo de guardado iniciado. Guardará en {self.save_path}")

    def image_callback(self, msg):
        try:
            # Convertir el mensaje ROS a imagen OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if cv_image is not None and cv_image.size > 0:
                # Guardar la imagen en la ruta especificada
                success = cv2.imwrite(self.save_path, cv_image)
                if success:
                    self.get_logger().info(f"✅ Imagen guardada en {self.save_path}")
                else:
                    self.get_logger().error(f"❌ Error al guardar imagen en {self.save_path}")
            else:
                self.get_logger().error("❌ Imagen recibida vacía o corrupta. No se guarda.")

        except Exception as e:
            self.get_logger().error(f"❌ Error procesando imagen: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
