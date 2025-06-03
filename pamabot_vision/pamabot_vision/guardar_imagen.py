import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class GuardarImagen(Node):
    def __init__(self):
        super().__init__('guardar_imagen')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("🟢 Nodo para guardar imágenes iniciado.")

    def image_callback(self, msg):
        try:
            # Convertir el mensaje de ROS a una imagen de OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info("✅ Imagen recibida y convertida correctamente.")

            # Guardar la imagen como archivo PNG
            image_path = '/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_vision/captured_image.png'
            cv2.imwrite(image_path, frame)
            self.get_logger().info(f"📸 Imagen guardada en: {image_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Error al procesar la imagen: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GuardarImagen()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()