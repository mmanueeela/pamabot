import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory


class ImagenPublisher(Node):
    def __init__(self):
        super().__init__('imagen_publisher')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # ✅ Obtener ruta desde el paquete instalado
        pkg_path = get_package_share_directory('pamabot_vision')
        ruta = os.path.join(pkg_path, 'images', 'ultima_imagen.jpg') #variable para cambiar


        if not os.path.exists(ruta):
            self.get_logger().error(f"❌ Imagen no encontrada en: {ruta}")
            self.imagen = None
        else:
            self.imagen = cv2.imread(ruta)
            if self.imagen is None:
                self.get_logger().error("❌ No se pudo leer la imagen con OpenCV.")
            else:
                self.get_logger().info("📤 Imagen cargada y lista para publicar.")


        # Publicar cada segundo
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.imagen is None:
            return

        msg = self.bridge.cv2_to_imgmsg(self.imagen, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info("📸 Imagen publicada en /camera/image_raw")

def main(args=None):
    rclpy.init(args=args)
    node = ImagenPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
