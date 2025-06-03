import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PublicadorImagenDetectada(Node):
    def __init__(self):
        super().__init__('publicador_imagen_detectada')
        self.publisher_ = self.create_publisher(Image, '/image_yolo_detected', 10)
        self.bridge = CvBridge()

        # Ruta de la imagen detectada
        self.image_path = '/home/mmanueeelaadmin/Escritorio/prueba.jpg'
        self.timer = self.create_timer(1.0, self.publicar_imagen)

    def publicar_imagen(self):
        # Cargar imagen
        frame = cv2.imread(self.image_path)
        if frame is None:
            self.get_logger().error('❌ No se pudo cargar la imagen.')
            return

        # Convertir y publicar
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('✅ Imagen detectada publicada en /image_yolo_detected')

def main(args=None):
    rclpy.init(args=args)
    nodo = PublicadorImagenDetectada()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
