import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode
import cv2
import numpy as np


class BarcodeReader(Node):
    def __init__(self):
        super().__init__('barcode_reader')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(String, '/codigo_detectado', 10)
        self.detected_codes = set()
        self.get_logger().info("🟢 Lector de códigos iniciado y esperando imágenes...")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"❌ Error al convertir la imagen: {e}")
            return

        barcodes = decode(frame)

        found_new_code = False

        for barcode in barcodes:
            content = barcode.data.decode('utf-8')
            if content not in self.detected_codes:
                self.detected_codes.add(content)
                self.get_logger().info(f"📋 Código detectado: {content}")

                msg_out = String()
                msg_out.data = content
                self.publisher.publish(msg_out)
                self.get_logger().info("✅ Mensaje publicado en /codigo_detectado")

                # Dibujar rectángulo
                pts = [(p.x, p.y) for p in barcode.polygon]
                cv2.polylines(frame, [np.array(pts, np.int32)], True, (0, 255, 0), 2)
                found_new_code = True

        # Mostrar solo si hubo código nuevo
        if found_new_code:
            cv2.imshow("Lector de código", frame)
            cv2.waitKey(3000)  # Muestra la imagen 3 segundos
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = BarcodeReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

