import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import pytesseract
import re


class SIPReader(Node):
    def __init__(self):
        super().__init__('sip_reader')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(String, '/sip_detectado', 10)
        self.get_logger().info("🟢 Lector de números SIP iniciado.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"❌ Error al convertir la imagen: {e}")
            return

        # Guardar la imagen como archivo PNG
        try:
            image_path = '/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_vision/captured_image.png'
            cv2.imwrite(image_path, frame)
            self.get_logger().info(f"📸 Imagen capturada y guardada en: {image_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Error al guardar la imagen: {e}")
            return

        # Preprocesamiento para mejorar OCR
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

        # OCR con Tesseract
        text = pytesseract.image_to_string(thresh, config='--psm 6')
        self.get_logger().info(f"📄 OCR detectado:\n{text.strip()}")

        # Buscar número SIP (ej. 8-10 dígitos)
        match = re.search(r'\b\d{8,10}\b', text)
        if match:
            sip = match.group(0)
            self.get_logger().info(f"✅ Número SIP detectado: {sip}")
            self.publisher.publish(String(data=sip))
        else:
            self.get_logger().info("⚠️ No se detectó ningún número SIP.")


def main(args=None):
    rclpy.init(args=args)
    node = SIPReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()