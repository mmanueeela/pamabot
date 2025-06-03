import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver_timer')
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.last_saved_time = time.time()
        self.save_interval = 5  # segundos
        self.save_dir = '/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_vision/images'
        
        # Verifica permisos
        if not os.access(self.save_dir, os.W_OK):
            self.get_logger().error(f"No se puede escribir en: {self.save_dir}")
        else:
            self.get_logger().info(f"Guardando imágenes en: {self.save_dir}")

    def listener_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_saved_time >= self.save_interval:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = f'image_{timestamp}.jpg'
                filepath = os.path.join(self.save_dir, filename)
                cv2.imwrite(filepath, cv_image)
                self.get_logger().info(f'Imagen guardada en: {filepath}')
                self.last_saved_time = current_time
            except Exception as e:
                self.get_logger().error(f'Error al convertir o guardar imagen: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
