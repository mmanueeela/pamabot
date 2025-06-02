#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.bridge = CvBridge()
        
        # Carga del modelo entrenado
        self.model = YOLO('/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_vision/pamabot_vision/modelos/best.pt')

        # Suscribirse al topic de cámara
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('YOLO detector listo y esperando imágenes...')

    def image_callback(self, msg):
        # Convertir la imagen de ROS a OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detección
        results = self.model(frame)[0]

        # Mostrar detecciones
        annotated_frame = results.plot()
        cv2.imshow("YOLO Detections", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

