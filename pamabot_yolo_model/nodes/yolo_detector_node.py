#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.bridge = CvBridge()

        # Cargar modelo YOLO
        self.model = YOLO('/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_yolo_model/best.pt')
        self.get_logger().info('Modelo YOLO cargado ✅')

        # Suscribirse a la cámara
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)

        # Publicar resultado
        self.publisher_ = self.create_publisher(String, '/yolo_result', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)

        names = self.model.names
        detections = results[0].boxes

        detected_classes = set()
        for box in detections:
            cls_id = int(box.cls[0])
            detected_classes.add(names[cls_id])

        if detected_classes:
            result_str = ', '.join(detected_classes)
            self.publisher_.publish(String(data=result_str))
            self.get_logger().info(f'Detectado: {result_str}')
        else:
            self.get_logger().info('Nada detectado')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
