# Copyright 2025 PAMABOT

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.node import Node
from pamabot_vision.barcode_reader import BarcodeReader
from cv_bridge import CvBridge
import cv2
import os

def test_barcode_reader_detects_code():
    rclpy.init()
    node = BarcodeReader()
    bridge = CvBridge()

    # Cargar imagen de prueba con código
    test_img_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'pamabot_vision', 'images', 'mi_codigo.png'
    )
    image = cv2.imread(test_img_path)
    assert image is not None, "No se pudo cargar la imagen de prueba."

    # Convertir a mensaje ROS y simular llamada
    msg = bridge.cv2_to_imgmsg(image, encoding='bgr8')
    node.image_callback(msg)

    # El test básico verifica que no haya fallos en la detección
    # Para más robustez, deberías suscribirte a /codigo_detectado o mockear self.publisher
    rclpy.shutdown()
