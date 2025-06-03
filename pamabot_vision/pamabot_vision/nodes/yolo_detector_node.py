from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import rclpy
from rclpy.node import Node

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')
        self.bridge = CvBridge()

        # Carga del modelo
        self.model = YOLO('/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/pamabot_vision/pamabot_vision/modelos/best.pt')

        # Suscripción al topic de cámara
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        # Publicador de la imagen anotada
        self.image_pub = self.create_publisher(Image, '/image_yolo_detected', 10)

        self.get_logger().info('YOLO detector listo y esperando imágenes...')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)[0]
        annotated_frame = results.plot()

        # Mostrar resultado en ventana
        cv2.imshow("YOLO Detections", annotated_frame)
        cv2.waitKey(1)

        # Publicar imagen detectada
        msg_out = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.image_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
