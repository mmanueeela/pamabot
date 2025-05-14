import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class QueueNavigation(Node):
    def __init__(self):
        super().__init__('queue_navigation')

        self.navigation_active = False
        self.received_command = False

        self.subscriber = self.create_subscription(
            Bool,
            '/queue_detected',
            self.queue_callback,
            10
        )

        self.command_sub = self.create_subscription(
            String,
            '/customer_command',
            self.customer_command_callback,
            10
        )

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def queue_callback(self, msg):
        if msg.data:
            self.get_logger().info("Cola detectada, iniciando navegación...")
            if not self.navigation_active:
                self.navigation_active = True
                self.send_navigation_goal()

    def customer_command_callback(self, msg):
        self.get_logger().info(f"[DEBUG] Comando recibido del cliente: {msg.data}")
        self.received_command = True

    def send_navigation_goal(self):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('El servidor de navegación no está disponible.')
            self.navigation_active = False
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # 导航到排队顾客坐标
        goal_msg.pose.pose.position.x = 3.803127
        goal_msg.pose.pose.position.y = 4.834725
        goal_msg.pose.pose.position.z = 0.013221

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.001017
        goal_msg.pose.pose.orientation.w = 0.999999

        self.get_logger().info("Enviando objetivo al área de atención (nueva)...")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("El objetivo fue rechazado.")
            self.navigation_active = False
            return

        self.get_logger().info("Objetivo aceptado, navegando...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        result = future.result().result

        self.get_logger().info("\n\n" + "="*50)
        self.get_logger().info("  💬  Estimado cliente, es un placer atenderle.")
        self.get_logger().info("="*50 + "\n")

        self.navigation_active = False
        self.wait_and_return()

    def wait_and_return(self):
        self.get_logger().info("Esperando 5 segundos por comando del cliente...")
        self.received_command = False

        def check_and_return():
            self.get_logger().info(f"[DEBUG] Estado recibido: {self.received_command}")
            if not self.received_command:
                self.get_logger().info("No se recibió comando. Volviendo a la zona de espera...")
                self.send_return_goal()
            else:
                self.get_logger().info("Se recibió comando. Permanecer en posición.")
            self.wait_timer.cancel()

        self.wait_timer = self.create_timer(5.0, check_and_return)

    def send_return_goal(self):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('El servidor de navegación no está disponible.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # 返回原点（等待区）坐标-Regresar a las coordenadas de origen (zona de espera)
        goal_msg.pose.pose.position.x = 5.256799
        goal_msg.pose.pose.position.y = -8.236029
        goal_msg.pose.pose.position.z = 0.009183

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.001004
        goal_msg.pose.pose.orientation.w = 0.999999

        self.get_logger().info("Volviendo a la zona de espera (nueva)...")
        self.nav_to_pose_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QueueNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
