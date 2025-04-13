import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
import math

class QueueDetector(Node):
    def __init__(self):
        super().__init__('queue_detector')

        self.target_x = 3.905034
        self.target_y = 4.927921
        self.detection_radius = 2.0

        self.last_state = None  # 上一次的检测状态

        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_callback,
            10
        )

        self.pub_state = self.create_publisher(Bool, '/queue_detected', 10)

    def model_callback(self, msg):
        detected = False

        for i, name in enumerate(msg.name):
            model_x = msg.pose[i].position.x
            model_y = msg.pose[i].position.y

            # 打印每个模型名称与位置
            self.get_logger().info(f"[DEBUG] 模型: {name}，位置: ({model_x:.2f}, {model_y:.2f})")

            # 检测 bobeye 模型是否进入区域
            if 'bobeye' in name:
                distance = math.sqrt((model_x - self.target_x)**2 + (model_y - self.target_y)**2)
                self.get_logger().info(f"[DEBUG] → bobeye 距离中心: {distance:.2f} m")
                if distance <= self.detection_radius:
                    self.get_logger().info(f"✅ 模型 '{name}' 进入排队检测区域！")
                    detected = True
                    break

        # 仅当状态变化时发布
        if detected != self.last_state:
            msg_out = Bool()
            msg_out.data = detected
            self.pub_state.publish(msg_out)
            state_str = "True" if detected else "False"
            self.get_logger().info(f"📤 发布新检测状态: {state_str}")
            self.last_state = detected

def main(args=None):
    rclpy.init(args=args)
    node = QueueDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
