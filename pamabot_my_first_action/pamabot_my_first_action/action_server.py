# action_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from pamabot_interface.action import NavigateToPose
from geometry_msgs.msg import Twist
import time

"""
Servidor de acción personalizado para mover el robot con feedback continuo.

Este nodo actúa como servidor de la acción `NavigateToPose`, y controla el movimiento del robot
simulando desplazamiento con publicación a `cmd_vel`. Envía feedback durante la ejecución.
"""

class MyActionServer(Node):
    """
    Nodo que implementa un servidor de acción para controlar el movimiento del robot.
    """

    def __init__(self):
        """
        Inicializa el servidor de acción, el publicador de velocidad (`cmd_vel`) y
        el mensaje Twist usado para enviar comandos de movimiento.
        """
        super().__init__('my_action_server')
        self.action_server = ActionServer(self, NavigateToPose, 'moving_as', self.execute_callback)
        self.cmd = Twist()
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def execute_callback(self, goal_handle):
        """
        Función de ejecución del servidor de acción.

        Simula el movimiento del robot durante un número de segundos especificado en el goal.
        Publica feedback en cada segundo y detiene el robot al finalizar.

        :param goal_handle: Objeto que representa el goal recibido.
        :return: Resultado de tipo NavigateToPose.Result.
        """
        self.get_logger().info('Recibiendo el Goal...')

        feedback_msg = NavigateToPose.Feedback()
        feedback_msg.feedback = "Moviendo el robot a la izquierda..."

        for i in range(1, goal_handle.request.secs):
            self.get_logger().info('Feedback: {}'.format(feedback_msg.feedback))
            goal_handle.publish_feedback(feedback_msg)
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.2
            self.publisher.publish(self.cmd)
            time.sleep(1)

        goal_handle.succeed()

        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher.publish(self.cmd)

        feedback_msg.feedback = "¡Acción finalizada!"
        result = NavigateToPose.Result()
        result.status = feedback_msg.feedback
        return result


def main(args=None):
    rclpy.init(args=args)

    my_action_server = MyActionServer()

    rclpy.spin(my_action_server)

if __name__=='__main__':
    main()