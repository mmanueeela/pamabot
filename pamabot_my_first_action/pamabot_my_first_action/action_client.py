#action_client.py
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from pamabot_interface.action import NavigateToPose


"""
Cliente de acción para la acción personalizada `NavigateToPose`.

Este nodo envía un objetivo (goal) al servidor de acciones `moving_as`, espera la respuesta,
recibe feedback durante la ejecución y finalmente muestra el resultado.
"""

class MyActionClient(Node):
    """
    Nodo que actúa como cliente de la acción `NavigateToPose`.
    """

    def __init__(self):
        """
        Inicializa el cliente de acción con el nombre `moving_as` y el tipo `NavigateToPose`.
        """
        super().__init__('my_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'moving_as')

    def send_goal(self, secs):
        """
        Envía un objetivo al servidor con el tiempo indicado en segundos.

        :param secs: Valor que se asigna al campo `secs` del mensaje de goal.
        """
        goal_msg = NavigateToPose.Goal()
        goal_msg.secs = secs

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback ejecutado cuando el servidor responde a la solicitud del goal.

        :param future: Resultado futuro del envío del goal.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Callback que maneja el resultado final de la acción.

        :param future: Resultado futuro del procesamiento del goal.
        """
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.status))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        """
        Callback que maneja los mensajes de feedback enviados por el servidor de acción.

        :param feedback_msg: Mensaje recibido con información parcial del progreso.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback))
def main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()

    future = action_client.send_goal(5) # se para secs como argumento

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
