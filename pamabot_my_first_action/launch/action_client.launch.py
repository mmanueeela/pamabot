from launch import LaunchDescription
from launch_ros.actions import Node

"""
Lanzador para el cliente de acción de pamabot.

Este script lanza el nodo `action_client` del paquete `pamabot_my_first_action`.
"""

def generate_launch_description():
    """
    Genera y retorna una descripción de lanzamiento con el nodo cliente de acción.

    :return: LaunchDescription que lanza el nodo `action_client`.
    """

    return LaunchDescription([
        Node(
            package='pamabot_my_first_action',
            executable='action_client',
            output='screen'
        ),
    ])