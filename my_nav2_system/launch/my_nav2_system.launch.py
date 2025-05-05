import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

"""
Módulo de lanzamiento para la localización del robot.

Este archivo lanza los nodos necesarios para realizar la localización usando Nav2,
incluyendo el servidor de mapas, AMCL, lifecycle manager y RViz.
"""

def generate_launch_description():
    """
    Genera y retorna una descripción de lanzamiento que incluye:

    - map_server: para cargar el mapa.
    - amcl: para la localización del robot.
    - lifecycle_manager: para gestionar el ciclo de vida de los nodos.
    - rviz2: para visualización con la configuración personalizada.

    :return: LaunchDescription con todos los nodos necesarios para la localización.
    """

    nav2_yaml = os.path.join(get_package_share_directory('my_nav2_system'), 'config', 'my_nav2_params.yaml')
    map_file = os.path.join(get_package_share_directory('my_nav2_system'), 'config', 'farmaciamapa.yaml')
    rviz_config_dir = os.path.join(get_package_share_directory('my_nav2_system'), 'config', 'pamabot_world.rviz')

    return LaunchDescription([
        Node(
            package = 'nav2_map_server',
            executable = 'map_server',
            name = 'map_server',
            output = 'screen',
            parameters=[nav2_yaml,
                        {'yaml_filename':map_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])