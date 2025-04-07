import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    pkg_nav2 = get_package_share_directory('my_nav2_system')
    pkg_world = get_package_share_directory('pamabot_my_world')

    nav2_yaml = os.path.join(pkg_nav2, 'config', 'my_nav2_params.yaml')
    map_file = '/home/mmanueeelaadmin/turtlebot3_ws/src/pamabot/my_nav2_system/config/farmaciaMapa.yaml'
    rviz_config = os.path.join(pkg_nav2, 'config', 'pamabot_world.rviz')
    # world_launch = os.path.join(pkg_world, 'launch', 'turtlebot3_my_world.launch.py')
    urdf_file = os.path.join(pkg_world, 'urdf', 'turtlebot3_burger.urdf')

    return LaunchDescription([

        # Gazebo
        # IncludeLaunchDescription(
          #  PythonLaunchDescriptionSource(world_launch)
       # ),

        Node(
            package = 'nav2_map_server',
            executable = 'map_server',
            name = 'map_server',
            output = 'screen',
            parameters=[{'use_sim_time': True}, {'yaml_filename':map_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),


        Node(
            package = 'nav2_planner',
            executable = 'planner_server',
            name = 'planner_server',
            output = 'screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_yaml, {'use_sim_time': True}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names':['map_server', 'amcl', 'planner_server', 'controller_server', 'recoveries_server', 'bt_navigator']}]
        ),

         Node(
               package='rviz2',
               executable='rviz2',
               name='rviz2',
               arguments=['-d', rviz_config],
               parameters=[{'use_sim_time': True}],
               output='screen'
        )
    ])
