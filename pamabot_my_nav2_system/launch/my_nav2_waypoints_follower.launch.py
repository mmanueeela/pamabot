# Copyright 2025 PAMABOT
# License: Apache 2.0
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    pkg_nav2 = get_package_share_directory('pamabot_my_nav2_system')
    pkg_world = get_package_share_directory('pamabot_my_world')

    nav2_yaml = os.path.join(pkg_nav2, 'config', 'my_nav2_params.yaml')
    map_file = os.path.join(pkg_nav2, 'config', 'farmaciaMapa.yaml')
    rviz_config = os.path.join(pkg_nav2, 'config', 'pamabot_world.rviz')
    world_launch = os.path.join(pkg_world, 'launch', 'turtlebot3_my_world.launch.py')
    urdf_file = os.path.join(pkg_world, 'urdf', 'turtlebot3_burger.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch)
        ),

        # robot_state_publisher (con robot_description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_desc}
            ]
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'turtlebot3_burger', '-topic', 'robot_description'],
            output='screen'
        ),

        # Nodos de Nav2
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, {'yaml_filename': map_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
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

        # Waypoint Follower ❤️
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'loop_rate': 20,
                'stop_on_failure': False,
                'waypoint_task_executor_plugin': 'wait_at_waypoint',
                'wait_at_waypoint': {
                    'plugin': 'nav2_waypoint_follower::WaitAtWaypoint',
                    'enabled': True,
                    'waypoint_pause_duration': 0
                }
            }]
        ),

        # Lifecycle manager incluyendo waypoint_follower 🧬
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': [
                    'map_server', 
                    'amcl', 
                    'planner_server', 
                    'controller_server', 
                    'recoveries_server', 
                    'bt_navigator',
                    'waypoint_follower'
                ]}
            ]
        ),

        # RViz2 activado 💻 (descomenta si quieres usarlo)
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config],
        #     parameters=[{'use_sim_time': True}],
        #     output='screen'
        # ),
    ])
