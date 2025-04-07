import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')  # fallback

def generate_launch_description():
    # Paths
    pkg_name = 'pamabot_my_world'
    pkg_share = get_package_share_directory(pkg_name)
    world_path = os.path.join(pkg_share, 'world/planta_farmacia.model')
    urdf_path = os.path.join(pkg_share, f'urdf/turtlebot3_{TURTLEBOT3_MODEL}.urdf')

    # Load URDF file as text
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    # Declare launch arguments for pose
    return LaunchDescription([
        
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_path}.items(),
        ),

        # Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
            ),
        ),

        # Robot state publisher con URDF directamente
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content,
                         'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        
    ])
