from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pamabot_my_first_action',
            executable='action_server',
            output='screen'
        ),
    ])