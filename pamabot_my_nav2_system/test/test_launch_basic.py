# Copyright 2025 PAMABOT
# License: Apache 2.0
import os
import unittest

from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions

from ament_index_python.packages import get_package_share_directory


def generate_test_description():
    nav2_yaml = os.path.join(
        get_package_share_directory('pamabot_my_nav2_system'),
        'config',
        'my_nav2_params.yaml'
    )

    test_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, nav2_yaml]
    )

    return LaunchDescription([
        test_node,
        launch_testing.actions.ReadyToTest()
    ]), {
        'test_node': test_node
    }


class TestMapServerLaunch(unittest.TestCase):
    def test_node_launched(self, proc_info, test_node):
        proc_info.assertWaitForStartup(process=test_node, timeout=10)
