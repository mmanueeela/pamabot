# Copyright 2025 PAMABOT 7

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
from pamabot_queue_detector.queue_detector_node import QueueDetector

def test_queue_detector_logic():
    rclpy.init()
    node = QueueDetector()

    # Simulamos un mensaje de modelo de Gazebo con un bobeye cerca del objetivo
    msg = ModelStates()
    msg.name = ['ground_plane', 'bobeye']
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x = 3.9
    pose.position.y = 4.9
    msg.pose = [Pose(), pose]

    node.model_callback(msg)

    assert node.last_state is True

    rclpy.shutdown()
