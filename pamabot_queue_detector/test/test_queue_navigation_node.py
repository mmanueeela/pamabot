# Copyright 2025 PAMABOT 7

import rclpy
from std_msgs.msg import Bool, String
from pamabot_queue_detector.queue_navigation_node import QueueNavigation

def test_queue_callback_sets_navigation_flag():
    rclpy.init()
    node = QueueNavigation()

    msg = Bool()
    msg.data = True
    node.queue_callback(msg)

    assert node.navigation_active is True

    rclpy.shutdown()

def test_customer_command_callback_sets_flag():
    rclpy.init()
    node = QueueNavigation()

    msg = String()
    msg.data = "Ir al cliente"
    node.customer_command_callback(msg)

    assert node.received_command is True

    rclpy.shutdown()
