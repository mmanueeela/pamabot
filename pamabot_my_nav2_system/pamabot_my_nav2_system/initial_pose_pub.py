# Copyright 2025 PAMABOT
# License: Apache 2.0
# initial_pose_pub.py
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class Publisher(Node):

    def __init__(self):
        super().__init__('initial_pose_pub_node')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 1)
        timer_period = 0.5  # seconds
        self.i = 0.0
        self.timer_ = self.create_timer(timer_period, self.callback)

    def callback(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 4.5
        msg.pose.pose.position.y = -8.0
        msg.pose.pose.orientation.w = 0.0
        self.get_logger().info('Publishing  Initial Position  \n X= 4.5 \n Y= -8.0 \n W = 0.0 ')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    publisher = Publisher()
    try:
        rclpy.spin_once(publisher)
    except KeyboardInterrupt:
        publisher.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()