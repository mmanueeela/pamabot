# Copyright 2025 PAMABOT
# License: Apache 2.0
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from builtin_interfaces.msg import Time

class WaypointFollowerClient(Node):
    def __init__(self):
        super().__init__('waypoint_follower_client')
        self.client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.timer = self.create_timer(2.0, self.send_goal_once)

    def send_goal_once(self):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('No action server found for follow_waypoints')
            return

        waypoints = self.create_waypoints()
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info('Sending goal with {} waypoints...'.format(len(waypoints)))
        self.client.send_goal_async(goal_msg)
        self.timer.cancel()

    def create_waypoints(self):
        poses = []
        coords = [
            (4.5, -8.0),  # Posición 1
            (4.5, -5.0),  # Posición 2
            (6.0, -5.0),  # Posición 3
        ]
        for x, y in coords:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            poses.append(pose)
        return poses

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
