import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class AutoWaypointNode(Node):

    def __init__(self):
        super().__init__('auto_waypoint_node')

        self.client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.waypoints = [
            (0.0, 0.0, 0.0)
        ]

        self.idx = 0
        self.timer = self.create_timer(1.0, self.start)

    def start(self):
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for Nav2 action server...')
            return

        self.timer.cancel()
        self.send_next_goal()

    def send_next_goal(self):
        if self.idx >= len(self.waypoints):
            self.get_logger().info('All waypoints completed')
            return

        x, y, yaw = self.waypoints[self.idx]

        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(x, y, yaw)

        self.get_logger().info(f'Go to waypoint {self.idx}: {x}, {y}')

        send_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb
        )
        send_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'Waypoint {self.idx} reached')

        self.idx += 1
        self.send_next_goal()

    def feedback_cb(self, feedback_msg):
        pass

    def make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y

        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose


def main():
    rclpy.init()
    node = AutoWaypointNode()
    rclpy.spin(node)
    rclpy.shutdown()
