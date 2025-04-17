#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient

class FakePathPublisher(Node):
    def __init__(self):
        super().__init__('fake_path_publisher')

        # Create client to FollowPath server
        self.follow_path_client = ActionClient(self, FollowPath, 'follow_path')

        # Try sending path
        self.followPath()




    def createPath(self):
        path_msg = Path()
        now = self.get_clock().now().to_msg()

        path_msg.header.stamp = now
        path_msg.header.frame_id = 'rover/map'

        poses = []
        steps = 120
        for i in range(steps + 1):
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'rover/map'
            pose.pose.position.x = (60.0 / steps) * i
            # pose.pose.position.y = (1.0 / steps) * i
            pose.pose.position.y = 0.0
            # pose.pose.orientation.w = 1.0
            poses.append(pose)

        path_msg.poses = poses

        return path_msg


    def _feedbackCallback(self, msg) -> None:
        self.get_logger().info(f'Received action feedback message\n-> Distance to goal {msg.feedback.distance_to_goal}\n-> Speed {msg.feedback.speed}')
        self.feedback = msg.feedback
        return

    def followPath(self):
        while not self.follow_path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'FollowPath' action server not available, waiting...")

        goal_msg = FollowPath.Goal()
        goal_msg.path = self.createPath()
        goal_msg.controller_id = ""
        goal_msg.goal_checker_id = ""

        self.get_logger().info('Executing path...')
        send_goal_future = self.follow_path_client.send_goal_async(
            goal_msg, self._feedbackCallback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle or not self.goal_handle.accepted:
            msg = 'FollowPath goal was rejected!'
            self.get_logger().error(msg)
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True


def main(args=None):
    rclpy.init(args=args)
    node = FakePathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.context.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
