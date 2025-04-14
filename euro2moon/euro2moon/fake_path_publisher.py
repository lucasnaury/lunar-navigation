#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class FakePathPublisher(Node):
    def __init__(self):
        super().__init__('fake_path_publisher')
        self.publisher_ = self.create_publisher(Path, '/plan', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_path)  # ✅ start repeating timer
        self.get_logger().info('Fake path publisher started.')

    def publish_path(self):
        path_msg = Path()
        now = self.get_clock().now().to_msg()

        path_msg.header.stamp = now
        path_msg.header.frame_id = 'odom'

        poses = []
        steps = 20
        for i in range(steps + 1):
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'odom'
            pose.pose.position.x = (60.0 / steps) * i
            pose.pose.position.y = (1.0 / steps) * i
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        path_msg.poses = poses

        self.publisher_.publish(path_msg)
        self.get_logger().info(f'Published fake path with {len(poses)} poses.')


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
