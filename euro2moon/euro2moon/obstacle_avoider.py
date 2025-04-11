#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
import numpy as np
import math
from threading import Lock
from sensor_msgs_py import point_cloud2
import time

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__(
            'obstacle_avoider',
            parameter_overrides=[],
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # Get parameters passed from launch file
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.base_frame = self.get_parameter('base_frame').value
        self.global_frame = self.get_parameter('global_frame').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.camera_points_topic = self.get_parameter('camera_points_topic').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.obstacle_height_threshold = self.get_parameter('obstacle_height_threshold').value
        self.update_frequency = self.get_parameter('update_frequency').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_z = self.get_parameter('goal_z').value
        self.direct_control = self.get_parameter('direct_control').value

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.lidar_sub = self.create_subscription(PointCloud2, self.lidar_topic, self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(PointCloud2, self.camera_points_topic, self.camera_callback, 10)
        self.costmap_sub = self.create_subscription(OccupancyGrid, self.costmap_topic, self.costmap_callback, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_path = None
        self.lidar_points = None
        self.camera_points = None
        self.current_costmap = None
        self.is_goal_reached = False
        self.is_navigating = False
        self.nav2_initialized = False
        self.path_lock = Lock()
        self.lidar_lock = Lock()
        self.camera_lock = Lock()
        self.costmap_lock = Lock()
        self.navigation_result = None
        self.odom_data = None
        self.init_attempts = 0
        self.max_init_attempts = 10

        self.goal_sent_time = None
        self.nav2_init_timer = self.create_timer(1.0, self.init_nav2)
        self.goal_timer = self.create_timer(5.0, self.check_goal_status)

        self.get_logger().info('Starting obstacle avoider...')
        self.get_logger().info(f'Target goal: x={self.goal_x}, y={self.goal_y}, z={self.goal_z}')

    def odom_callback(self, msg):
        self.odom_data = msg

    def lidar_callback(self, msg):
        with self.lidar_lock:
            self.lidar_points = msg

    def camera_callback(self, msg):
        with self.camera_lock:
            self.camera_points = msg

    def costmap_callback(self, msg):
        with self.costmap_lock:
            self.current_costmap = msg

    def path_callback(self, msg):
        with self.path_lock:
            self.current_path = msg

    def init_nav2(self):
        if self.nav2_initialized:
            self.nav2_init_timer.cancel()
            return

        self.init_attempts += 1
        if self.init_attempts > self.max_init_attempts:
            self.get_logger().warn('Nav2 initialization failed after max attempts')
            self.nav2_init_timer.cancel()
            return

        try:
            client = self.create_client(GetState, 'bt_navigator/get_state')
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for bt_navigator to become available...')
                return

            request = GetState.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.done():
                response = future.result()
                if response.current_state.id == 3:
                    self.get_logger().info('Nav2 is active!')
                else:
                    self.get_logger().info(f'Nav2 not ready. Current state: {response.current_state.id}')
                    return
            else:
                self.get_logger().info('No response from lifecycle node')
                return

            if not self.nav_to_pose_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().info('Waiting for navigate_to_pose server...')
                return

            # self.nav2_initialized = True
            # self.send_goal()
            self.get_logger().info('Waiting for bt_navigator to activate after transition...')
            time.sleep(2.0)  # Wait briefly for activation to complete

            # Check if it's now active
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done() and future.result().current_state.id == 3:
                self.nav2_initialized = True
                self.get_logger().info('bt_navigator is now ACTIVE. Proceeding to send goal...')
                self.send_goal()
            else:
                self.get_logger().warn('bt_navigator still not active. Will retry on next timer tick.')




        except Exception as e:
            self.get_logger().error(f'Nav2 init exception: {e}')

    def send_goal(self):
        if not self.nav2_initialized:
            self.get_logger().warn('Nav2 not initialized')
            return

        if self.is_navigating:
            self.get_logger().info('Already navigating')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = self.goal_z
        goal_msg.pose.pose.orientation.w = 1.0

        self.goal_sent_time = self.get_clock().now()
        self.is_navigating = True

        self.get_logger().info('Sending goal to Nav2...')
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('Goal reached successfully!')
            self.is_goal_reached = True
        else:
            self.get_logger().error(f'Goal failed with status: {status}')

        self.is_navigating = False
        self.goal_sent_time = None

    def check_goal_status(self):
        if not self.nav2_initialized:
            return

        if self.is_goal_reached:
            return

        if not self.is_navigating:
            self.get_logger().info('Resending goal...')
            self.send_goal()
        elif self.goal_sent_time is not None:
            elapsed = (self.get_clock().now() - self.goal_sent_time).nanoseconds / 1e9
            if elapsed > 60.0:
                self.get_logger().warn(f'Navigation timeout after {elapsed:.1f}s')
                self.is_navigating = False
                self.goal_sent_time = None


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down obstacle avoider...')
        cmd_vel = Twist()
        node.cmd_vel_pub.publish(cmd_vel)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    