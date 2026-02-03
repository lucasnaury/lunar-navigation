#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from nav_msgs.msg import Path
from time import sleep

"""
Basic navigation demo to follow a given path after smoothing
"""

def createQuat(x,y,z,w):
    q = Quaternion()
    q.x = float(x)
    q.y = float(y)
    q.z = float(z)
    q.w = float(w)
    return q

def createPath(node:BasicNavigator):
    path_msg = Path()
    now = node.get_clock().now().to_msg()

    path_msg.header.stamp = now
    path_msg.header.frame_id = 'rover/map'

    # Get target point
    use_moon = node.get_parameter("use_moon").get_parameter_value().bool_value
    targetPos = (15.0, -15.0) if use_moon else (60.0, 0.0)
    targetRot = createQuat(0, 0, -0.3824995, 0.9239557) if use_moon else Quaternion()

    poses = []
    steps = 200
    for i in range(steps + 1):
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'rover/map'
        pose.pose.position.x = (targetPos[0] / steps) * i
        pose.pose.position.y = (targetPos[1] / steps) * i
        pose.pose.orientation = targetRot
        poses.append(pose)

    path_msg.poses = poses

    return path_msg

def main() -> None:
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'rover/map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 0.0
    # initial_pose.pose.position.y = 0.0
    # initial_pose.pose.orientation.z = 0.0
    # initial_pose.pose.orientation.w = 1.0
    # navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    # navigator.waitUntilNav2Active()
    # navigator.lifecycleStartup()

    navigator.declare_parameter("use_moon", False)

    while True:

        # Get the path, smooth it
        path = createPath(navigator)
        # smoothed_path = navigator.smoothPath(path)

        # navigator.get_logger().info(str(smoothed_path))

        # Follow path
        # navigator.followPath(smoothed_path)
        navigator.followPath(path)

        i = 0
        while not navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated distance remaining to goal position: '
                    + f'{feedback.distance_to_goal:.3f}'
                    + '\nCurrent speed of the robot: '
                    + f'{feedback.speed:.3f}'
                )

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print("Goal failed")
        else:
            print('Goal has an invalid return status!')


        sleep(1)
        break


if __name__ == '__main__':
    main()