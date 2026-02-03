#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import cv2
import os

class TrajectoryPlot(Node):
    def __init__(self):
        super().__init__('rover_path_image')

        # Subscribe to /odom
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Create a blank OpenCV image 
        self.image_size = 700 # pixels
        self.scale = 10  # pixels per meter
        self.origin = (15, self.image_size // 2)
        self.map_image = np.ones((self.image_size, self.image_size, 3), dtype=np.uint8) * 255

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Convert to pixel coordinates
        px = int(self.origin[0] + x * self.scale)
        py = int(self.origin[1] - y * self.scale)  # y-axis inverted for image

        # Draw point on image
        if 0 <= px < self.image_size and 0 <= py < self.image_size:
            cv2.circle(self.map_image, (px, py), radius=2, color=(0, 0, 255), thickness=-1)

        # Show the image
        cv2.imshow("Rover Trajectory", self.map_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        save_path = os.path.expanduser('~/trajectory_plot.png')
        cv2.imwrite(save_path, node.map_image)
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
