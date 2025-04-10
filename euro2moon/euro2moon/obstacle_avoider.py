#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import math
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from threading import Lock
from sensor_msgs_py import point_cloud2
import struct

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Declare and get parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('lidar_topic', '/lidar')
        self.declare_parameter('camera_points_topic', '/camera_points')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('safety_distance', 0.05)  # meters
        self.declare_parameter('max_linear_velocity', 0.5)  # m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s
        self.declare_parameter('goal_tolerance', 0.2)  # meters
        self.declare_parameter('obstacle_height_threshold', 0.3)  # meters
        self.declare_parameter('update_frequency', 10.0)  # Hz
        
        # Get parameters
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
        
        # Initialize variables
        self.current_path = None
        self.lidar_points = None
        self.camera_points = None
        self.current_costmap = None
        self.is_goal_reached = False
        self.path_lock = Lock()
        self.lidar_lock = Lock()
        self.camera_lock = Lock()
        self.costmap_lock = Lock()
        
        # Set up QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            sensor_qos
        )
        
        self.camera_sub = self.create_subscription(
            PointCloud2,
            self.camera_points_topic,
            self.camera_callback,
            sensor_qos
        )
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self.costmap_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )
        
        # Setup TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Create a timer for obstacle avoidance
        self.timer = self.create_timer(1.0 / self.update_frequency, self.obstacle_avoidance_callback)
        
        self.get_logger().info('Obstacle Avoider node initialized')
    
    def lidar_callback(self, msg):
        """Callback for LiDAR pointcloud data"""
        with self.lidar_lock:
            self.lidar_points = msg
    
    def camera_callback(self, msg):
        """Callback for RGBD camera pointcloud data"""
        with self.camera_lock:
            self.camera_points = msg
    
    def costmap_callback(self, msg):
        """Callback for costmap data"""
        with self.costmap_lock:
            self.current_costmap = msg
    
    def path_callback(self, msg):
        """Callback for path data"""
        with self.path_lock:
            self.current_path = msg
            self.is_goal_reached = False
    
    def get_next_waypoint(self):
        """Get the next waypoint from the current path"""
        if self.current_path is None or len(self.current_path.poses) == 0:
            return None
        
        try:
            # Get the transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.global_frame,
                rclpy.time.Time()
            )
            
            # Find the closest point on the path that is ahead of the robot
            min_dist = float('inf')
            next_waypoint = None
            
            for pose in self.current_path.poses:
                # Transform the pose from map frame to base_link frame
                x = pose.pose.position.x - transform.transform.translation.x
                y = pose.pose.position.y - transform.transform.translation.y
                
                # Only consider points ahead of the robot (positive x)
                if x > 0:
                    dist = math.sqrt(x * x + y * y)
                    if dist < min_dist:
                        min_dist = dist
                        next_waypoint = pose
            
            # If no point is ahead, take the last point
            if next_waypoint is None and len(self.current_path.poses) > 0:
                next_waypoint = self.current_path.poses[-1]
            
            return next_waypoint
        
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'Could not transform from {self.global_frame} to {self.base_frame}: {ex}')
            return None
    
    def check_lidar_obstacles(self):
        """Check for obstacles using LiDAR pointcloud data"""
        if self.lidar_points is None:
            return False, 0.0
            
        with self.lidar_lock:
            try:
                # Extract points from the PointCloud2 message
                pc_data = list(point_cloud2.read_points(
                    self.lidar_points, 
                    field_names=("x", "y", "z"),
                    skip_nans=True
                ))
                
                if not pc_data:
                    return False, 0.0
                    
                # Convert to numpy array for faster processing
                points = np.array(pc_data)
                
                # Only consider points in front of the robot and within safety distance
                points_in_range = points[
                    (points[:, 0] > 0) &                          # In front of robot
                    (points[:, 0] < self.safety_distance) &       # Within forward safety distance
                    (np.abs(points[:, 1]) < self.safety_distance) # Within lateral safety distance
                ]
                
                if len(points_in_range) > 5:  # More than 5 points to be considered an obstacle
                    min_distance = np.min(np.sqrt(points_in_range[:, 0]**2 + points_in_range[:, 1]**2))
                    self.get_logger().debug(f'LiDAR obstacle detected at distance: {min_distance:.2f}m')
                    return True, min_distance
                
                return False, 0.0
                
            except Exception as e:
                self.get_logger().error(f'Error processing LiDAR data: {e}')
                return False, 0.0
    
    def check_camera_obstacles(self):
        """Check for obstacles using camera pointcloud data"""
        if self.camera_points is None:
            return False, 0.0
            
        with self.camera_lock:
            try:
                # Extract points from the PointCloud2 message
                pc_data = list(point_cloud2.read_points(
                    self.camera_points, 
                    field_names=("x", "y", "z"),
                    skip_nans=True
                ))
                
                if not pc_data:
                    return False, 0.0
                    
                # Convert to numpy array for faster processing
                points = np.array(pc_data)
                
                # Filter points by height to separate ground from obstacles
                # Only consider points above a minimum height but below a maximum height
                points_filtered = points[
                    (points[:, 2] > 0.05) &                       # Above ground
                    (points[:, 2] < self.obstacle_height_threshold) # Below max height
                ]
                
                # Check for obstacles in front of the robot
                points_in_range = points_filtered[
                    (points_filtered[:, 0] > 0) &                          # In front of robot
                    (points_filtered[:, 0] < self.safety_distance) &       # Within forward safety distance
                    (np.abs(points_filtered[:, 1]) < self.safety_distance) # Within lateral safety distance
                ]
                
                if len(points_in_range) > 5:  # More than 5 points to be considered an obstacle
                    min_distance = np.min(np.sqrt(points_in_range[:, 0]**2 + points_in_range[:, 1]**2))
                    self.get_logger().debug(f'Camera obstacle detected at distance: {min_distance:.2f}m')
                    return True, min_distance
                
                return False, 0.0
                
            except Exception as e:
                self.get_logger().error(f'Error processing camera data: {e}')
                return False, 0.0
    
    def check_costmap_obstacles(self):
        """Check for obstacles using the costmap"""
        if self.current_costmap is None:
            return False, 0.0
        
        with self.costmap_lock:
            try:
                # Extract costmap data
                width = self.current_costmap.info.width
                height = self.current_costmap.info.height
                resolution = self.current_costmap.info.resolution
                origin_x = self.current_costmap.info.origin.position.x
                origin_y = self.current_costmap.info.origin.position.y
                
                # Try to get the robot's position in the costmap
                transform = self.tf_buffer.lookup_transform(
                    self.current_costmap.header.frame_id,
                    self.base_frame,
                    rclpy.time.Time()
                )
                
                robot_x = transform.transform.translation.x
                robot_y = transform.transform.translation.y
                
                # Convert to grid coordinates
                grid_x = int((robot_x - origin_x) / resolution)
                grid_y = int((robot_y - origin_y) / resolution)
                
                # Check a rectangular area in front of the robot
                safe_distance_cells = int(self.safety_distance / resolution)
                min_distance = float('inf')
                has_obstacle = False
                
                for i in range(grid_x, min(grid_x + safe_distance_cells, width)):
                    for j in range(grid_y - safe_distance_cells // 2, min(grid_y + safe_distance_cells // 2, height)):
                        if 0 <= i < width and 0 <= j < height:
                            index = j * width + i
                            if index < len(self.current_costmap.data):
                                cost = self.current_costmap.data[index]
                                # Check if the cost indicates an obstacle (typically cost > 50)
                                if cost > 50:
                                    # Calculate distance to this obstacle
                                    cell_x = origin_x + (i + 0.5) * resolution
                                    cell_y = origin_y + (j + 0.5) * resolution
                                    dist = math.sqrt((cell_x - robot_x)**2 + (cell_y - robot_y)**2)
                                    min_distance = min(min_distance, dist)
                                    has_obstacle = True
                
                if has_obstacle:
                    self.get_logger().debug(f'Costmap obstacle detected at distance: {min_distance:.2f}m')
                    return True, min_distance
                
                return False, 0.0
                
            except tf2_ros.TransformException as ex:
                self.get_logger().error(f'Could not transform from {self.current_costmap.header.frame_id} to {self.base_frame}: {ex}')
                return False, 0.0
            except Exception as e:
                self.get_logger().error(f'Error checking costmap: {e}')
                return False, 0.0
    
    def check_goal_reached(self, goal_pose):
        """Check if the robot has reached the goal"""
        if goal_pose is None:
            return False
        
        try:
            # Get the transform from global frame to base_link
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            
            # Calculate distance to goal
            dx = goal_pose.pose.position.x - transform.transform.translation.x
            dy = goal_pose.pose.position.y - transform.transform.translation.y
            distance = math.sqrt(dx * dx + dy * dy)
            
            return distance < self.goal_tolerance
            
        except tf2_ros.TransformException as ex:
            self.get_logger().error(f'Could not transform from {self.global_frame} to {self.base_frame}: {ex}')
            return False
    
    def calculate_avoidance_velocity(self, obstacle_distance):
        """Calculate velocity command for obstacle avoidance"""
        if obstacle_distance <= 0.0:
            return 0.0, 0.0  # Stop if obstacle is too close
        
        # Scale velocity based on obstacle distance
        linear_velocity = min(self.max_linear_velocity, 
                             self.max_linear_velocity * (obstacle_distance / self.safety_distance))
        
        # Always slow down when obstacles are detected
        linear_velocity = max(0.0, min(linear_velocity, 0.2))
        
        # Encourage turning away from obstacles by setting angular velocity
        angular_velocity = self.max_angular_velocity * 0.5
        
        return linear_velocity, angular_velocity
    
    def obstacle_avoidance_callback(self):
        """Main callback for obstacle avoidance"""
        # Check if we have a path
        if self.current_path is None:
            self.get_logger().debug('No path available')
            self.stop_robot()
            return
        
        # Get the next waypoint
        next_waypoint = self.get_next_waypoint()
        if next_waypoint is None:
            self.get_logger().debug('No valid waypoint found')
            self.stop_robot()
            return
        
        # Check if we've reached the final goal
        if self.check_goal_reached(self.current_path.poses[-1]):
            self.get_logger().info('Goal reached!')
            self.stop_robot()
            self.is_goal_reached = True
            return
        
        # Check for obstacles from multiple sources
        lidar_obstacle, lidar_distance = self.check_lidar_obstacles()
        camera_obstacle, camera_distance = self.check_camera_obstacles()
        costmap_obstacle, costmap_distance = self.check_costmap_obstacles()
        
        # Determine the overall obstacle status and closest obstacle
        has_obstacle = lidar_obstacle or camera_obstacle or costmap_obstacle
        
        if has_obstacle:
            # Find minimum distance to nearest obstacle
            distances = []
            if lidar_obstacle:
                distances.append(lidar_distance)
            if camera_obstacle:
                distances.append(camera_distance)
            if costmap_obstacle:
                distances.append(costmap_distance)
                
            min_distance = min(distances) if distances else self.safety_distance
            
            # Log obstacle detection sources
            obstacles_found = []
            if lidar_obstacle:
                obstacles_found.append("LiDAR")
            if camera_obstacle:
                obstacles_found.append("Camera")
            if costmap_obstacle:
                obstacles_found.append("Costmap")
                
            self.get_logger().info(f'Obstacles detected by: {", ".join(obstacles_found)} at {min_distance:.2f}m')
            
            # Calculate avoidance velocity
            linear_vel, angular_vel = self.calculate_avoidance_velocity(min_distance)
            
            # Publish velocity command for avoidance
            # Note: In a real system with Nav2, you might want to let Nav2 handle this
            # and only intervene if absolutely necessary
            cmd_vel = Twist()
            cmd_vel.linear.x = linear_vel
            cmd_vel.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd_vel)
            
            self.get_logger().debug(f'Avoiding obstacle: lin_vel={linear_vel:.2f}, ang_vel={angular_vel:.2f}')
        else:
            # No obstacles, let Nav2 handle normal navigation
            self.get_logger().debug('No obstacles detected, proceeding with Nav2 path')
    
    def stop_robot(self):
        """Stop the robot by publishing zero velocity"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()