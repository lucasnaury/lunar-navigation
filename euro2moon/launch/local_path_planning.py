import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the package directory
    pkg_path = FindPackageShare('euro2moon').find('euro2moon')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = os.path.join(pkg_path, 'maps', 'map.yaml')
    params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')

    # Define the launch components
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'x': '-10',
            'y': '0',
            'z': '20',
            'yaw': '0',
            'world_file': os.path.join(pkg_path, 'worlds', 'plane.world')
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'config', 'local.rviz')],
        respawn=False
    )
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'),
                                    'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items(),
    )
    
    obstacle_avoider_node = Node(
        package='euro2moon',
        executable='obstacle_avoider',
        name='obstacle_avoider',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'base_frame': 'base_link'},
            {'global_frame': 'map'},
            {'lidar_topic': '/lidar'},
            {'camera_points_topic': '/camera_points'},
            {'costmap_topic': '/local_costmap/costmap'},
            {'cmd_vel_topic': '/cmd_vel'},
            {'path_topic': '/plan'},
            {'safety_distance': 0.5},
            {'max_linear_velocity': 0.5},
            {'max_angular_velocity': 1.0},
            {'goal_tolerance': 0.2},
            {'obstacle_height_threshold': 0.3},
            {'update_frequency': 10.0}
        ]
    )

    return LaunchDescription([
        simulation_launch,
        rviz_node,
        nav2_launch,
        obstacle_avoider_node
    ])