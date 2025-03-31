import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
# from webots_ros2_driver.webots_launcher import WebotsLauncher
# from webots_ros2_driver.webots_controller import WebotsController



def generate_launch_description():

    pkg_path = FindPackageShare('euro2moon').find('euro2moon')

    return LaunchDescription([

        # Launch simulation with robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'simulation.launch.py')),
            launch_arguments={
                'x': '-10',
                'y': '0',
                'z': '2',
                'yaw': '0',

                'world_file': os.path.join(FindPackageShare('euro2moon').find('euro2moon'), 'worlds', 'plane.world')
            }.items()
        ),

        # Local path planning nodes
        # ...
        

    ])
     




