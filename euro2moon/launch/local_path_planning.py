import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    pkg_path = FindPackageShare('euro2moon').find('euro2moon')

    return LaunchDescription([

        # Launch simulation with robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'simulation.launch.py')),
            launch_arguments={
                'x': '-16.0',
                'y': '16.0',
                'z': '2.0',
                'yaw': '-0.785',

                'world_file': os.path.join(FindPackageShare('euro2moon').find('euro2moon'), 'worlds', 'moon.world')
            }.items()
        ),

        # Launch RViz with the simulation config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', os.path.join(FindPackageShare('euro2moon').find('euro2moon'), 'config', 'local.rviz')],
            respawn=False
        )

        # Local path planning nodes
        # ...
        

    ])
     




