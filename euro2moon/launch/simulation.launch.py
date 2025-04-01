import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_path = FindPackageShare('euro2moon').find('euro2moon')

    return LaunchDescription([

        # Launch gazebo with map and bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'gazebo.launch.py')),
        ),

        # Launch teleop_keyboard
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            output='screen',
            prefix = 'xterm -e',
        ),

    ])
     




