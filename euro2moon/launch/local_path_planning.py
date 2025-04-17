import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    autostart = LaunchConfiguration('autostart', default='true')
    map_yaml_file = os.path.join(pkg_path, 'maps', 'map.yaml')
    params_file = os.path.join(pkg_path, 'config', 'nav2_params.yaml')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Declare launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 if true'
    )
    
    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start nav2 stack'
    )

    # Define the launch components
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'x': '0.0',
            'y': '0.0',
            'z': '0.0',
            'yaw': '0.0',
            'world_file': os.path.join(pkg_path, 'worlds', 'plane.world')
        }.items()
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'config', 'local.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        respawn=False
    )
    
    # Include Nav2 navigation launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_use_rviz_arg,
        declare_autostart_arg,
        simulation_launch,
        rviz_node,
        # nav2_bringup_launch
    ])