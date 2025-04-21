import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction, LogInfo, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    # Get the package directory
    pkg_path = FindPackageShare('euro2moon').find('euro2moon')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = os.path.join(pkg_path, 'config', 'navigation', 'nav2_params.yaml')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_moon = LaunchConfiguration('use_moon', default='false')
    
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

    declare_use_moon_arg = DeclareLaunchArgument(
        'use_moon',
        default_value='false',
        description='Launch procedurally generated moon map if true'
    )
    
    declare_autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start nav2 stack'
    )
    
    # Define the launch components
    plane_simulation_launch = GroupAction(
        condition=UnlessCondition(use_moon),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'simulation.launch.py')),
            launch_arguments={
                'x': '0.0',
                'y': '0.0',
                'z': '0.0',
                'yaw': '0.0',
                'world_file': os.path.join(pkg_path, 'worlds', 'plane.world')
            }.items()
        )]
    )

    moon_simulation_launch = GroupAction(
        condition=IfCondition(use_moon),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'simulation.launch.py')),
            launch_arguments={
                'x': '-16.0',
                'y': '16.0',
                'z': '2.0',
                'yaw': '-0.785',

                'world_file': os.path.join(pkg_path, 'worlds', 'moon.world')
            }.items()
        )]
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
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )

    # Static transform between map and odon
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_map_pub',
        arguments=["--frame-id", "rover/map", "--child-frame-id", "rover/odom"],
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_use_rviz_arg,
        declare_use_moon_arg,
        declare_autostart_arg,

        plane_simulation_launch,
        moon_simulation_launch,

        rviz_node,
        static_tf,

        # launch nav2 5s after gazebo
        RegisterEventHandler(
            OnProcessStart(
                target_action=rviz_node,
                on_start=[
                    LogInfo(msg='\nSTARTING nav2\n'),
                    TimerAction(
                        period=5.0,
                        actions=[navigation_launch],
                    )
                ]
            )
        ),
    ])