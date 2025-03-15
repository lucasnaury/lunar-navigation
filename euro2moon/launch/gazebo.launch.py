import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    urdfPath = os.path.join(FindPackageShare('euro2moon').find('euro2moon'), 'urdf', 'rover', 'model.sdf')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('x', default_value='0', description='X position of the robot'),
        DeclareLaunchArgument('y', default_value='0', description='Y position of the robot'),
        DeclareLaunchArgument('z', default_value='1.57', description='Z position of the robot'),
        DeclareLaunchArgument('roll', default_value='0', description='Roll of the robot'),
        DeclareLaunchArgument('pitch', default_value='0', description='Pitch of the robot'),
        DeclareLaunchArgument('yaw', default_value='0', description='Yaw of the robot'),
        DeclareLaunchArgument('robot_name', default_value='rover', description='Robot name'),
        DeclareLaunchArgument('world_file', default_value=LaunchConfiguration('world_file', default=os.path.join(FindPackageShare('euro2moon').find('euro2moon'), 'worlds', 'moon.world')),
                               description='World file for the simulation'),
        DeclareLaunchArgument('robot_description', default_value=os.path.join(FindPackageShare('euro2moon').find('euro2moon'), 'urdf', 'rover.sdf')),
 

        GroupAction([
            # Set gazebo env variable
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH",value = PathJoinSubstitution([FindPackageShare('euro2moon'),"worlds"])),
            

            # Launch gazebo with correct world       
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(FindPackageShare('ros_gz_sim').find('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
                launch_arguments={
                    'gz_args': ["-r ", LaunchConfiguration('world_file'), ' --verbose 4'],
                    'on_exit_shutdown': 'true',
                }.items()
            ),
        ]),


        # Launch gazebo/ROS2 bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            output='screen',
            parameters=[
                {"config_file": os.path.join(FindPackageShare('euro2moon').find('euro2moon'), 'config', 'gazebo_bridge.yaml')}
            ]
        ),

        # Spawn sdf
        Node(package='ros_gz_sim', executable='create',
            arguments=['-name', 'rover',
                # Default spawn point
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-Y', LaunchConfiguration('yaw'),
                '-file', urdfPath],
            output='screen'),

        # robot state publisher node
        # Node(package='robot_state_publisher', executable='robot_state_publisher',
        #     output='screen',
        #     parameters = [
        #         {'ignore_timestamp': False},
        #         {'use_sim_time': True},
        #         {'use_tf_static': True},
        #         {'robot_description': open(urdfPath).read()}],
        #     arguments = [urdfPath]
        # )	


    ])
