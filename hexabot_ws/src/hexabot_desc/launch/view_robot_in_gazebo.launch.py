import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    pkg_name = LaunchConfiguration('pkg_name', default='hexabot_desc')
    urdf_file_name = LaunchConfiguration('urdf_file_name', default='hexabot_v2.urdf.xacro')

    # Find the package share directory
    pkg_share_path = FindPackageShare(pkg_name)

    # Use PathJoinSubstitution for cleaner and safer path handling
    urdf_path = PathJoinSubstitution([pkg_share_path, 'urdf', urdf_file_name])

    # Get the path to the gazebo launch file
    gazebo_launch_path = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])
    
    # Process the URDF and Xacro files using Command
    robot_description = Command(['xacro ', urdf_path])

    # Launch Gazebo with the empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path)
    )

    # Node to publish the robot model
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'hexabot'],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('pkg_name', default_value='hexabot_desc', description='Name of the package containing the robot description.'),
        DeclareLaunchArgument('urdf_file_name', default_value='hexabot_v2.urdf', description='URDF file name.'),
        gazebo,
        robot_state_publisher_node,
        spawn_entity
    ])