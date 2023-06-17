#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    # Retrieve launch configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Path to the Xacro file
    xacro_path = join(get_package_share_directory('new_bcr_robot'), 'urdf', 'new_bcr_robot.xacro')
    doc = get_xacro_to_doc(xacro_path, {"wheel_odom_topic": "odom", "sim_gazebo": "true", "two_d_lidar_enabled": "true", "camera_enabled": "true"})

    # Launch the robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': doc.toxml()}],
        remappings=[('joint_states', '/new_bcr_robot/joint_states')]
    )

    # Launch the spawn_entity node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'new_bcr_robot'],
        output='screen'
    )

    # Include the Gazebo launch file
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py"))
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('world', default_value=[FindPackageShare('new_bcr_robot'), '/worlds/gazebo/small_warehouse.world']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot_description', default_value=doc.toxml()),
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
