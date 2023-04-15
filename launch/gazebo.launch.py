#!/usr/bin/env python3

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_path = os.path.join(get_package_share_directory('new_bcr_robot'), 'urdf', 'new_bcr_robot.xacro')
    doc = get_xacro_to_doc(xacro_path, {"wheel_odom_topic": "odom", "sim_gazebo": "true"})

    robot_state_publisher = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': doc.toxml()}]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'new_bcr_robot'],
        output='screen')

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_force_system.so'],
        output='screen', emulate_tty=True
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_description', default_value=doc.toxml()),
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])