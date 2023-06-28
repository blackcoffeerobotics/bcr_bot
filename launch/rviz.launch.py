#!/usr/bin/env python3

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def get_xacro_to_doc(xacro_file_path, mappings):
	doc = xacro.parse(open(xacro_file_path))
	xacro.process_doc(doc, mappings=mappings)
	return doc

def generate_launch_description():

	use_sim_time = LaunchConfiguration('use_sim_time', default='false')

	xacro_path = os.path.join(get_package_share_directory('bcr_bot'), 'urdf', 'bcr_bot.xacro')
	doc = get_xacro_to_doc(xacro_path, {"wheel_odom_topic": "odom"})

	robot_state_publisher = Node(
		package='robot_state_publisher', 
		executable='robot_state_publisher', 
		name='robot_state_publisher',
		output='screen',
		parameters=[{'use_sim_time': use_sim_time}, {'robot_description': doc.toxml()}]
	)

	rviz = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', os.path.join(get_package_share_directory('bcr_bot'), 'rviz', 'entire_setup.rviz')]
	)

	return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot_description', default_value=doc.toxml()),
        robot_state_publisher, rviz
    ])