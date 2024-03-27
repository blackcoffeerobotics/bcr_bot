#!/usr/bin/env python3

from os.path import join


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.actions import AppendEnvironmentVariable

def generate_launch_description():
    # Get bcr_bot package's share directory path
    bcr_bot_path = get_package_share_directory('bcr_bot')

    # Retrieve launch configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    
    world_file = LaunchConfiguration("world_file", default = join(bcr_bot_path, 'worlds', 'small_warehouse.sdf'))
    
    # Include the Gazebo launch file
    gazebo_share = get_package_share_directory("gazebo_ros")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py"))
    )

    # spawing bcr_bot
    spawn_bcr_bot_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(bcr_bot_path, "launch", "bcr_bot_gazebo_spawn.launch.py")),
    )

    return LaunchDescription([
        # Declare launch arguments
        
        AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=join(bcr_bot_path, "models")),

        SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value="/usr/share/gazebo-11:" + join(bcr_bot_path, "worlds")),
        DeclareLaunchArgument('world', default_value = world_file),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value = use_sim_time),
        gazebo,spawn_bcr_bot_node
    ])
