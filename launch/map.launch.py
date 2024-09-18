import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():

    ld = LaunchDescription()

    # Map server
    map_server_config_path = os.path.join(
        get_package_share_directory('bcr_bot'),
        'config',
        'bcr_map.yaml'
    )

    map_server_cmd = Node(
         package='nav2_map_server',
         executable='map_server', 
         name='map_server',
         output='screen', 
         parameters=[{'use_sim_time': True}, 
             {'yaml_filename': map_server_config_path}
            ])
    
    lifecycle_manager = Node(
         package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
     name='lifecycle_manager_mapper', 
 output='screen',
 parameters=[{'use_sim_time': True}, 
             {'autostart': True}, 
             {'node_names': ['map_server']}])
    

    ld.add_action(map_server_cmd)
    ld.add_action(lifecycle_manager)

    return ld