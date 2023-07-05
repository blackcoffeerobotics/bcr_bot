#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    bcr_bot_path = get_package_share_directory("bcr_bot")
    world_file = LaunchConfiguration("world_file", default = join(bcr_bot_path, "worlds", "small_warehouse.sdf"))

    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)

    # robot_description_content = get_xacro_to_doc(
    #     join(bcr_bot_path, "urdf", "bcr_bot.xacro"),
    #     {"sim_gz": "true",
    #      "two_d_lidar_enabled": "true",
    #      "conveyor_enabled": "false",
    #      "camera_enabled": "true"
    #     }
    # ).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"use_sim_time": use_sim_time},
                    {'robot_description': Command( \
                    ['xacro ', join(bcr_bot_path, 'urdf/bcr_bot.xacro'),
                    ' camera_enabled:=', camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' sim_gz:=', "true"
                    ])}],
        remappings=[
            ('/joint_states', 'bcr_bot/joint_states'),
        ]
    )
 
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : world_file
        }.items()
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/robot_description",
            "-name", "bcr_bot",
            "-allow_renaming", "true",
            "-z", "0.28",
            "-x", position_x,
            "-y", position_y,
            "-Y", orientation_yaw
        ]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/kinect_camera@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/kinect_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/world/default/model/bcr_bot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"
        ],
        remappings=[
            ('/world/default/model/bcr_bot/joint_state', 'bcr_bot/joint_states'),
            ('/odom', 'bcr_bot/odom'),
            ('/scan', 'bcr_bot/scan'),
            ('/kinect_camera', 'bcr_bot/kinect_camera'),
            ('/imu', 'bcr_bot/imu'),
            ('/cmd_vel', 'bcr_bot/cmd_vel'),
            ('/camera_info', 'bcr_bot/camera_info'),
            ('/kinect_camera/points', 'bcr_bot/kinect_camera/points'),
        ]
    )

    transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments = ["--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--yaw", "0.0",
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", "kinect_camera",
                    "--child-frame-id", "bcr_bot/base_link/kinect_camera"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time),
        DeclareLaunchArgument("world_file", default_value=world_file),
        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),        
        robot_state_publisher,
        gz_sim, gz_spawn_entity, gz_ros2_bridge,
        transform_publisher
    ])