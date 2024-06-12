#!/usr/bin/python3

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
   
    bcr_bot_path = get_package_share_directory("bcr_bot")
    position_x = LaunchConfiguration("position_x")
    position_y = LaunchConfiguration("position_y")
    orientation_yaw = LaunchConfiguration("orientation_yaw")
    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration("stereo_camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration("two_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source")

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
        parameters=[
                    {'robot_description': Command( \
                    ['xacro ', join(bcr_bot_path, 'urdf/bcr_bot.xacro'),
                    ' camera_enabled:=', camera_enabled,
                    ' stereo_camera_enabled:=', stereo_camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' odometry_source:=', odometry_source,
                    ' sim_ign:=', "true"
                    ])}],
        remappings=[
            ('/joint_states', 'bcr_bot/joint_states'),
        ]
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
            "/stereo_camera/left/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "stereo_camera/right/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "kinect_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "stereo_camera/left/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "stereo_camera/right/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/kinect_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/world/default/model/bcr_bot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model"
        ],
        remappings=[
            ('/world/default/model/bcr_bot/joint_state', 'bcr_bot/joint_states'),
            ('/odom', 'bcr_bot/odom'),
            ('/scan', 'bcr_bot/scan'),
            ('/kinect_camera', 'bcr_bot/kinect_camera'),
            ('/stereo_camera/left/image_raw', 'bcr_bot/stereo_camera/left/image_raw'),
            ('/stereo_camera/right/image_raw', 'bcr_bot/stereo_camera/right/image_raw'),
            ('/imu', 'bcr_bot/imu'),
            ('/cmd_vel', 'bcr_bot/cmd_vel'),
            ('kinect_camera/camera_info', 'bcr_bot/kinect_camera/camera_info'),
            ('stereo_camera/left/camera_info', 'bcr_bot/stereo_camera/left/camera_info'),
            ('stereo_camera/right/camera_info', 'bcr_bot/stereo_camera/right/camera_info'),
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
                    "--child-frame-id", "bcr_bot/base_footprint/kinect_camera"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera_enabled", default_value = camera_enabled),
        DeclareLaunchArgument("stereo_camera_enabled", default_value = stereo_camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled", default_value = two_d_lidar_enabled),
        DeclareLaunchArgument("position_x", default_value="0.0"),
        DeclareLaunchArgument("position_y", default_value="0.0"),
        DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument("odometry_source", default_value="world"),
        robot_state_publisher,
        gz_spawn_entity, transform_publisher, gz_ros2_bridge
    ])
