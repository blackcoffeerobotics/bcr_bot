#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    runtime_mjcf = LaunchConfiguration("runtime_mjcf")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("bcr_bot"), "urdf", "bcr_bot.xacro"]
    )

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    runtime_mjcf_arg = DeclareLaunchArgument(
        "runtime_mjcf",
        default_value="true",
        description="Generate the MuJoCo model from robot_description at runtime",
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    xacro_file,
                    " sim_mujoco:=true",
                    " runtime_mjcf:=",
                    runtime_mjcf,
                    " camera_enabled:=true",
                    " stereo_camera_enabled:=true",
                    " two_d_lidar_enabled:=true",
                    " conveyor_enabled:=false",
                ]
            ),
            value_type=str,
        )
    }
    controller_parameters = PathJoinSubstitution(
        [FindPackageShare("bcr_bot"), "config", "mujoco_controllers.yaml"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    urdf_to_mjcf_converter = Node(
        package="mujoco_ros2_control",
        executable="robot_description_to_mjcf.sh",
        output="both",
        emulate_tty=True,
        arguments=[
            "--add_free_joint",
            "--publish_topic",
            "/mujoco_robot_description",
        ],
        condition=IfCondition(runtime_mjcf),
    )

    mujoco_control = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": use_sim_time},
            ParameterFile(controller_parameters, allow_substs=True),
        ],
        on_exit=Shutdown(),
    )

    point_cloud = Node(
        package="depth_image_proc",
        executable="point_cloud_xyzrgb_node",
        name="kinect_point_cloud",
        output="both",
        parameters=[{"use_sim_time": use_sim_time, "exact_sync": True}],
        remappings=[
            ("rgb/image_rect_color", "/kinect_camera_sensor/color"),
            ("depth_registered/image_rect", "/kinect_camera_sensor/depth"),
            ("points", "/bcr_bot/kinect_camera/points"),
        ],
    )

    twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name="bcr_bot_twist_stamper",
        output="both",
        parameters=[{"use_sim_time": use_sim_time, "frame_id": "base_footprint"}],
        remappings=[
            ("cmd_vel_in", "/bcr_bot/cmd_vel"),
            ("cmd_vel_out", "/bcr_bot/cmd_vel_stamped"),
        ],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--param-file",
            controller_parameters,
        ],
        output="both",
    )

    imu_sensor_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "imu_sensor_broadcaster",
            "--param-file",
            controller_parameters,
            "--controller-ros-args",
            "--ros-args --remap /imu_sensor_broadcaster/imu:=/bcr_bot/imu",
        ],
        output="both",
    )

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--param-file",
            controller_parameters,
            "--controller-ros-args",
            "--ros-args --remap /diff_drive_controller/cmd_vel:=/bcr_bot/cmd_vel_stamped "
            "--remap /diff_drive_controller/odom:=/bcr_bot/odom",
        ],
        output="both",
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            runtime_mjcf_arg,
            robot_state_publisher,
            urdf_to_mjcf_converter,
            mujoco_control,
            point_cloud,
            twist_stamper,
            joint_state_broadcaster,
            imu_sensor_broadcaster,
            diff_drive_controller,
        ]
    )
