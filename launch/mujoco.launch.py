#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("bcr_bot"), "urdf", "bcr_bot.xacro"]
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    xacro_file,
                    " sim_mujoco:=true",
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

    diff_drive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--param-file",
            controller_parameters,
            "--controller-ros-args",
            "--ros-args --remap /diff_drive_controller/cmd_vel:=/bcr_bot/cmd_vel "
            "--remap /diff_drive_controller/odom:=/bcr_bot/odom",
        ],
        output="both",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            robot_state_publisher,
            mujoco_control,
            joint_state_broadcaster,
            diff_drive_controller,
        ]
    )
