ARG ROS_DISTRO=jazzy

# Official ROS 2 Jazzy base image
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

# Runtime directory required by graphical applications such as RViz
RUN mkdir -p /tmp/runtime-root && chmod 0700 /tmp/runtime-root
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

RUN apt-get update && apt-get install --no-install-recommends -y \
    apt-utils \
    git \
    vim \
    python3-pip \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-interfaces \
    ros-${ROS_DISTRO}-mujoco-ros2-control \
    ros-${ROS_DISTRO}-mujoco-ros2-control-plugins \
    ros-${ROS_DISTRO}-ros2controlcli \
    ros-${ROS_DISTRO}-imu-sensor-broadcaster \
    ros-${ROS_DISTRO}-depth-image-proc \
    ros-${ROS_DISTRO}-twist-stamper \
    ros-${ROS_DISTRO}-fastcdr \
    ros-${ROS_DISTRO}-fastrtps \
    ros-${ROS_DISTRO}-fastrtps-cmake-module \
    ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    ros-${ROS_DISTRO}-rmw-fastrtps-shared-cpp \
    ros-${ROS_DISTRO}-rosidl-dynamic-typesupport-fastrtps \
    ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-c \
    ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# Enable Bash commands such as source
SHELL ["/bin/bash", "-c"]

# ROS 2 workspace inside the container
ARG WORKSPACE=/root/ros2_ws
ENV WORKSPACE=${WORKSPACE}

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

WORKDIR ${WORKSPACE}

# Install dependencies required for URDF to MJCF conversion 
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \ 
    ros2 run mujoco_ros2_control robot_description_to_mjcf.sh --install-only && \ 
    /root/.ros/ros2_control/.venv/bin/python -m pip install pycollada==0.9.2

CMD ["bash"]