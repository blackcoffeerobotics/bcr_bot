ARG ROS_DISTRO=jazzy

# Official ROS 2 Jazzy base image
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

# Runtime directory required by graphical applications such as RViz
RUN mkdir -p /tmp/runtime-root && chmod 0700 /tmp/runtime-root
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# General development tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    apt-utils \
    git \
    vim \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Common ROS dependencies used by bcr_bot
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Enable Bash commands such as source
SHELL ["/bin/bash", "-c"]

# ROS 2 workspace inside the container
ARG WORKSPACE=/root/ros2_ws
ENV WORKSPACE=${WORKSPACE}

WORKDIR ${WORKSPACE}