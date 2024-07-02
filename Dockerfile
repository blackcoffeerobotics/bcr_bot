ARG ROS_DISTRO=humble

# Use base image, https://hub.docker.com/_/ros/
FROM ros:$ROS_DISTRO-ros-base

# Prevent console from interacting with the user
# Read more here - https://bobcares.com/blog/debian_frontendnoninteractive-docker
ARG DEBIAN_FRONTEND=noninteractive

# Prevent hash mismatch error for apt-get update, qqq makes the terminal quiet while downloading pkgs
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt-get update -yqqq

# Set folder for RUNTIME_DIR. Only to prevent warnings when running RViz2 and Gz
RUN mkdir tmp/runtime-root && chmod 0700 tmp/runtime-root
ENV XDG_RUNTIME_DIR='/tmp/runtime-root'

# https://stackoverflow.com/questions/51023312/docker-having-issues-installing-apt-utils

# Non Python/ROS Dependencies
# apt-utils: https://stackoverflow.com/questions/51023312/docker-having-issues-installing-apt-utils
RUN apt-get install --no-install-recommends -yqqq \
    apt-utils \
    vim \
    git

# Python Dependencies
RUN apt-get install --no-install-recommends -yqqq \
    python3-pip
        
# Install additional ROS packages
RUN apt-get install --no-install-recommends -yqq \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup

# Using shell to use bash commands like 'source'
SHELL ["/bin/bash", "-c"]

# Target workspace for ROS2 packages
ARG WORKSPACE=/root/ros2_ws

# Add target workspace in environment
ENV WORKSPACE=$WORKSPACE

WORKDIR $WORKSPACE
