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

# Gazebo Fortress
RUN apt-get install --no-install-recommends -yqqq \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-interfaces \
    ros-$ROS_DISTRO-ros-gz-bridge

# Using shell to use bash commands like 'source'
SHELL ["/bin/bash", "-c"]

# Target workspace for ROS2 packages
ARG WORKSPACE=/root/ros2_ws

# Add target workspace in environment
ENV WORKSPACE=$WORKSPACE

WORKDIR $WORKSPACE