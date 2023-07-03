# New BCR Robot

## About

This repository contains a Gazebo simulation for a differential drive robot, equipped with an IMU, a depth camera and a 2D LiDAR. The primary contriution of this project is to support multiple ROS and Gazebo distros. Currently, the project supports the following versions - 

1. [ROS Noetic + Gazebo Classic 11 (branch ros1)](#noetic--classic-ubuntu-2004)
2. [ROS2 Humble + Gazebo Classic 11 (branch ros2)](#humble--classic-ubuntu-2204)
3. [ROS2 Humble + Gazebo Fortress (branch ros2)](#humble--fortress-ubuntu-2204)

Each of the following sections describes depedencies, build and run instructions for each of the above combinations

## Noetic + Classic (Ubuntu 20.04)

### Dependencies

In addition to ROS1 Noetic and Gazebo Classic installations, the dependencies can be installed with [rosdep](http://wiki.ros.org/rosdep)

```bash
# From the root directory of the workspace. This will install everything mentioned in package.xml
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
catkin build --packages-select bcr_bot
```

### Run

To launch the robot in Gazebo,
```bash
roslaunch bcr_bot gazebo.launch
```
To view in rviz,
```bash
roslaunch bcr_bot rviz.launch
```
### Configuration

The launch file accepts multiple launch arguments,
```bash
roslaunch bcr_bot gazebo.launch 
	camera_enabled:=True
	two_d_lidar_enabled:=True
	position_x:=0.0
	position_y:=0.0
	orientation_yaw:=0.0
	odometry_source:=1
	world_file:=small_warehouse.world
```

## Humble + Classic (Ubuntu 22.04)

### Dependencies

In addition to ROS2 Humble and Gazebo Classic installations, we need to manually install [gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2) (since the same branch supports Classic and Fortress)

```bash
sudo apt-get install ros-humble-gazebo-ros-pkgs
```
Remainder of the dependencies can be installed with [rosdep](http://wiki.ros.org/rosdep)

```bash
# From the root directory of the workspace. This will install everything mentioned in package.xml
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
colcon build --packages-select bcr_bot
```

### Run

To launch the robot in Gazebo,
```bash
ros2 launch bcr_bot gazebo.launch.py
```
To view in rviz,
```bash
ros2 launch bcr_bot rviz.launch.py
```
### Configuration

The launch file accepts multiple launch arguments,
```bash
ros2 launch bcr_bot gazebo.launch.py 
	camera_enabled:=True
	two_d_lidar_enabled:=True
	position_x:=0.0
	position_y:=0.0
	orientation_yaw:=0.0
	odometry_source:=1
	world_file:=small_warehouse.world
```

## Humble + Fortress (Ubuntu 22.04)

### Dependencies

In addition to ROS2 Humble and [Gazebo Fortress installations](https://gazebosim.org/docs/fortress/install_ubuntu), we need to manually install interfaces between ROS2 and Gazebo sim as follows,

```bash
sudo apt-get install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces 
```
Remainder of the dependencies can be installed with [rosdep](http://wiki.ros.org/rosdep)

```bash
# From the root directory of the workspace. This will install everything mentioned in package.xml
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
colcon build --packages-select bcr_bot
```

### Run

To launch the robot in Gazebo,
```bash
ros2 launch bcr_bot gz.launch.py
```
To view in rviz,
```bash
ros2 launch bcr_bot rviz.launch.py
```

### Configuration

The launch file accepts multiple launch arguments,
```bash
ros2 launch bcr_bot gz.launch.py 
	camera_enabled:=True
	two_d_lidar_enabled:=True
	position_x:=0.0
	position_y:=0.0
	orientation_yaw:=0.0
	world_file:=small_warehouse.world
```

### Simulation and Visualization
1. Gz Sim (Ignition Gazebo) (small_warehouse World):
	![](res/gz.jpg)

2. Rviz (Depth camera) (small_warehouse World):
	![](res/rviz.jpg)
