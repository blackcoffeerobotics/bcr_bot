# New BCR Robot

This branch of our repository supports ROS Noetic integration with the classic Gazebo simulator. Now, you can leverage the power of ROS Noetic and the familiar environment of Gazebo Classic to develop and test your applications with our 6-wheeled differential drive robot. This combination allows for seamless integration with the ROS ecosystem and provides a robust simulation environment for your robotics projects.

## Dependencies

```bash
# From the root directory of the workspace. This will install everything mentioned in package.xml
rosdep install --from-paths src --ignore-src -r -y
```

## Build Instructions

* Build the package:

```bash
catkin build --packages-select new_bcr_robot
```

## Run Instructions

* To Launch the robot in gazebo:

```bash
roslaunch new_bcr_robot gazebo.launch
```

* To Launch the robot in RViz:

```bash
roslaunch new_bcr_robot rviz.launch
```

## Configuration

### Xacro 

The xacro (refer to `urdf/new_bcr_robot.xacro`) has loads of configuration options as xacro arguments. To mention any one of the configurations, head over to the desired launch file; `gazebo.launch` or `rviz.launch` and edit the robot description parameter.

Example:
```xml
<param name="robot_description" command="$(find xacro)/xacro $(find new_bcr_robot)/urdf/new_bcr_robot.xacro
	two_d_lidar_enabled:=$(arg two_d_lidar_enabled)
	camera_enabled:=$(arg camera_enabled)
	wheel_odom_topic:=$(arg wheel_odom_topic)
	conveyor_enabled:=$(arg conveyor_enabled)
	robot_namespace:=$(arg robot_namespace)
	" />
```

1. `two_d_lidar_enabled`: If you want to use the 2D Lidar in the simulation.
2. `camera_enabled`: If the camera is to be used, enable this flag to true.
3. `wheel_odom_topic`: The odometry topic to publish wheel odom from diff drive plugin into.
4. `robot_namespace`: The namespace of the robot


### World File

The `worlds` directory has two `.world` files, to pass them to gazebo edit the `launch/gazebo.launch` and modify the arg `world_name`.

* Small Warehouse World (Default):

	![](res/gazebo.jpg)

* Depth camera visualisation :

	![](res/rviz.png)