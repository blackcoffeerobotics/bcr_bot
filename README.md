# New BCR Robot

A 6 wheeled differential drive robot with an IMU and a  2D laser scanner.

## Build Instructions

* Build the package:

		colcon build --symlink-install

## Run Instructions

* Launch the robot in gazebo:

		ros2 launch new_bcr_robot gazebo.launch.py

* Launch the robot in RViz:

		ros2 launch new_bcr_robot rviz.launch.py

* Launch the robot in Ignition Gazebo Fortress

		ros2 launch new_bcr_robot gz.launch.py
