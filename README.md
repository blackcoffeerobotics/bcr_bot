# New BCR Robot

A 6 wheeled differential drive robot with an IMU and a  2D laser scanner.

## Build Instructions

* Install the dependencies:

	  rosdep install --from-paths src --ignore-src -r -y

* Build the package:

	  catkin build new_bcr_robot

## Run Instructions

* Launch the robot in an gazebo:

	  roslaunch new_bcr_robot gazebo.launch

* Launch the robot in RViz:

	  roslaunch new_bcr_robot rviz.launch
