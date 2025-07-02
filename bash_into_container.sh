#!/bin/bash
xhost +local:root

docker exec -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" -e "ROS_DOMAIN_ID=0" bcr_arm_container /bin/bash -c "cd /bcr_ws && colcon build && source install/setup.bash && exec bash"

xhost -local:root