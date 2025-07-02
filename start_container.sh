#!/bin/bash

# Start a new BCR ARM container if not already running
if [ ! "$(docker ps -q -f name=bcr_arm_container)" ]; then
    if [ "$(docker ps -aq -f status=exited -f name=bcr_arm_container)" ]; then
        # Container exists but is stopped, remove it
        docker rm bcr_arm_container
    fi
    
    # Enable X11 forwarding
    xhost +local:docker
    
    # Start new container
    docker run -it \
        --name bcr_arm_container \
        --network host \
        -e DISPLAY=$DISPLAY \
        -e QT_X11_NO_MITSHM=1 \
        -e CYCLONEDX_URI=/ddsconfig.xml \
        -e ROS_DOMAIN_ID=0 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev/dri:/dev/dri \
        -v "$(dirname "$0")/../../docker/ddsconfig.xml":/ddsconfig.xml \
        -v "$(dirname "$0")":/bcr_ws/src/bcr_bot \
        --privileged \
        bcr_arm:jazzy-harmonic \
        /bin/bash -c "cd /bcr_ws && rosdep install -i --from-path src --rosdistro jazzy -y && . /opt/ros/jazzy/setup.bash && colcon build && exec bash"
        
    echo "BCR ARM container started."
else
    echo "BCR ARM container is already running."
fi
