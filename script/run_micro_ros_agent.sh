#!/bin/bash


# check if ROS_DISTRO environment variable is set
if [ -z "$ROS_DISTRO" ]; then
    echo "Please set the ROS_DISTRO environment variable (e.g., export ROS_DISTRO=humble)"
    exit 1
fi

# Set the Docker image name for Micro-ROS Agent
IMAGE_NAME="microros/micro-ros-agent:$ROS_DISTRO"


# Set the Serial Port, default is /dev/ttyUSB0
# if you are using a different port, please change it here
SERIAL_PORT="/dev/ttyUSB0"
if [ ! -e $SERIAL_PORT ]; then
    echo "Cannot find $SERIAL_PORT, please check if the device is connected correctly."
    exit 1
fi


# Running the Docker container with the specified image and serial port
# -v /dev:/dev: Mount the host's /dev directory to the container's /dev directory
echo "Starting micro-ROS agent with Serial Port: $SERIAL_PORT"
docker run -it --rm \
    -v /dev:/dev \
    -v /dev/shm:/dev/shm \
    --privileged \
    --net=host \
    $IMAGE_NAME serial --dev $SERIAL_PORT -v4
