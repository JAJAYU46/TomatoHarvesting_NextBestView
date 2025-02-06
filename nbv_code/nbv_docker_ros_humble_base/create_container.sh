#!/bin/bash
xhost local:root
XAUTH=/tmp/.docker.xauth
# # Set the necessary environment variable for X11 and authentication
# export DISPLAY=:0
# export XAUTH=/tmp/.docker.xauth
# touch $XAUTH
# xauth nlist $DISPLAY | sed 's/^..../ffff/' | xauth -f $XAUTH nmerge -


# Function to check if a container exists
container_exists() {
    docker ps -a --format '{{.Names}}' | grep -w "$1" > /dev/null 2>&1
}
# Starting index for the container name
index=3
# Check if container with the name exists
while container_exists "nbv_container${index}_nbv_image_fullinstall1"; do
    # If container exists, increment the index
    index=$((index + 1))
done
# Define the container name with the dynamic index
CONTAINER_NAME="nbv_container${index}_nbv_image_fullinstall1"

# Define container name
# CONTAINER_NAME="nbv_container3_nbv_image_fullinstall1"

# Run the Docker container
docker run --rm -it \
  --runtime=nvidia --gpus all \
  -v /dev/bus/usb:/dev/bus/usb \
  -v /home/rmml02/nbv_JaJaYu/nbv_docker_ros_humble_base/TomatoHarvesting_NextBestView/nbv_code/ros2_ws2/src:/home/ros2_ws2/src \
  --device-cgroup-rule='c 189:* rmw' \
  --name=$CONTAINER_NAME \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --env="XAUTHORITY=$XAUTH" \
  --volume="$XAUTH:$XAUTH" \
  --privileged \
  nbv_image_fullinstall1\
  bash
# nbv_image_fullinstall1
# --net=host \
# <Debug> if docker container can't communicate with host => delete --net=host, because this will force net into STP? instead of UDP, and UDP is neccesary for the connection between host and container
# <QNote> If using --net=host is for container-container communication?? don't know, but host-container communication will work without it
# <QNote> --network=host is for Jetson Docker?? --network=host and --net=host can't exist at the same time, will have some error like host be accessed by two things
# --runtime=nvidia --gpus all \ this is important, or the docker container may not be able to access gpu resources

echo "Container '$CONTAINER_NAME' started successfully."

# xhost local:root
# XAUTH=/tmp/.docker.xauth

# docker run -it \
#     -v /dev/bus/usb:/dev/bus/usb \
#     --device-cgroup-rule='c 189:* rmw' \
#     --name=ros2_nbv_try_container8 \
#     --env="DISPLAY=$DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     --env="XAUTHORITY=$XAUTH" \
#     --volume="$XAUTH:$XAUTH" \
#     --net=host \
#     --privileged \
#     ros2_tb5_dev \
#     bash

# echo "Done."