#!/usr/bin/env bash

# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   an X server

IMG_NAME=openrobotics/space_robots_demo

# Replace `/` with `_` to comply with docker container naming
# And append `_runtime`
CONTAINER_NAME="$(tr '/' '_' <<< "$IMG_NAME")"

# Start the container
docker run --rm -it --name $CONTAINER_NAME --network host \
    --gpus all \
    -e DISPLAY \
    -e TERM \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    $IMG_NAME