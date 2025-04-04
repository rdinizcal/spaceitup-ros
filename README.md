# Space ROS @ Workstation-2
Date: April 4th 2025, 16:26

This canvas intends to document and explain how did I run the newest version of ROS Space in the Workstation-2 with the GPU (NVIDIA RTX 4500)

If you are here to run Curiosity rather than installing it skip the first section and go directly to Running Curiosity in Mars

## Installing the Docker

Clone the newest version of Space-ROS

```
git clone https://github.com/space-ros/docker/
```

Modify the Dockerfile to install the nvidia drivers in the container

Enter the space_robots directory:
```
cd ./space_robots
```

Add the following lines to the Dockerfile :

```
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
# Add NVIDIA dependencies
RUN sudo apt-get update && sudo apt-get install -y \
    nvidia-utils-535 \
    libnvidia-gl-535 \
    libnvidia-extra-535 \
    && sudo rm -rf /var/lib/apt/lists/*
```

Note: I selected 535 since it seems to be the newest and most stable driver for the Nvidia RTX 4500 installed in the Workstation-2.
Modify the run.sh file. Overwrite the docker run ... line to:

```
docker run --rm -it --name $CONTAINER_NAME --network host \
    --gpus all \
    -e DISPLAY \
    -e TERM \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    $IMG_NAME
```

Build and run the dockers:

```
./../moveit2/build.sh
./build.sh
./run.sh
```

As a result you should be inside the docker container:

```
spaceros-user@gssilab-Workstation-2:~/demos_ws$ 
```

## Running Curiosity in Mars

This section expects that you are inside the container. If not, go back to Installing the Docker.
Inside the docker, we must run: (1). the gazebo simulation, (2). the robot's nodes.
Note: For every new terminal open, you need to make sure that the ROS is sourced. Here is how to do it:

```
source /opt/ros/humble/setup.bash
source ~/demos_ws/install/setup.bash
```

### Run Gazebo Simulation

Open a new terminal within the docker. (I've used devcontainers in vscode, you can do it however you want.)

```
ros2 launch curiosity_gazebo curiosity_gazebo.launch.py
```

The Gazebo Simulation should pop-up in the screen. You should not see any errors in the terminal.

### Run Curiosity Mars Rover

Open a new terminal within the docker. (I've used devcontainers in vscode, you can do it however you want.)

```
ros2 launch curiosity_rover_demo mars_rover.launch.py
```

You should not expect anything different in the simulation, here you are just running a layer of nodes that allow you to better control the rover using /move_forward or /move_stop primitives.

### Check Mars Rover

Open a new terminal within the docker. (I've used devcontainers in vscode, you can do it however you want.)

```
# Move forward
ros2 service call /move_forward std_srvs/srv/Empty
# Turn left
ros2 service call /turn_left std_srvs/srv/Empty
# Turn right
ros2 service call /turn_right std_srvs/srv/Empty
# Stop
ros2 service call /move_stop std_srvs/srv/Empty
```

## Troubleshooting

Niente. Good luck!
