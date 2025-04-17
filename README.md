# Space ROS @ Workstation-2

Created on: April 4th 2025, 16:26

Last Updated on: April 17th 2025, 17:24 (Ricardo D. Caldas)

This canvas intends to document and explain how did I run the newest version of ROS Space in the Workstation-2 with the GPU (NVIDIA RTX 4500)

If you are here to run Curiosity rather than installing it skip the first section and go directly to Running Curiosity in Mars

## Installing the Docker

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
Inside the docker, we must run: (1). the gazebo simulation, (2). the robot's nodes, (3). run the navigation stack, (4). run them mission using a behavior tree.
Note: For every new terminal open, you need to make sure that the ROS is sourced. Here is how to do it:

```
source /opt/ros/humble/setup.bash
source /opt/spaceros/install/setup.bash
source ~/demos_ws/install/setup.bash
```

### Run Gazebo Simulation

Open a new terminal within the docker. (I've used devcontainers in vscode, you can do it however you want.)

```
ros2 launch curiosity_gazebo mars_sample_return.launch.py
```

The Gazebo Simulation should pop-up in the screen. You should not see any errors in the terminal.

### Run Curiosity Mars Rover

Open a new terminal within the docker. (I've used devcontainers in vscode, you can do it however you want.)

```
ros2 launch curiosity_rover_demo mars_rover.launch.py
```

You should not expect anything different in the simulation, here you are just running a layer of nodes that allow you to better control the rover using /move_forward or /move_stop primitives.

#### Manually Checking Mars Rover

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

### Run Navigation Stack

Open a new terminal within the docker. (I've used devcontainers in vscode, you can do it however you want.)

```
ros2 run mars_rover_nav mars_rover_nav
```

### Run Mission Using Behavior Trees

Open a new terminal within the docker. (I've used devcontainers in vscode, you can do it however you want.)

```
ros2 run py_trees_bt_runner bt_runner /home/spaceros-user/demos_ws/src/mission/trees/inspection_mission.xml
```
## Troubleshooting

### ❗ Error: RMW Implementation Mismatch

When running `colcon build`, you may encounter the following error:

```bash
CMake Error at /opt/spaceros/install/rmw_implementation/share/rmw_implementation/cmake/rmw_implementation-extras.cmake:25 (message): The RMW implementation has been specified as 'rmw_fastrtps_cpp' via get_default_rmw_implementation, but rmw_implementation was built only with support for 'rmw_cyclonedds_cpp'.
```

#### ✅ Fix

This error occurs when the default RMW (ROS Middleware) implementation is set to `rmw_fastrtps_cpp`, but the current Space ROS installation only supports `rmw_cyclonedds_cpp`.

To fix this, set the `RMW_IMPLEMENTATION` environment variable to match the installed middleware:

```bash
source /opt/spaceros/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
colcon build
```
