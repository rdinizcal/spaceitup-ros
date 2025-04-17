# spaceros-bt-packages
Collection of packages for running Behavior Trees over Space ROS

## Download & installation
**Step 1:** Create and access the ROS2 workspace folder
```
mkdir ros2_ws
cd ros2_ws
```
**Step 2:** Clone the repository and extract files inside the workspace's `src/` folder
```
git clone https://github.com/gianlucafilippone/spaceros-bt-packages.git
mv spaceros-bt-packages src
```
**Step 3:** Build packages
```
colcon build
. install/setup.bash
```

## Packages list

### mars_rover_nav
Action server for Mars Rover navigation in simulated environment. Contains both a mocked version that just receives action goals and publishes a mocked odometry and a simple action server that leverages the mars rover default odometry data and sends Twist messages into the `/cmd_vel` channel.

#### Usage
Mocked version:

```
ros2 run mars_rover_nav mock_nav
```

Simple action server:
```
ros2 run mars_rover_nav mars_rover_nav
```

Sending goals:
```
ros2 action send_goal /NavigateToGoal nav2_msgs/action/NavigateToPose \
"{pose: {pose: {position: {x: 2.0, y: 3.0}, orientation: {w: 1.0}}}}"

```

### py_trees_bt_runner
Behavior tree executor implemented using [`py_trees`](https://py-trees.readthedocs.io/) and [`py_trees_ros`](https://py-trees-ros.readthedocs.io/). The executor reads BTs in the XML syntax provided by [Groot2](https://www.behaviortree.dev/groot). Trees must be provided as command line arguments or set as defaults within the executor's code.

#### Usage
Using the default tree:
```
ros2 run py_trees_bt_runner bt_runner
```

Using custom tree:
```
ros2 run py_trees_bt_runner bt_runner /full/path/to/tree_file.xml
```

(Example: `ros2 run py_trees_bt_runner bt_runner /home/user/ros2_ws/src/trees/inspection_mission.xml`)

> [!NOTE]
> Only the actions related to the _Inspection Mission_ (`inspection_mission.xml`) are implemented. They require the navigation system up and running.

### trees
Example trees in the [Groot2](https://www.behaviortree.dev/groot) XML syntax.
