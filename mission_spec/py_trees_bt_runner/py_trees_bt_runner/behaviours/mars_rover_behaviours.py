import py_trees
import py_trees.behaviour
from py_trees_ros.subscribers import ToBlackboard
from py_trees_ros.action_clients import FromConstant

from  nav2_msgs.action import NavigateToPose
from  nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import math

## Actions ##
class NavigateToGoal(FromConstant):
    def __init__(self, name, x, y):
        self.x = float(x)
        self.y = float(y)
        pose = PoseStamped()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        super().__init__(name=name, action_type=NavigateToPose, action_name="/NavigateToGoal", action_goal=goal_msg)

        self.blackboard_client = self.attach_blackboard_client(name="NavigateToGoalClient")
        self.blackboard_client.register_key("last_reached_location/x", py_trees.common.Access.WRITE)
        self.blackboard_client.register_key("last_reached_location/y", py_trees.common.Access.WRITE)

    def update(self):
        status = super().update()

        if status == py_trees.common.Status.SUCCESS:
            self.logger.info(f"[{self.name}] Action completed, updating robot location on the blackboard")
            self.blackboard_client.last_reached_location.x = self.x
            self.blackboard_client.last_reached_location.y = self.y
            print(self.blackboard_client)

        return status

class TakePicture(py_trees.behaviour.Behaviour):
    def __init__(self, name="TakePicture", picture_id="0"):
        super().__init__(name)
        self.picture_id = int(picture_id)
        self.blackboard = py_trees.blackboard.Client(name="TakePictureClient")
        self.blackboard_key = f"picture_{self.picture_id}"
        self.blackboard.register_key(self.blackboard_key, py_trees.common.Access.WRITE)

    def update(self):
        self.logger.info(f"Taking picture with ID {self.picture_id}")

        if not self.blackboard.exists(self.blackboard_key):
            self.blackboard.set(self.blackboard_key, True)
        else:
            self.logger.info(f"Picture with ID {self.picture_id} already taken. Taking another picture.")
            self.blackboard.set(self.blackboard_key, True)

        return py_trees.common.Status.SUCCESS

## Conditions ##
class IsRobotClose(ToBlackboard):
    def __init__(self, name="IsRobotClose", x="0", y="0", range="0.1"):
        super().__init__(topic_name="/model/curiosity_mars_rover/odometry", topic_type=Odometry, qos_profile=10, blackboard_variables={"odom_pose": "pose.pose"}, name=name, clearing_policy=py_trees.common.ClearingPolicy.NEVER)
        self.target_x = float(x)
        self.target_y = float(y)
        self.range = float(range)
        self.blackboard_client = self.attach_blackboard_client(name="IsRobotCloseClient")
        self.blackboard_client.register_key(key="odom_pose", access=py_trees.common.Access.READ)

    def update(self):
        self.logger.info(f"Checking if robot is close to ({self.target_x}, {self.target_y})")

        to_blackboard_result = super().update()
        if to_blackboard_result == py_trees.common.Status.SUCCESS:
            print(self.blackboard_client)
            if not self.blackboard_client.exists("odom_pose"):
                self.logger.warn("No pose available on blackboard")
                return py_trees.common.Status.RUNNING

            pose = self.blackboard_client.get("odom_pose")
            x = pose.position.x
            y = pose.position.y
            dist = math.hypot(self.target_x - x, self.target_y - y)

            if dist <= self.range:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE

        else:
            return to_blackboard_result
    
class LocationReached(py_trees.behaviour.Behaviour):
    def __init__(self, name="LocationReached", x="0", y="0"):
        super().__init__(name)
        self.x = float(x)
        self.y = float(y)
        self.blackboard_client = self.attach_blackboard_client(name="LocationReachedClient")
        self.blackboard_client.register_key("last_reached_location/x", py_trees.common.Access.READ)
        self.blackboard_client.register_key("last_reached_location/y", py_trees.common.Access.READ)

    def update(self):
        self.logger.info(f"Checking if robot reached location ({self.x}, {self.y})")
        if self.blackboard_client.exists("last_reached_location/x") and self.blackboard_client.exists("last_reached_location/y"):
            robot_x = self.blackboard_client.get("last_reached_location/x")
            robot_y = self.blackboard_client.get("last_reached_location/y")
            if abs(self.x - robot_x) <= 0.1 and abs(self.y - robot_y) <= 0.1:
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class IsPictureTaken(py_trees.behaviour.Behaviour):
    def __init__(self, name="IsPictureTaken", picture_id="0"):
        super().__init__(name)
        self.picture_id = int(picture_id)

    def update(self):
        self.logger.info(f"Checking if picture with ID {self.picture_id} is taken")
        return py_trees.common.Status.SUCCESS