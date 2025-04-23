#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import math
from rclpy.duration import Duration

class MarsRoverNavServer(Node):
    def __init__(self):
        super().__init__('mars_rover_nav_server')

        self.callback_group = ReentrantCallbackGroup()

        self.action_server = ActionServer(
            self,
            NavigateToPose,
            'NavigateToGoal',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/curiosity_mars_rover/odometry',
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )

        self.current_pose = None
        self.goal_pose = None
        self.active_goal_handle = None

        self.timer = self.create_timer(0.1, self.control_loop, callback_group=self.callback_group)
        self.get_logger().info("NavigateToGoal action server ready.")

    def goal_callback(self, goal_request):
        self.get_logger().info("Received new goal request.")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal")
        self.goal_pose = goal_handle.request.pose.pose
        self.get_logger().info(f"Goal pose: {self.goal_pose}")
        self.active_goal_handle = goal_handle

        while not goal_handle.is_cancel_requested and rclpy.ok():
            if self.goal_pose is None or self.current_pose is None:
                continue

            dx = self.goal_pose.position.x - self.current_pose.position.x
            dy = self.goal_pose.position.y - self.current_pose.position.y
            distance = math.hypot(dx, dy)

            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = distance
            goal_handle.publish_feedback(feedback)

            if distance < 0.2:
                self.stop_robot()
                self.get_logger().info("Goal reached successfully.")
                goal_handle.succeed()
                result = NavigateToPose.Result()
                return result

        if goal_handle.is_cancel_requested:
            self.stop_robot()
            goal_handle.canceled()
            self.get_logger().info("Goal canceled.")
            return NavigateToPose.Result()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if self.goal_pose is None or self.current_pose is None or self.active_goal_handle is None:
            return

        if self.active_goal_handle.is_cancel_requested:
            return

        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        distance = math.hypot(dx, dy)

        yaw = self.get_yaw_from_quaternion(self.current_pose.orientation)
        target_angle = math.atan2(dy, dx)

        angle_diff = self.normalize_angle(target_angle - yaw)

        cmd = Twist()
        if distance > 0.2:
            if abs(angle_diff) > 0.1:
                cmd.angular.z = 0.5 * angle_diff
                # cmd.linear.x = max(0.5 * distance, 1.0)
                cmd.linear.x = 1.0 if 0.5 * distance > 1.0 else 0.5 * distance
            else:
                cmd.angular.z = 0.0
                cmd.linear.x = 3.0 if 0.5 * distance > 3.0 else 0.5 * distance
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.goal_pose = None
            self.active_goal_handle = None

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def get_yaw_from_quaternion(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = MarsRoverNavServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
        # rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
