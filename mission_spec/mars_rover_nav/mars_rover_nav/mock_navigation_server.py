import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
import math
import time

class MockNavServer(Node):
    def __init__(self):
        super().__init__('mock_nav_server')

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'NavigateToGoal',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.odom_pub = self.create_publisher(Odometry, '/model/curiosity_mars_rover/odometry', 10)

        # Timer for idle odometry publishing
        self.odom_timer = self.create_timer(0.1, self.publish_odom)
        self.timer_active = True

        self.current_pose = Pose()
        self.current_pose.position.x = 0.0
        self.current_pose.position.y = 0.0
        self.current_pose.orientation.w = 1.0

        self.get_logger().info('NavigateToGoal server ready (with feedback and odometry timer)')

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        # Pause idle odometry publishing
        self.odom_timer.cancel()
        self.get_logger().info('Goal started: odometry timer paused')

        goal_pose = goal_handle.request.pose.pose
        self.get_logger().info(
            f"Moving to x={goal_pose.position.x}, y={goal_pose.position.y}"
        )

        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y

        dx = goal_x - start_x
        dy = goal_y - start_y
        distance = math.hypot(dx, dy)

        if distance == 0:
            goal_handle.succeed()
            self.odom_timer.reset()
            return NavigateToPose.Result()

        speed = 0.5  # units/sec
        total_time = distance / speed
        steps = int(total_time / 0.1)
        step_dx = dx / steps
        step_dy = dy / steps

        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                self.odom_timer.reset()
                return NavigateToPose.Result()

            # Update position
            self.current_pose.position.x += step_dx
            self.current_pose.position.y += step_dy
            self.current_pose.orientation = goal_pose.orientation

            self.publish_odom()

            # Send feedback
            current_dx = self.current_pose.position.x - start_x
            current_dy = self.current_pose.position.y - start_y
            traveled = math.hypot(current_dx, current_dy)

            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = distance - traveled

            time.sleep(0.1)

        # Snap to final goal
        self.current_pose.position.x = goal_x
        self.current_pose.position.y = goal_y
        self.current_pose.orientation = goal_pose.orientation

        self.publish_odom()
        self.odom_timer.reset()

        goal_handle.succeed()
        self.get_logger().info('Goal completed, odometry timer resumed.')
        return NavigateToPose.Result()

    def publish_odom(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.pose.pose = self.current_pose
        odom_msg.twist.twist = Twist()  # Still mocked
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockNavServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
