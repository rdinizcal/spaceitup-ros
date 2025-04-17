import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

import py_trees_ros
import py_trees.console as console

from py_trees_bt_runner.utils.node_factory import NODE_FACTORY
from py_trees_bt_runner.utils.xml_bt_parser import XmlBTParser

class BehaviorTreeRunner(Node):
    def __init__(self):
        super().__init__('bt_runner_node')

        self.callback_group = ReentrantCallbackGroup()

        self.parser = XmlBTParser(NODE_FACTORY)
        self.tree = None
        self.root = None
        self.tick_period_ms = 1000.0

        self.subscription = self.create_subscription(String, 'behavior_tree', self.load_tree_callback, 10, callback_group=self.callback_group)
        self.get_logger().info("BT Executor started. Waiting for behavior tree...")

    def load_tree_callback(self, msg):
        self.get_logger().info("Received a new tree!")
        self.root = self.parser.load_tree_from_string(msg.data)
        self.setup_and_start()

    def setup_and_start(self):
        if self.tree is not None:
            self.get_logger().info("Shutting down current tree...")
            self.tree.shutdown()

        self.tree = py_trees_ros.trees.BehaviourTree(
            root=self.root,
            unicode_tree_debug=True
        )

        try:
            self.get_logger().info("Starting new tree execution...")
            # self.tree.setup(node_name="behavior_tree_execution", timeout=15.0)
            self.tree.setup(timeout=15.0)
        except py_trees_ros.exceptions.TimedOutError as e:
            console.logerror(console.red + "Failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
            self.tree.shutdown()
            rclpy.try_shutdown()
            exit(1)
        except KeyboardInterrupt:
            console.logerror("Tree setup interrupted")
            self.tree.shutdown()
            rclpy.try_shutdown()
            exit(1)

        print("Tree setup complete")
        # self.tree.tick_tock(period_ms=self.tick_period_ms)
        if self.tick_timer is not None:
            self.tick_timer.cancel()
        self.tick_timer = self.create_timer(1.0, self.tick_tree)

    def shutdown(self):
        if self.tree:
            self.tree.shutdown()

    def tick_tree(self):
        if self.tree:
            self.tree.tick()

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeRunner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
