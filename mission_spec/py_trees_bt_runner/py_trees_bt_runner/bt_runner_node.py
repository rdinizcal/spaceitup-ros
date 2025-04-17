import sys
import rclpy
import py_trees_ros
import py_trees.console as console

from py_trees_bt_runner.utils.node_factory import NODE_FACTORY
from py_trees_bt_runner.utils.xml_bt_parser import XmlBTParser
import os

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) > 1:
        tree_path = sys.argv[1]
    else:
        default_tree = "trees/inspection_mission.xml" # Default tree path from the workspace folder

        tree_path = os.path.join(os.path.dirname(__file__).split("/install")[0], "src", default_tree)

    parser = XmlBTParser(NODE_FACTORY)

    root = parser.load_tree_from_file(tree_path)
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )

    try:
        tree.setup(node_name="bt_executor", timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        exit(1)
    except KeyboardInterrupt:
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        exit(1)

    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()