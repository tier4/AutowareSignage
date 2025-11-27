# This Python file uses the following encoding: utf-8
import sys

import rclpy
from rclpy.node import Node

from autonomous_state_display.autonomous_state_display_core import AutonomousStateDisplay
from ament_index_python.packages import get_package_share_directory

def main(args=None):
    package_path = get_package_share_directory("autonomous_state_display")

    rclpy.init(args=args)
    node = Node("autonomous_state_display")

    autonomous_state_display = AutonomousStateDisplay(node)

    while True:
        rclpy.spin_once(node, timeout_sec=0.01)


if __name__ == "__main__":
    main()
