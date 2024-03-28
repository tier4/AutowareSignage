# This Python file uses the following encoding: utf-8
import sys

import rclpy
from rclpy.node import Node

from external_signage.external_signage_core import ExternalSignage
from ament_index_python.packages import get_package_share_directory

def main(args=None):
    package_path = get_package_share_directory("external_signage")

    rclpy.init(args=args)
    node = Node("external_signage")

    external_signage = ExternalSignage(node)

    while True:
        rclpy.spin_once(node, timeout_sec=0.01)


if __name__ == "__main__":
    main()
