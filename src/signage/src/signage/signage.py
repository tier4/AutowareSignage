# This Python file uses the following encoding: utf-8
import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtQml import QQmlApplicationEngine

from PyQt5.QtCore import pyqtProperty
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QObject
from PyQt5.QtCore import pyqtSlot

from ament_index_python import get_resource

import rclpy
from rclpy.node import Node
from autoware_system_msgs.msg import AutowareState

from signage.view_controller import ViewControllerProperty

def main(args=None):
    _, package_path = get_resource('packages', 'signage')

    rclpy.init(args=args)
    node = Node("signage")

    app = QApplication(sys.argv)
    engine = QQmlApplicationEngine()

    viewController = ViewControllerProperty(node)

    ctx = engine.rootContext()
    ctx.setContextProperty("viewController", viewController)
    engine.load(os.path.join(package_path, 'share', 'signage', 'resource', 'page', 'main.qml'))

    if not engine.rootObjects():
        rclpy.shutdown()
        sys.exit(-1)

    while True:
        app.processEvents()
        rclpy.spin_once(node,timeout_sec=0.01)


if __name__ == "__main__":
    main()


