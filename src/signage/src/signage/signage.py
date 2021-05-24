#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8
import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget
from PyQt5.QtQml import QQmlApplicationEngine

from PyQt5.QtCore import pyqtProperty
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QObject
from PyQt5.QtCore import pyqtSlot

import rospy, rospkg

from autoware_system_msgs.msg import AutowareState

from signage.view_controller import ViewControllerProperty

def main(args=None):
    rospy.init_node('signage', anonymous=True)

    app = QApplication(sys.argv)
    engine = QQmlApplicationEngine()

    viewController = ViewControllerProperty()

    ctx = engine.rootContext()
    ctx.setContextProperty("viewController", viewController)
    engine.load(os.path.join(rospkg.RosPack().get_path('signage'), 'resource', 'page', 'main.qml'))

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()


