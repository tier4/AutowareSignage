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

from autoware_api_msgs.msg import AwapiAutowareStatus
from autoware_api_msgs.msg import AwapiVehicleStatus

class ViewControllerProperty(QObject):
    _view_mode_changed_signal = pyqtSignal(str)
    def __init__(self, node=None):
        super(ViewControllerProperty, self).__init__()
        node.get_logger().info("%s initializing..." % "signage")
        self._node = node
        self.sub = self._node.create_subscription(AwapiAutowareStatus, "/awapi/autoware/get/status", self.sub_autoware_status, 10)

        self._cycle_view_control_timer = self._node.create_timer(
            0.1,
            self.pub_view_state_control)

        self.is_auto_mode = False
        self.is_emergency_mode = False
        self.is_stopping = False
        self.is_driving = False
        self._view_mode = ""

    # view mode
    @pyqtProperty(str, notify=_view_mode_changed_signal)
    def view_mode(self):
        return self._view_mode

    @view_mode.setter
    def view_mode(self, view_mode):
        self._view_mode = view_mode
        self._view_mode_changed_signal.emit(view_mode)

    def pub_view_state_control(self):
        if self.is_emergency_mode:
            self.view_mode = "emergency_stopped"
        elif not self.is_auto_mode:
            self.view_mode = "manual_driving"
        elif self.is_stopping:
            self.view_mode = "stopping"
        elif self.is_driving:
            self.view_mode = "driving"
        else:
            self.view_mode = "out_of_service"

        # self._node.get_logger().info('view mode %r' % (self._view_mode))

    def sub_autoware_status(self, message):
        self.is_emergency_mode = message.emergency_stopped
        self.is_auto_mode = message.control_mode == 1
        self.is_stopping = message.autoware_state != "Driving" and message.autoware_state != "InitializingVehicle"
        self.is_driving = message.autoware_state == "Driving"
