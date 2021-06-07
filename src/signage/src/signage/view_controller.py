#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8
from PyQt5.QtCore import pyqtProperty
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QObject
from PyQt5.QtCore import pyqtSlot

import rospy

from autoware_api_msgs.msg import AwapiAutowareStatus

class ViewControllerProperty(QObject):
    _view_mode_changed_signal = pyqtSignal(str)
    def __init__(self):
        super(ViewControllerProperty, self).__init__()
        rospy.Subscriber("/awapi/autoware/get/status", AwapiAutowareStatus, self.sub_autoware_status)

        self.is_auto_mode = False
        self.is_emergency_mode = False
        self.is_stopping = False
        self.is_driving = False
        self._view_mode = ""

        rospy.Timer(rospy.Duration(0.1), self.update_view_state)


    # view mode
    @pyqtProperty(str, notify=_view_mode_changed_signal)
    def view_mode(self):
        return self._view_mode

    @view_mode.setter
    def view_mode(self, view_mode):
        self._view_mode = view_mode
        self._view_mode_changed_signal.emit(view_mode)

    def update_view_state(self, event):
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


    def sub_autoware_status(self, message):
        self.is_emergency_mode = message.emergency_stopped
        self.is_auto_mode = message.control_mode == 1
        self.is_stopping = message.autoware_state != "Driving" and message.autoware_state != "InitializingVehicle"
        self.is_driving = message.autoware_state == "Driving"
