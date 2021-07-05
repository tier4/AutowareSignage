#!/usr/bin/env python
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

from PyQt5.QtCore import pyqtSignal
from PyQt5.QtCore import QObject

import rospy
from std_msgs.msg import String

class AnnounceControllerProperty(QObject):
    _announce_signal = pyqtSignal(str)
    def __init__(self, autoware_state_interface=None):
        super(AnnounceControllerProperty, self).__init__()
        autoware_state_interface.set_autoware_state_callback(self.sub_autoware_state)
        autoware_state_interface.set_emergency_stopped_callback(self.sub_emergency)

        self._in_driving_state = False
        self._in_emergency_state = False
        self._autoware_state = ""
        self._emergency_trigger_time = 0

    def sub_autoware_state(self, autoware_state):
        self._autoware_state = autoware_state
        if autoware_state == "Driving" and not self._in_driving_state:
            self._announce_signal.emit("engage")
            self._in_driving_state = True
        elif autoware_state == "ArrivedGoal" and self._in_driving_state:
            self._announce_signal.emit("arrived")
            self._in_driving_state = False

    def sub_emergency(self, emergency_stopped):
        if emergency_stopped and not self._in_emergency_state:
            self._announce_signal.emit("emergency")
            self._in_emergency_state = True
        elif not emergency_stopped and self._in_emergency_state:
            self._announce_signal.emit("emergency_cancel")
            self._in_emergency_state = False
        elif emergency_stopped and self._in_emergency_state:
            if not self._emergency_trigger_time:
                self._emergency_trigger_time = rospy.get_time()
            elif rospy.get_time() - self._emergency_trigger_time > 180:
                self._announce_signal.emit("in_emergency")
                self._emergency_trigger_time = 0

    def announce_going_to_depart_and_arrive(self, message):
        if message == "going_to_depart":
            rospy.sleep(5)
            self._announce_signal.emit("going_to_depart")
        elif message == "going_to_arrive" and self._autoware_state == "Driving":
            self._announce_signal.emit("going_to_arrive")
