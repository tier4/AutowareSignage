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

        rospy.Subscriber("/signage/put/announce", String, self.sub_announce)

        self._in_driving_state = False
        self._in_emergency_state = False
        self._delay_count = 0

    def sub_autoware_state(self, autoware_state):
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
            if self._delay_count > 200:
                self._announce_signal.emit("in_emergency")
                self._delay_count = 0
            self._delay_count += 1

    def sub_announce(self, message):
        msgs = message.data
        if msgs == "going_to_depart":
            rospy.sleep(10)
            self._announce_signal.emit("going_to_depart")
        elif msgs == "going_to_arrive":
            self._announce_signal.emit("going_to_arrive")