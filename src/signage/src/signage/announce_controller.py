# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

from PyQt5.QtCore import QObject
from PyQt5.QtMultimedia import QSound

import time
from ament_index_python.packages import get_package_share_directory

# The higher the value, the higher the priority
PRIORITY_DICT = {
    "emergency" : 3,
    "emergency_cancel" : 3,
    "engage" : 2,
    "arrived" : 2,
    "in_emergency" : 2,
    "going_to_depart" : 1,
    "going_to_arrive" : 1
}

class AnnounceControllerProperty(QObject):
    def __init__(self, node, autoware_state_interface=None):
        super(AnnounceControllerProperty, self).__init__()
        autoware_state_interface.set_autoware_state_callback(self.sub_autoware_state)
        autoware_state_interface.set_emergency_stopped_callback(self.sub_emergency)

        self._node = node
        self._in_driving_state = False
        self._in_emergency_state = False
        self._autoware_state = ""
        self._current_announce = ""
        self._pending_announce_list = []
        self._emergency_trigger_time = 0
        self._sound = QSound("")
        self._package_path = get_package_share_directory('signage') + "/resource/sound/"
        self._check_playing_timer = self._node.create_timer(
            1,
            self.check_playing_callback)

    def process_pending_announce(self):
        try:
            for play_sound in self._pending_announce_list:
                self._pending_announce_list.remove(play_sound)
                current_time = self._node.get_clock().now().to_msg().sec
                if current_time - play_sound["requested_time"] <= 10:
                    self.send_announce(play_sound["message"])
                    break
        except Exception as e:
            self._node.get_logger().error("not able to check the pending playing list: " + str(e))


    def check_playing_callback(self):
        try:
            if not self._current_announce:
                return

            if self._sound.isFinished():
                self._current_announce = ""
                self.process_pending_announce()
        except Exception as e:
            self._node.get_logger().error("not able to check the current playing: " + str(e))

    def play_sound(self, message):
        self._sound = QSound(self._package_path + message + ".wav")
        self._sound.play()

    def send_announce(self, message):
        priority = PRIORITY_DICT.get(message, 0)
        previous_priority = PRIORITY_DICT.get(self._current_announce, 0)

        if priority == 3:
            self._sound.stop()
            self.play_sound(message)
        elif priority == 2:
            if priority > previous_priority:
                self._sound.stop()
                self.play_sound(message)
            elif priority == previous_priority:
                self.play_sound(message)
        elif priority == 1:
            if not previous_priority:
                self.play_sound(message)
            elif previous_priority in [2,1]:
                self._pending_announce_list.append(
                    {
                        "message":message,
                        "requested_time":self._node.get_clock().now().to_msg().sec
                    })
        self._current_announce = message

    def sub_autoware_state(self, autoware_state):
        self._autoware_state = autoware_state
        if autoware_state == "Driving" and not self._in_driving_state:
            self.send_announce("engage")
            self._in_driving_state = True
        elif autoware_state == "ArrivedGoal" and self._in_driving_state:
            self.send_announce("arrived")
            self._in_driving_state = False

    def sub_emergency(self, emergency_stopped):
        if emergency_stopped and not self._in_emergency_state:
            self.send_announce("emergency")
            self._in_emergency_state = True
        elif not emergency_stopped and self._in_emergency_state:
            self.send_announce("emergency_cancel")
            self._in_emergency_state = False
        elif emergency_stopped and self._in_emergency_state:
            if not self._emergency_trigger_time:
                self._emergency_trigger_time = self._node.get_clock().now().to_msg().sec
            elif self._node.get_clock().now().to_msg().sec - self._emergency_trigger_time > 180:
                self.send_announce("in_emergency")
                self._emergency_trigger_time = 0

    def announce_going_to_depart_and_arrive(self, message):
        if message == "going_to_depart":
            self.send_announce("going_to_depart")
        elif message == "going_to_arrive" and self._autoware_state == "Driving":
            self.send_announce("going_to_arrive")
