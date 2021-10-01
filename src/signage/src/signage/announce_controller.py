# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

from PyQt5.QtMultimedia import QSound

import simpleaudio as sa
from ament_index_python.packages import get_package_share_directory
from autoware_hmi_msgs.srv import Announce

# The higher the value, the higher the priority
PRIORITY_DICT = {
    "emergency" : 3,
    "restart_engage" : 3,
    "engage" : 2,
    "arrived" : 2,
    "thank_you" : 2,
    "in_emergency" : 2,
    "going_to_depart" : 1,
    "going_to_arrive" : 1
}

class AnnounceControllerProperty():
    def __init__(self, node, autoware_state_interface=None):
        super(AnnounceControllerProperty, self).__init__()
        autoware_state_interface.set_autoware_state_callback(self.sub_autoware_state)
        autoware_state_interface.set_emergency_stopped_callback(self.sub_emergency)
        autoware_state_interface.set_control_mode_callback(self.sub_control_mode)
        autoware_state_interface.set_velocity_callback(self.sub_velocity)
        autoware_state_interface.set_door_status_callback(self.sub_door_status)

        self._node = node
        self._in_driving_state = False
        self._in_emergency_state = False
        self._autoware_state = ""
        self._current_announce = ""
        self._is_auto_mode = False
        self._is_auto_running = False
        self._sent_door_announce = False
        self._pending_announce_list = []
        self._emergency_trigger_time = 0
        self._sound = QSound("")
        self._node.declare_parameter("signage_stand_alone", False)
        self._signage_stand_alone = self._node.get_parameter("signage_stand_alone").get_parameter_value().bool_value
        self._package_path = get_package_share_directory('signage') + "/resource/sound/"
        self._check_playing_timer = self._node.create_timer(
            1,
            self.check_playing_callback)
        self._srv = self._node.create_service(Announce, '/api/signage/set/announce', self.announce_service)

    def announce_service(self, request, response):
        try:
            if self._signage_stand_alone:
                filename = ""
                annouce_type = request.kind
                if annouce_type == 1:
                    filename = self._package_path + 'engage.wav'
                elif annouce_type == 2 and self._is_auto_running:
                    filename = self._package_path + 'restart_engage.wav'
                if filename:
                    wave_obj = sa.WaveObject.from_wave_file(filename)
                    play_obj = wave_obj.play()
                    play_obj.wait_done()
            self._node.get_logger().info("return announce response")
        except Exception as e:
            self._node.get_logger().error("not able to play the annoucen, ERROR: {}".format(str(e)))
        return response

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

    def sub_control_mode(self, control_mode):
        self._is_auto_mode = control_mode == 1

    def sub_velocity(self, velocity):
        if velocity > 0 and self._is_auto_mode and self._in_driving_state:
            self._is_auto_running = True
        elif velocity < 0:
            self._is_auto_running = False

    def sub_autoware_state(self, autoware_state):
        self._autoware_state = autoware_state
        if autoware_state == "Driving" and not self._in_driving_state:
            self._in_driving_state = True
        elif autoware_state in ["WaitingForRoute", "WaitingForEngage", "ArrivedGoal", "Planning"] and self._in_driving_state:
            if self._signage_stand_alone:
                self.send_announce("arrived")
            self._is_auto_running = False
            self._in_driving_state = False

    def sub_emergency(self, emergency_stopped):
        if emergency_stopped and not self._in_emergency_state:
            self.send_announce("emergency")
            self._in_emergency_state = True
        elif not emergency_stopped and self._in_emergency_state:
            self._in_emergency_state = False
        elif emergency_stopped and self._in_emergency_state and self._signage_stand_alone:
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

    def sub_door_status(self, door_status):
        if door_status == 1 and not self._sent_door_announce:
            self.send_announce("thank_you")
            self._sent_door_announce = True
        elif door_status in [0, 2, 4, 5]:
            self._sent_door_announce = False
