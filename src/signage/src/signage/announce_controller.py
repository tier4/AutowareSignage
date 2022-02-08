# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

from PyQt5.QtMultimedia import QSound

import simpleaudio as sa
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory
from tier4_hmi_msgs.srv import Announce

# The higher the value, the higher the priority
PRIORITY_DICT = {
    "emergency": 3,
    "restart_engage": 3,
    "door_close": 3,
    "door_open": 3,
    "engage": 2,
    "arrived": 2,
    "thank_you": 2,
    "in_emergency": 2,
    "going_to_depart": 1,
    "going_to_arrive": 1,
}


class AnnounceControllerProperty:
    def __init__(self, node, autoware_state_interface=None):
        super(AnnounceControllerProperty, self).__init__()

        self._node = node
        self._prev_autoware_state = ""
        self._prev_prev_autoware_state = ""
        self._in_driving_state = False
        self._in_emergency_state = False
        self._autoware_state = ""
        self._current_announce = ""
        self._is_auto_mode = False
        self._is_auto_running = False
        self._door_announce = True
        # self._pre_door_announce_status = 0
        self._pending_announce_list = []
        self._emergency_trigger_time = 0
        self._sound = QSound("")
        self._node.declare_parameter("signage_stand_alone", False)
        self._signage_stand_alone = (
            self._node.get_parameter("signage_stand_alone").get_parameter_value().bool_value
        )
        self._package_path = get_package_share_directory("signage") + "/resource/sound/"
        self._check_playing_timer = self._node.create_timer(1, self.check_playing_callback)
        self._srv = self._node.create_service(
            Announce, "/api/signage/set/announce", self.announce_service
        )
        autoware_state_interface.set_velocity_callback(self.sub_velocity)
        autoware_state_interface.set_door_status_callback(self.sub_door_status)

    def announce_service(self, request, response):
        try:
            if self._signage_stand_alone:
                filename = ""
                annouce_type = request.kind
                if annouce_type == 1:
                    filename = self._package_path + "engage.wav"
                elif annouce_type == 2 and self._is_auto_running:
                    filename = self._package_path + "restart_engage.wav"
                if filename:
                    wave_obj = sa.WaveObject.from_wave_file(filename)
                    play_obj = wave_obj.play()
                    play_obj.wait_done()
                self._node.get_logger().info(
                    "return announce response: {}".format(str(annouce_type))
                )
        except Exception as e:
            self._node.get_logger().error("not able to play the annoucen, ERROR: {}".format(str(e)))
        return response

    def process_pending_announce(self):
        try:
            for play_sound in self._pending_announce_list:
                time_diff = self._node.get_clock().now() - play_sound["requested_time"]
                if not self._signage_stand_alone and time_diff > Duration(seconds=5):
                    # delay the announce for going to depart when the signage is not stand alone
                    self.play_sound(play_sound["message"])
                    self._pending_announce_list.remove(play_sound)
                    break
                elif self._signage_stand_alone and time_diff <= Duration(seconds=10):
                    self.play_sound(play_sound["message"])
                    self._pending_announce_list.remove(play_sound)
                    break
        except Exception as e:
            self._node.get_logger().error("not able to check the pending playing list: " + str(e))

    def check_playing_callback(self):
        try:
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
            elif previous_priority in [2, 1]:
                self._pending_announce_list.append(
                    {
                        "message": message,
                        "requested_time": self._node.get_clock().now(),
                    }
                )
        self._current_announce = message

    def sub_velocity(self, velocity):
        if velocity > 0:
            self._is_auto_running = True
        elif velocity < 0:
            self._is_auto_running = False

    def announce_arrived(self):
        if self._signage_stand_alone:
            self.send_announce("arrived")

    def announce_emergency(self, message):
        if self._signage_stand_alone:
            self.send_announce(message)

    def announce_going_to_depart_and_arrive(self, message):
        self.send_announce(message)

    def sub_door_status(self, door_status):
        if door_status == 1 and not self._door_announce and not self._in_emergency_state:
            # Only announce when the bus reach the goal and not in emergency state
            self.send_announce("thank_you")
            self._door_announce = True
        elif door_status == 3 and self._pre_door_announce_status != 3:
            # Should able to give warning everytime the door is opening
            self.send_announce("door_open")
            self._pre_door_announce_status = door_status
        elif door_status == 4 and self._pre_door_announce_status != 4:
            # Should able to give warning everytime the door is closing
            self.send_announce("door_close")
            self._pre_door_announce_status = door_status
        elif door_status in [0, 2, 5]:
            self._pre_door_announce_status = 0
