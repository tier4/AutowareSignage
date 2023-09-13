# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

from PyQt5.QtMultimedia import QSound
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory
from pulsectl import Pulse

from tier4_hmi_msgs.srv import Volume
from tier4_external_api_msgs.msg import ResponseStatus

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
    def __init__(self, node, autoware_interface, parameter_interface):
        super(AnnounceControllerProperty, self).__init__()

        self._node = node
        self._parameter = parameter_interface.parameter
        self._current_announce = ""
        self._pending_announce_list = []
        self._sound = QSound("")
        self._prev_depart_and_arrive_type = ""
        self._package_path = get_package_share_directory("signage") + "/resource/sound/"
        self._check_playing_timer = self._node.create_timer(1, self.check_playing_callback)

        self._pulse = Pulse()
        # Get default sink at startup
        self._sink = self._pulse.get_sink_by_name(self._pulse.server_info().default_sink_name)
        self._node.create_service(Volume, "~/volume", self.set_volume)

    def process_pending_announce(self):
        try:
            for play_sound in self._pending_announce_list:
                time_diff = self._node.get_clock().now() - play_sound["requested_time"]
                if not self._parameter.signage_stand_alone and time_diff > Duration(seconds=5):
                    # delay the announce for going to depart when the signage is not stand alone
                    self.play_sound(play_sound["message"])
                    self._pending_announce_list.remove(play_sound)
                    break
                elif self._parameter.signage_stand_alone and time_diff <= Duration(seconds=10):
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

    def announce_arrived(self):
        if self._parameter.signage_stand_alone:
            self.send_announce("thank_you")

    def announce_emergency(self, message):
        if self._parameter.signage_stand_alone:
            self.send_announce(message)

    def announce_going_to_depart_and_arrive(self, message):
        if self._prev_depart_and_arrive_type != message:
            # To stop repeat announcement
            self.send_announce(message)
            self._prev_depart_and_arrive_type = message

    def set_volume(self, request, response):
        try:
            self._pulse.volume_set_all_chans(self._sink, request.volume)
            response.status.code = ResponseStatus.SUCCESS
        except Exception:
            response.status.code = ResponseStatus.ERROR
        return response
