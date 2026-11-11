# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

import os
import time
import wave

from PyQt5.QtMultimedia import QSound
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory
from pulsectl import Pulse

from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from tier4_hmi_msgs.srv import SetVolume
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
    "will_stop": 1,
    "cancel": 1,
}

CURRENT_VOLUME_PATH = "/opt/autoware/volume.txt"

# Seconds added to the wav length before assuming playback ended without isFinished().
PLAYBACK_FALLBACK_MARGIN = 1.5
# Queued will_stop / cancel older than this are stale and dropped.
QUEUED_ANNOUNCE_MAX_AGE = 30.0
# Replay an interrupted will_stop only if less than this share was heard.
WILL_STOP_REPLAY_MAX_PROGRESS = 0.7
# Stop request messages that follow the next bus stop.
STOP_REQUEST_MESSAGES = ("will_stop", "cancel")


class AnnounceControllerProperty:
    def __init__(self, node, autoware_interface, parameter_interface):
        super(AnnounceControllerProperty, self).__init__()

        self._node = node
        self._parameter = parameter_interface.parameter
        self._announce_settings = parameter_interface.announce_settings
        self._current_announce = ""
        self._pending_announce_list = []
        self._sound = QSound("")
        # QSound.isFinished() stays false on some ECU images, which leaves
        # priority-1 clips such as will_stop and cancel queued forever.
        self._sound_started_monotonic = None
        self._sound_duration = 0.0
        self._prev_depart_and_arrive_type = ""
        self._package_path = get_package_share_directory("signage") + "/resource/sound/"
        self._check_playing_timer = self._node.create_timer(1, self.check_playing_callback)

        self._pulse = Pulse()
        if os.path.isfile(CURRENT_VOLUME_PATH):
            with open(CURRENT_VOLUME_PATH, "r") as f:
                volume = f.readline()
                if volume != "":
                    self._sink = self._pulse.get_sink_by_name(
                        self._pulse.server_info().default_sink_name
                    )
                    self._pulse.volume_set_all_chans(self._sink, float(volume))

        self._get_volume_pub = self._node.create_publisher(Float32, "~/get/volume", 1)
        self._node.create_timer(1.0, self.publish_volume_callback)
        self._node.create_service(SetVolume, "~/set/volume", self.set_volume)
        self._node.create_service(Trigger, "~/test/volume", self.test_volume)

    def _enqueue_announce(self, message, front=False):
        item = {
            "message": message,
            "requested_time": self._node.get_clock().now(),
        }
        if message in STOP_REQUEST_MESSAGES:
            # Keep one queued clip, and let the newest of will_stop / cancel win:
            # the stop request it replaces no longer applies to the next stop.
            self._pending_announce_list = [
                pending
                for pending in self._pending_announce_list
                if pending["message"] not in STOP_REQUEST_MESSAGES
            ]
        if front:
            self._pending_announce_list.insert(0, item)
        else:
            self._pending_announce_list.append(item)

    def _defer_will_stop(self):
        """Replay will_stop after a higher-priority announce preempts it."""
        if self._current_announce != "will_stop" or self._playback_finished():
            return
        elapsed = self._playback_elapsed()
        if (
            elapsed is not None
            and self._sound_duration > 0
            and elapsed >= self._sound_duration * WILL_STOP_REPLAY_MAX_PROGRESS
        ):
            # Almost fully heard already; replaying would sound like a duplicate.
            return
        self._enqueue_announce("will_stop", front=True)

    def _resolve_sound_path(self, message):
        filename = message + ".wav"
        candidates = [
            os.path.join(self._package_path, filename),
            os.path.normpath(
                os.path.join(os.path.dirname(__file__), "..", "..", "resource", "sound", filename)
            ),
        ]
        for path in candidates:
            if os.path.isfile(path):
                return path
        self._node.get_logger().error("sound file not found: " + candidates[0])
        return candidates[0]

    def _wav_duration_seconds(self, path):
        try:
            with wave.open(path) as wav_file:
                rate = wav_file.getframerate()
                if rate <= 0:
                    return 0.0
                return wav_file.getnframes() / float(rate)
        except Exception:
            return 0.0

    def _playback_elapsed(self):
        if self._sound_started_monotonic is None:
            return None
        return time.monotonic() - self._sound_started_monotonic

    def _playback_finished(self):
        elapsed = self._playback_elapsed()
        if elapsed is None:
            return True
        if self._sound.isFinished():
            return True
        # Fallback for ECUs where isFinished() never becomes true. The margin
        # covers the asynchronous load before audio actually starts, so the next
        # clip does not cut the current one.
        return elapsed >= (self._sound_duration + PLAYBACK_FALLBACK_MARGIN)

    def _play_pending(self, play_sound):
        self.play_sound(play_sound["message"])
        self._current_announce = play_sound["message"]
        self._pending_announce_list.remove(play_sound)

    def process_pending_announce(self):
        try:
            for play_sound in self._pending_announce_list:
                time_diff = self._node.get_clock().now() - play_sound["requested_time"]
                message = play_sound["message"]
                # will_stop / cancel must still play after engage, even if the
                # request has been waiting longer than the usual announce window.
                if message in STOP_REQUEST_MESSAGES:
                    if time_diff > Duration(seconds=QUEUED_ANNOUNCE_MAX_AGE):
                        self._pending_announce_list.remove(play_sound)
                        break
                    ready = True
                elif not self._parameter.signage_stand_alone and time_diff > Duration(seconds=5):
                    # delay the announce for going to depart when the signage is not stand alone
                    ready = True
                elif self._parameter.signage_stand_alone and time_diff <= Duration(seconds=10):
                    ready = True
                else:
                    ready = False
                if ready:
                    self._play_pending(play_sound)
                    break
        except Exception as e:
            self._node.get_logger().error("not able to check the pending playing list: " + str(e))

    def check_playing_callback(self):
        try:
            if self._playback_finished():
                self._current_announce = ""
                self.process_pending_announce()
        except Exception as e:
            self._node.get_logger().error("not able to check the current playing: " + str(e))

    def play_sound(self, message):
        path = self._resolve_sound_path(message)
        self._sound = QSound(path)
        self._sound.play()
        self._sound_started_monotonic = time.monotonic()
        self._sound_duration = self._wav_duration_seconds(path)

    # skip announce by setting
    def check_announce_or_not(self, message):
        try:
            return getattr(self._announce_settings, message)
        except Exception as e:
            self._node.get_logger().error("check announce or not: " + str(e))
            return False

    def send_announce(self, message):
        priority = PRIORITY_DICT.get(message, 0)

        if not self.check_announce_or_not(message):
            return

        # Clear a finished clip first, so a stale name does not block this announce.
        if self._playback_finished():
            self._current_announce = ""
        previous_priority = PRIORITY_DICT.get(self._current_announce, 0)

        if message in STOP_REQUEST_MESSAGES and self._current_announce == message:
            # The same clip is already playing; do not repeat it.
            return

        played = False
        if priority == 3:
            self._defer_will_stop()
            self._sound.stop()
            self.play_sound(message)
            played = True
        elif priority == 2:
            if priority > previous_priority:
                self._defer_will_stop()
                self._sound.stop()
                self.play_sound(message)
                played = True
            elif priority == previous_priority:
                self.play_sound(message)
                played = True
        elif priority == 1:
            if not previous_priority:
                self.play_sound(message)
                played = True
            elif previous_priority in [2, 1] or message in STOP_REQUEST_MESSAGES:
                # Play will_stop or cancel after a higher-priority clip such as engage.
                self._enqueue_announce(message)
        if played:
            self._current_announce = message

    def announce_arrived(self):
        if self._parameter.signage_stand_alone:
            self.send_announce("thank_you")

    def announce_emergency(self, message):
        if self._parameter.signage_stand_alone:
            self.send_announce(message)

    def announce_going_to_depart_and_arrive(self, message):
        self.send_announce(message)

    def publish_volume_callback(self):
        self._sink = self._pulse.get_sink_by_name(self._pulse.server_info().default_sink_name)
        self._get_volume_pub.publish(Float32(data=self._sink.volume.value_flat))

    def set_volume(self, request, response):
        try:
            self._sink = self._pulse.get_sink_by_name(self._pulse.server_info().default_sink_name)
            self._pulse.volume_set_all_chans(self._sink, request.volume)
            with open(CURRENT_VOLUME_PATH, "w") as f:
                f.write(f"{self._sink.volume.value_flat}\n")
            response.status.code = ResponseStatus.SUCCESS
        except Exception:
            response.status.code = ResponseStatus.ERROR
        return response

    def test_volume(self, request, response):
        try:
            self.play_sound("test_volume")
            response.success = True
        except Exception:
            response.success = False
        return response

