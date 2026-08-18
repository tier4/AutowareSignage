# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

import os

from PyQt5.QtMultimedia import QSound
from rclpy.duration import Duration
from ament_index_python.packages import get_package_share_directory
from pulsectl import Pulse

from std_msgs.msg import Float32
from std_srvs.srv import Trigger
from tier4_hmi_msgs.srv import SetVolume
from tier4_external_api_msgs.msg import ResponseStatus

from signage.settings_store import KEY_VOLUME

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
    # UC-04 停止案内 (SYS-HMI-04/05/06)。障害物/横断歩道/一般とも共通で「停車します」。
    "temporary_stop": 2,
    # 立席運行 転倒防止アナウンス (MRM=3 の下に配置)
    # 急減速・急操舵は priority 2。発車警告は engage(2) に続けて再生させるため
    # priority 1 とし、engage 再生中はキューに積んで後続で再生する。
    # 急減速/急操舵/複合とも音声は「手すりにおつかまりください」で統一する
    # (表示は standing_warning_type で事象別に出し分ける)。
    "standing_sudden": 2,
    "standing_depart": 1,
    "going_to_depart": 1,
    "going_to_arrive": 1,
    "arrive_caution": 1,  # UC-05 接近時(10m)の車内安全配慮アナウンス (SYS-HMI-07)
}

class AnnounceControllerProperty:
    def __init__(self, node, autoware_interface, parameter_interface, settings_store):
        super(AnnounceControllerProperty, self).__init__()

        self._node = node
        self._parameter = parameter_interface.parameter
        self._settings = settings_store
        self._announce_settings = parameter_interface.announce_settings
        self._announce_interval = parameter_interface.announce_interval
        # 発話カテゴリごとの前回発話時刻 (VVAS の TimeoutClass 相当)。
        # 未発話のカテゴリはキーを持たず、in_interval は False を返す。
        self._announce_timeout = {}
        self._current_announce = ""
        self._pending_announce_list = []
        self._sound = QSound("")
        self._prev_depart_and_arrive_type = ""
        self._package_path = get_package_share_directory("signage") + "/resource/sound/"
        self._check_playing_timer = self._node.create_timer(1, self.check_playing_callback)

        self._pulse = Pulse()
        volume = self._settings.get(KEY_VOLUME)
        if volume is not None:
            self._sink = self._pulse.get_sink_by_name(
                self._pulse.server_info().default_sink_name
            )
            self._pulse.volume_set_all_chans(self._sink, float(volume))

        self._get_volume_pub = self._node.create_publisher(Float32, "~/get/volume", 1)
        self._node.create_timer(1.0, self.publish_volume_callback)
        self._node.create_service(SetVolume, "~/set/volume", self.set_volume)
        self._node.create_service(Trigger, "~/test/volume", self.test_volume)

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
        # 音声ファイルが無い/再生に失敗した場合は False を返し、呼び出し側で
        # 提供失敗として記録できるようにする (NFR-08/UC02-04/UC03-05)
        sound_path = self._package_path + message + ".wav"
        if not os.path.isfile(sound_path):
            self._node.get_logger().error(
                "announce sound file not found: {}".format(sound_path)
            )
            return False
        try:
            self._sound = QSound(sound_path)
            self._sound.play()
            return True
        except Exception as e:
            self._node.get_logger().error(
                "not able to play the announce '{}': {}".format(message, str(e))
            )
            return False

    # skip announce by setting
    def check_announce_or_not(self, message):
        try:
            # 急減速・急操舵の各アナウンスは単一の standing_sudden フラグで制御する
            if message.startswith("standing_sudden"):
                return self._announce_settings.standing_sudden
            return getattr(self._announce_settings, message)
        except Exception as e:
            self._node.get_logger().error("check announce or not: " + str(e))
            return False

    def _log_announce_outcome(self, message, outcome):
        # 全アナウンス共通の提供結果ログ (NFR-08/NFR-10)。
        # played / queued / skipped_by_setting / suppressed_by_priority / failed
        # 提供失敗のみ error、それ以外は info で記録する。戻り値はそのまま返す
        log = "announce '{}' provided -> {}".format(message, outcome)
        if outcome == "failed":
            self._node.get_logger().error(log)
        else:
            self._node.get_logger().info(log)
        return outcome

    def send_announce(self, message):
        # 提供結果を返す: played / queued / skipped_by_setting /
        # suppressed_by_priority / failed
        priority = PRIORITY_DICT.get(message, 0)
        previous_priority = PRIORITY_DICT.get(self._current_announce, 0)

        if not self.check_announce_or_not(message):
            return self._log_announce_outcome(message, "skipped_by_setting")

        def _play():
            return "played" if self.play_sound(message) else "failed"

        outcome = "suppressed_by_priority"
        if priority == 3:
            self._sound.stop()
            outcome = _play()
        elif priority == 2:
            if priority > previous_priority:
                self._sound.stop()
                outcome = _play()
            elif priority == previous_priority:
                outcome = _play()
        elif priority == 1:
            if not previous_priority:
                outcome = _play()
            elif previous_priority in [2, 1]:
                self._pending_announce_list.append(
                    {
                        "message": message,
                        "requested_time": self._node.get_clock().now(),
                    }
                )
                outcome = "queued"
        self._current_announce = message
        return self._log_announce_outcome(message, outcome)

    def stop_standing_announce(self):
        # MRM最優先: 再生中の立席アナウンスを停止する (画面はMRMへ切替わるため音声も揃える)
        if self._current_announce.startswith("standing_"):
            self._sound.stop()
            self._current_announce = ""

    def in_interval(self, category):
        # VVAS の announce_interval と同方式: 前回発話から interval 秒未満なら True
        # (= 再発話抑止中)。未発話のカテゴリは False (起動直後に警告表示が出ないように)。
        trigger_time = self._announce_timeout.get(category)
        if trigger_time is None:
            return False
        duration = getattr(self._announce_interval, category)
        return self._node.get_clock().now() - trigger_time < Duration(seconds=duration)

    def set_timeout(self, category):
        # 発話トリガー時に呼び、当該カテゴリの前回発話時刻を「今」に更新する
        self._announce_timeout[category] = self._node.get_clock().now()

    def announce_arrived(self):
        if self._parameter.signage_stand_alone:
            # 終点・通常停留所とも音声は「ご乗車ありがとうございました」(thank_you) を流用する。
            # 到着停留所名は route_handler 側の乗客サイネージ表示で提示する (SYS-HMI-07)。
            self.send_announce("thank_you")

    def in_interval(self, category):
        # VVAS の announce_interval と同方式: 前回発話から interval 秒未満なら True
        # (= 再発話抑止中)。未発話のカテゴリは False (起動直後に表示が出ないように)。
        trigger_time = self._announce_timeout.get(category)
        if trigger_time is None:
            return False
        duration = getattr(self._announce_interval, category)
        return self._node.get_clock().now() - trigger_time < Duration(seconds=duration)

    def set_timeout(self, category):
        # 発話トリガー時に呼び、当該カテゴリの前回発話時刻を「今」に更新する
        self._announce_timeout[category] = self._node.get_clock().now()

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
            self._settings.set(KEY_VOLUME, self._sink.volume.value_flat)
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
