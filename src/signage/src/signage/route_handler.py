# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

import os
import json
from datetime import datetime

import signage.signage_utils as utils
from autoware_adapi_v1_msgs.msg import (
    RouteState,
    OperationModeState,
    MotionState,
    LocalizationInitializationState,
    DoorStatus,
)


class RouteHandler:
    def __init__(
        self,
        node,
        viewController,
        announceController,
        autoware_interface,
        parameter_interface,
        ros_service_interface,
        cvm_interface,
        standing_mode_interface,
    ):
        self._node = node
        self._viewController = viewController
        self._announce_interface = announceController
        self._autoware = autoware_interface
        self._parameter = parameter_interface.parameter
        self._service_interface = ros_service_interface
        self._cvm = cvm_interface
        self._standing_mode = standing_mode_interface
        self._schedule_details = utils.init_ScheduleDetails()
        self._display_details = utils.init_DisplayDetails()
        self._current_task_details = utils.init_CurrentTask()
        self.task_list = utils.init_TaskList()
        self._display_phrase = ""
        self._in_emergency_state = False
        self._emergency_trigger_time = self._node.get_clock().now()
        self._engage_trigger_time = self._node.get_clock().now()
        self._is_stopping = True
        self._is_driving = False
        self._previous_driving_status = False
        self._reach_final = False
        self._pre_door_announce_status = DoorStatus.UNKNOWN
        self._fms_check_time = 0
        self._prev_motion_state = 0
        self._prev_route_state = 0
        self._skip_announce = False
        self._announce_engage = False
        self._in_slow_stop_state = False
        self._in_slowing_state = False
        self._announced_depart = False
        self._announced_arrive = False
        # UC-05: 接近10mの安全配慮アナウンス発話済みフラグ・直近到着停留所 (到着表示用)
        self._announced_arrive_caution = False
        self._arrived_station = ["", ""]
        # UC-04: 停止1回につき停止案内は1回。再発進でリセットする。
        # 初期値 True (disarm) = 未発進では発話しない。最初の実発進で arm される。
        self._stop_announce_executed = True
        self._trigger_external_signage = False
        self._processing_thread = False

        # 立席運行 転倒防止アナウンスの状態 (UC-02/UC-03)
        # 発車警告と急減速・急操舵警告はそれぞれ独立したクールダウンを持つ
        # 再発話抑止・警告表示時間は announce_controller の announce_interval で管理する
        # (VVAS と同方式)。ここでは表示に使う直近の急制動種別のみ保持する。
        self._sudden_warning_type = ""

        self.process_station_list_from_fms()

        self._node.create_timer(0.2, self.route_checker_callback)
        self._node.create_timer(0.2, self.emergency_checker_callback)
        self._node.create_timer(0.2, self.view_mode_callback)
        self._node.create_timer(0.2, self.calculate_time_callback)
        self._node.create_timer(0.2, self.door_status_callback)
        self._node.create_timer(0.2, self.announce_engage_when_starting)
        self._node.create_timer(0.2, self.stop_reason_checker_callback)
        self._node.create_timer(0.2, self.sudden_motion_checker_callback)

    def emergency_checker_callback(self):
        # MRM の状態更新と緊急アナウンスを、責務ごとにヘルパーへ分割して実行する。
        #   - _update_comfortable_stop_state: comfortable stop の減速中/停止後判定
        #   - _update_emergency_state:        緊急停止中かどうかの判定 (_in_emergency_state)
        #   - _announce_emergency:            緊急アナウンスの発話
        if (
            self._parameter.ignore_emergency
            or self._autoware.information.operation_mode == OperationModeState.STOP
        ):
            # Ignore the emergency
            self._in_emergency_state = False
            self._in_slowing_state = False
            self._in_slow_stop_state = False
            return

        current_time = self._node.get_clock().now()
        mrm_behavior = self._autoware.information.mrm_behavior
        in_emergency = mrm_behavior == 12
        # comfortable stop = MRM 挙動が緊急(12)でも通常(1)でもないもの
        in_comfortable_stop = mrm_behavior not in [1, 12]

        self._update_comfortable_stop_state(in_comfortable_stop, current_time)
        # 発話は初回/繰り返し判定に更新前の _in_emergency_state を使うため、
        # 状態更新 (_update_emergency_state) より先に呼ぶ。
        self._announce_emergency(in_emergency, current_time)
        self._update_emergency_state(in_emergency, current_time)

    def _update_comfortable_stop_state(self, in_comfortable_stop, current_time):
        # comfortable stop 中は motion_state から減速中(slowing)/停止後(slow_stop)を判定する。
        # comfortable stop を抜けたら freeze 期間(emergency_ignore_period)経過後に解除する。
        if in_comfortable_stop:
            motion_state = self._autoware.information.motion_state
            self._in_slowing_state = motion_state == MotionState.MOVING
            self._in_slow_stop_state = motion_state == MotionState.STOPPED
            self._emergency_trigger_time = current_time
        elif (
            utils.check_timeout(
                current_time, self._emergency_trigger_time, self._parameter.emergency_ignore_period
            )
            or not self._parameter.freeze_emergency
        ):
            self._in_slowing_state = False
            self._in_slow_stop_state = False

    def _update_emergency_state(self, in_emergency, current_time):
        # 緊急停止中かどうか(_in_emergency_state)を判定する。
        # 緊急に入ったら即 True。抜けたら freeze 期間(emergency_ignore_period)経過後に False。
        if in_emergency:
            self._in_emergency_state = True
        elif (
            utils.check_timeout(
                current_time, self._emergency_trigger_time, self._parameter.emergency_ignore_period
            )
            or not self._parameter.freeze_emergency
        ):
            self._in_emergency_state = False

    def _announce_emergency(self, in_emergency, current_time):
        # 緊急停止中の車内アナウンスを発話する。
        # 初回は "emergency"、以降は emergency_repeat_period ごとに "in_emergency" を繰り返す。
        # 初回/繰り返しの判定には更新前の _in_emergency_state を使う (False = 緊急突入の初回tick)。
        if not in_emergency:
            return

        audio = ""
        if not self._in_emergency_state:
            audio = "emergency"
        elif utils.check_timeout(
            current_time, self._emergency_trigger_time, self._parameter.emergency_repeat_period
        ):
            audio = "in_emergency"

        if audio:
            self._announce_interface.announce_emergency(audio)
            self._emergency_trigger_time = current_time

    def door_status_callback(self):
        door_status = self._autoware.information.door_status
        if self._pre_door_announce_status == door_status:
            # same announce, return
            return

        if door_status == DoorStatus.OPENING:
            # Should able to give warning everytime the door is opening
            self._announce_interface.send_announce("door_open")
        elif door_status == DoorStatus.CLOSING:
            # Should able to give warning everytime the door is closing
            self._announce_interface.send_announce("door_close")

        self._pre_door_announce_status = door_status

    def announce_engage_when_starting(self):
        try:
            if not self._parameter.signage_stand_alone:
                return

            if (
                self._autoware.information.localization_init_state
                == LocalizationInitializationState.UNINITIALIZED
                or self._autoware.information.autoware_control == False
            ):
                self._prev_motion_state = 0
                return

            if (
                self._autoware.information.motion_state
                in [MotionState.STARTING, MotionState.MOVING]
                and self._prev_motion_state == 1
            ):
                # 再発進したので次の停止で停止案内を再度出せるようにする (UC-04)
                self._stop_announce_executed = False
                if self._announce_engage and not self._skip_announce:
                    self._skip_announce = True
                elif utils.check_timeout(
                    self._node.get_clock().now(),
                    self._engage_trigger_time,
                    self._parameter.accept_start,
                ):
                    self._announce_interface.send_announce("restart_engage")
                    self._engage_trigger_time = self._node.get_clock().now()

                if self._autoware.information.motion_state == MotionState.STARTING:
                    self._service_interface.accept_start()

            # Check to see if it has not stopped waiting for start acceptance
            if self._autoware.information.motion_state != MotionState.STARTING:
                self._accept_start_time = self._node.get_clock().now()

            # Send again when stopped in starting state for a certain period of time
            if (
                self._autoware.information.motion_state == MotionState.STARTING
                and utils.check_timeout(
                    self._node.get_clock().now(),
                    self._accept_start_time,
                    self._parameter.accept_start,
                )
            ):
                self._service_interface.accept_start()

            self._prev_motion_state = self._autoware.information.motion_state
        except Exception as e:
            self._node.get_logger().error("not able to play the announce, ERROR: {}".format(str(e)))

    def stop_reason_checker_callback(self):
        # UC-04 (SYS-HMI-04/05/06): 走行中の予定外停止で車内に「停車します」と案内する。
        # 停止種別 (障害物/横断歩道/一般) は車内では区別しないため停止理由は参照しない。
        # e2e プランナでは停止理由 (velocity_factors の behavior) が出ない場合があるため、
        # 停止理由に依存せず運行状態から一時停止を判定する:
        #   AUTONOMOUS 走行中 (route 継続中) にゴール手前で停止 = 障害物/信号/横断歩道等の一時停止。
        #   到着停止 (AUTONOMOUS 離脱 & ゴール近傍) と発進待ちは除外される。
        try:
            if not self._parameter.signage_stand_alone:
                return
            if not self._autoware.information.autoware_control:
                return
            # MRM (緊急停止) 中は停止案内より緊急案内を優先するためスキップ
            if self._in_emergency_state:
                return
            # 発進前 (発進待ち) は disarm 状態。最初の STOPPED->MOVING (実発進) で
            # announce_engage_when_starting が arm (False) する。engage 直後の
            # 「AUTONOMOUS だが未発進で停止中」での誤発話を防ぐ。
            if self._stop_announce_executed:
                return

            info = self._autoware.information
            is_temporary_stop = (
                info.operation_mode == OperationModeState.AUTONOMOUS
                and info.route_state == RouteState.SET
                and info.motion_state == MotionState.STOPPED
                and info.goal_distance > self._parameter.arriving_distance
            )
            if is_temporary_stop:
                if self._announce_interface.in_interval("stop_reason"):
                    return
                self._announce_interface.send_announce("temporary_stop")
                self._announce_interface.set_timeout("stop_reason")
                self._stop_announce_executed = True
        except Exception as e:
            self._node.get_logger().error(
                "not able to check the stop reason, ERROR: {}".format(str(e))
            )

    def process_station_list_from_fms(self, force_update=False):
        try:
            data = json.loads(self._autoware.information.active_schedule)
            if not data:
                self._schedule_details = utils.init_ScheduleDetails()
                self._display_details = utils.init_DisplayDetails()
                self._current_task_details = utils.init_CurrentTask()
                raise Exception("No data from fms")
            elif utils.check_schedule_update(self._schedule_details, data) and not force_update:
                self._fms_check_time = self._node.get_clock().now()
                raise Exception("same schedule, skip")

            self._fms_check_time = self._node.get_clock().now()

            self._schedule_details = utils.update_schedule_details(data)

            self._display_details.route_name = utils.get_route_name(
                data.get("tags", []),
            )

            self.task_list = utils.separate_task_list(data.get("tasks", []))

            if not self.task_list.doing_list and not self.task_list.todo_list:
                self._schedule_details = utils.init_ScheduleDetails()
                self._display_details = utils.init_DisplayDetails()
                self._current_task_details = utils.init_CurrentTask()
                raise Exception("doing_list is not found, skip")

            for task in self.task_list.doing_list:
                self._current_task_details = utils.process_current_task(task)

            if force_update and self._schedule_details.schedule_type != "loop":
                # Currently loop do not provide done list for previous station so we cannot remove the it
                if not self.task_list.done_list:
                    self._display_details.previous_station = ["", ""]
                else:
                    self._display_details.previous_station = (
                        utils.get_previous_station_name_from_fms(self.task_list.done_list)
                    )

            if self._display_details.previous_station == ["", ""] and self.task_list.done_list:
                self._display_details.previous_station = utils.get_previous_station_name_from_fms(
                    self.task_list.done_list
                )

            self._display_details.next_station_list = utils.create_next_station_list(
                self._current_task_details,
                self.task_list.todo_list,
                "fms",
                self._schedule_details.schedule_type,
            )

            # Reset previous station when reach goal
            if self._reach_final and self.task_list.doing_list:
                self._reach_final = False
                self._display_details.previous_station = ["", ""]

            self._fms_check_time = self._node.get_clock().now()
        except Exception as e:
            self._node.get_logger().warning(
                "Unable to get the task from FMS, ERROR: " + str(e), throttle_duration_sec=5
            )

    def arrived_goal(self):
        try:
            # UC-05: 音声は終点・通常停留所とも thank_you で共通 (announce_arrived)。
            # 通常停留所のみ乗客サイネージに到着表示 (arrived) を出す。終点は _reach_final
            # 経由で「終点です」表示になるため到着表示 (set_timeout) は行わない。
            is_final = not self.task_list.todo_list
            arrived_station = self._current_task_details.arrival_station
            self._announce_interface.announce_arrived()
            # 次の接近で再度安全配慮を出せるようリセット
            self._announced_arrive_caution = False

            if self._current_task_details == utils.init_CurrentTask():
                raise Exception("No current task details")

            self._display_details.previous_station = self._current_task_details.departure_station

            if not is_final:
                # 通常停留所: 到着表示(SYS-HMI-07)の表示時間を起算し到着停留所名を保持する
                self._arrived_station = arrived_station
                self._announce_interface.set_timeout("arrived")

            if not self.task_list.todo_list:
                # Reach final station
                self._current_task_details.departure_station = (
                    self._current_task_details.arrival_station
                )
                self._current_task_details.arrival_station = ["", ""]
                self._reach_final = True
                return

            next_task = self.task_list.todo_list.pop(0)
            # Get the next task from todo_list
            self._current_task_details = utils.process_current_task(next_task)

            self._display_details.next_station_list = utils.create_next_station_list(
                self._current_task_details,
                self.task_list.todo_list,
                "local",
                self._schedule_details.schedule_type,
            )
        except Exception as e:
            self._node.get_logger().error("Unable to update the goal, ERROR: " + str(e))

    # ========== Timer Callback =============

    def route_checker_callback(self):
        try:
            if self._autoware.information.operation_mode == OperationModeState.AUTONOMOUS:
                # Check whether the vehicle is move in autonomous
                self._is_driving = True
                self._is_stopping = False
                self._announced_depart = False

                if (
                    not self._trigger_external_signage
                    and self._autoware.information.autoware_control
                ):
                    self._service_interface.trigger_external_signage(True)
                    self._trigger_external_signage = True
                if (
                    not self._announce_engage
                    and self._parameter.signage_stand_alone
                    and self._autoware.information.autoware_control
                ):
                    # UC-02: バス停発車 (STOP->AUTONOMOUS) のタイミング。
                    # 立席モードON時は standing_depart (発進します+転倒防止の注意) のみ再生し、
                    # engage との「発進します」二重発話を避ける。
                    # 立席モードOFF、またはクールダウン等で standing_depart を再生しなかった
                    # 場合は通常の engage (発進します) を再生する。
                    if not self.trigger_depart_warning():
                        self._announce_interface.send_announce("engage")
                    self._service_interface.trigger_external_signage(True)
                    self._trigger_external_signage = True
                    self._announce_engage = True
            elif self._autoware.information.route_state in [RouteState.ARRIVED, RouteState.UNSET]:
                # Check whether the vehicle arrive to goal
                self._is_driving = False
                self._is_stopping = True
                self._skip_announce = False
                self._announce_engage = False
                self._announced_arrive = False

            if (
                self._autoware.information.operation_mode != OperationModeState.AUTONOMOUS
                or self._autoware.information.autoware_control != True
            ) and self._trigger_external_signage:
                self._service_interface.trigger_external_signage(False)
                self._trigger_external_signage = False

            if self._prev_route_state != RouteState.SET:
                if self._autoware.information.route_state == RouteState.SET:
                    self.process_station_list_from_fms(force_update=True)

            if not self._fms_check_time:
                self.process_station_list_from_fms()
            elif utils.check_timeout(
                self._node.get_clock().now(), self._fms_check_time, self._parameter.check_fms_time
            ):
                self.process_station_list_from_fms()

            if self._in_emergency_state:
                return

            if (
                not self._autoware.information.autoware_control
                and self._parameter.ignore_manual_driving
                and self._parameter.set_goal_by_distance
            ):
                if self._autoware.information.goal_distance < self._parameter.goal_distance:
                    self._is_stopping = True
                    self._is_driving = False
                    self._announced_depart = False

            if self._reach_final:
                self._previous_driving_status = False
                return

            if self._is_stopping and self._previous_driving_status:
                self.arrived_goal()
                self._previous_driving_status = False

            if self._is_driving:
                self._previous_driving_status = self._is_driving

            self._prev_route_state = self._autoware.information.route_state
        except Exception as e:
            self._node.get_logger().error("Error unable to check the route: " + str(e))

    def calculate_time_callback(self):
        try:
            # UC-05 (SYS-HMI-07): 接近10mの車内安全配慮は FMS スケジュールに依存しない
            # (_is_driving と goal_distance だけで判定できる) ため、FMS タスク未取得でも
            # 発話できるよう以降の FMS ゲートより前で評価する。
            if (
                self._is_driving
                and 0
                < self._autoware.information.goal_distance
                < self._parameter.arriving_distance
                and not self._announced_arrive_caution
            ):
                self._announce_interface.send_announce("arrive_caution")
                self._announced_arrive_caution = True

            if self._current_task_details == utils.init_CurrentTask():
                return

            remain_minute = utils.get_remain_minute(
                self._current_task_details.depart_time, self._node.get_clock().now().to_msg().sec
            )

            if self._announce_interface.in_interval("arrived"):
                # UC-05: 停止後 announce_interval.arrived の間「‹停留所名›に到着しました」を表示
                self._display_phrase = utils.handle_phrase("arrived", self._arrived_station[0])
            elif self._reach_final:
                # 終点では「終点です」の文言表示は行わない (ユーザー方針 2026-07-14)。
                # is_stopping ブランチに落として発車待ち文言を出さないよう、ここで空表示に固定する。
                self._display_phrase = ""
            elif self._is_stopping:
                # handle text and announce while bus is stopping
                if remain_minute > 2:
                    # display the text with the remaining time for departure
                    self._display_phrase = utils.handle_phrase(
                        "remain_minute", round(remain_minute)
                    )
                else:
                    # the departure time is close (within 1 min), announce going to depart
                    self._display_phrase = utils.handle_phrase("departing")
                    if not self._announced_depart:
                        self._announce_interface.announce_going_to_depart_and_arrive(
                            "going_to_depart"
                        )
                        self._announced_depart = True
            elif self._is_driving:
                # handle text and announce while bus is running
                # 到着予告(表示・音声とも)は going_to_arrive 設定で一括制御する。
                # Phase1方針ではデフォルト除外のため、無効時は「もうすぐ到着」表示も出さない。
                if (
                    self._announce_interface.check_announce_or_not("going_to_arrive")
                    and self._autoware.information.goal_distance < 100
                    and self._autoware.information.goal_distance > 0
                ):
                    # display text and announce if the goal is within 100m
                    self._display_phrase = utils.handle_phrase("arriving")
                    if not self._announced_arrive:
                        self._announce_interface.announce_going_to_depart_and_arrive(
                            "going_to_arrive"
                        )
                        self._announced_arrive = True
                else:
                    self._display_phrase = ""
            else:
                self._display_phrase = ""
        except Exception as e:
            self._node.get_logger().error("Error in getting calculate the time: " + str(e))

    def trigger_depart_warning(self):
        # standing_depart を再生したら True を返す (発進アナウンスを兼ねるため呼び出し側で利用)
        # 立席運用モードOFF時は提供しない (SYS2-UC02-02)
        if not self._standing_mode.is_standing_mode:
            return False
        # SYS2-UC02-05: インターバル(5s)中の再発車はスキップ (キューに積まない)
        # スキップ分はログに残さず、実際に発話するときのみ記録する (UC-03 と方針統一)
        if self._announce_interface.in_interval("standing_depart"):
            return False
        self._announce_interface.set_timeout("standing_depart")
        # 提供結果 (played/queued/failed 等) は send_announce 側で記録される
        self._announce_interface.send_announce("standing_depart")
        return True

    def trigger_sudden_warning(self, warning_type, reasons):
        # SYS2-UC03-06: インターバル(10s)中の同種イベントはスキップ (キューに積まない)
        # スキップ分はログに残さず、実際に発話するときのみ記録する
        if self._announce_interface.in_interval("standing_sudden"):
            return
        self._announce_interface.set_timeout("standing_sudden")
        self._sudden_warning_type = warning_type
        # 閾値チューニング用: どの値がどの閾値を超えて発話したかを記録する
        # (提供結果 played/queued/failed 等は send_announce 側で別途記録される)
        self._node.get_logger().info(
            "UC-03: sudden motion triggered [{}: {}]".format(
                warning_type, ", ".join(reasons)
            )
        )
        self._announce_interface.send_announce("standing_" + warning_type)

    def sudden_motion_checker_callback(self):
        # UC-03: 急減速・急操舵の事後検知。立席運用モードON・走行中・MRM非作動のときのみ評価する。
        try:
            if not self._standing_mode.is_standing_mode:
                return
            if not self._is_driving:
                return
            # MRMが最優先。MRM(緊急停止/快適減速)作動中は本警告を出さず、
            # 直前にトリガー済みで再生中の立席アナウンスがあれば停止する (SYS2-UC03-01)
            if self._in_emergency_state or self._in_slowing_state or self._in_slow_stop_state:
                self._announce_interface.stop_standing_announce()
                return

            info = self._autoware.information
            param = self._parameter

            # 閾値を超えた項目を記録する (どの値が閾値以上だったかをログに残す)
            decel_reasons = []
            if info.longitudinal_acceleration <= -param.sudden_decel_threshold:
                decel_reasons.append(
                    "accel={:.3f} <= -{:.3f}".format(
                        info.longitudinal_acceleration, param.sudden_decel_threshold
                    )
                )
            # jerk は減速方向 (負値) のみを対象とする。加速方向 (正のjerk) は
            # 発進・加速時の挙動で転倒リスクが低いため対象外とし、jerk <= -閾値 で判定する。
            if info.longitudinal_jerk <= -param.sudden_decel_jerk_threshold:
                decel_reasons.append(
                    "jerk={:.3f} <= -{:.3f}".format(
                        info.longitudinal_jerk, param.sudden_decel_jerk_threshold
                    )
                )

            # 急操舵は横加速度・横ジャークの「いずれか」超過で判定する (SYS2-UC03-01)。
            turn_reasons = []
            # 横加速度: metric をそのまま使用 (検討案 最大横加速度 >= 0.8)
            if info.lateral_acceleration_abs >= param.sudden_lateral_accel_threshold:
                turn_reasons.append(
                    "lateral_accel_abs={:.3f} >= {:.3f}".format(
                        info.lateral_acceleration_abs, param.sudden_lateral_accel_threshold
                    )
                )
            # 横ジャーク: metric 未配信のため計算で近似する。横加速 a_y ≒ v^2*δ/L の
            # 時間微分 (速度一定近似) より 横ジャーク ≒ v^2 * steering_rate / wheel_base。
            # 左右どちらの向きでも転倒リスクとなるため絶対値で閾値判定する (検討案 最大横ジャーク >= 0.5)。
            if info.wheel_base > 0.0:
                lateral_jerk = info.velocity**2 * info.steering_rate / info.wheel_base
                if abs(lateral_jerk) >= param.sudden_lateral_jerk_threshold:
                    turn_reasons.append(
                        "lateral_jerk={:.3f} (v={:.2f}, steering_rate={:.3f}, L={:.2f}) >= {:.3f}".format(
                            lateral_jerk,
                            info.velocity,
                            info.steering_rate,
                            info.wheel_base,
                            param.sudden_lateral_jerk_threshold,
                        )
                    )

            is_sudden_decel = bool(decel_reasons)
            is_sudden_turn = bool(turn_reasons)

            if not is_sudden_decel and not is_sudden_turn:
                return

            if is_sudden_decel and is_sudden_turn:
                warning_type = "sudden_stop_turn"
            elif is_sudden_decel:
                warning_type = "sudden_stop"
            else:
                warning_type = "sudden_turn"

            self.trigger_sudden_warning(warning_type, decel_reasons + turn_reasons)
        except Exception as e:
            self._node.get_logger().error("Error in sudden motion checker: " + str(e))

    def view_mode_callback(self):
        try:
            self._viewController.clock_string = datetime.now().strftime("%H:%M")
            self._viewController.route_name = self._display_details.route_name
            self._viewController.departure_station_name = (
                self._current_task_details.departure_station
            )
            self._viewController.arrival_station_name = self._current_task_details.arrival_station
            self._viewController.previous_station_name = self._display_details.previous_station
            self._viewController.next_station_list = self._display_details.next_station_list
            self._viewController.display_phrase = self._display_phrase

            cvm_override = self._cvm.get_view_mode_override()

            if (
                self._autoware.is_disconnected
                and not self._parameter.ignore_disconnected
                and not self._parameter.ignore_emergency
            ):
                view_mode = "disconnected"
            elif cvm_override is not None:
                view_mode = cvm_override
            elif (
                not self._autoware.information.autoware_control
                and not self._parameter.ignore_manual_driving
            ):
                view_mode = "manual_driving"
            elif self._in_emergency_state:
                # comfortable stop (slowing/slow_stop) と揃えて、緊急停止も減速中と停止後で
                # 表示を固定する。減速中 (emergency_slowing) は EmergencyStop、
                # 停止後 (emergency_stopped) は EmergencyStopping を表示する。
                if self._autoware.information.motion_state == MotionState.STOPPED:
                    view_mode = "emergency_stopped"
                else:
                    view_mode = "emergency_slowing"
            elif self._in_slowing_state:
                view_mode = "slowing"
            elif self._in_slow_stop_state:
                view_mode = "slow_stop"
            elif self._announce_interface.in_interval("standing_sudden"):
                # UC-03: 急減速・急操舵警告 (MRMの直下・発車警告より上位)
                # 表示時間＝再発話抑止インターバル (announce_interval.standing_sudden) で連動
                view_mode = "standing_sudden_warning"
            elif self._announce_interface.in_interval("standing_depart"):
                # UC-02: 発車時警告 (表示時間＝announce_interval.standing_depart)
                view_mode = "standing_depart_warning"
            elif self._announce_interface.in_interval("arrived"):
                # UC-05 (SYS-HMI-07): 到着直後 announce_interval.arrived 秒間は
                # 「‹停留所名›に到着しました」を専用画面で表示する (stopping より優先)
                view_mode = "arrived"
            elif self._is_stopping and self._current_task_details.departure_station != ["", ""]:
                view_mode = "stopping"
            elif self._is_driving and self._current_task_details.arrival_station != ["", ""]:
                view_mode = "driving"
            elif self._is_driving:
                view_mode = "auto_driving"
            elif self._current_task_details.departure_station != ["", ""]:
                view_mode = "stopping"
            else:
                view_mode = "out_of_service"
                self._announced_depart = False

            if view_mode == "standing_sudden_warning":
                self._viewController.standing_warning_type = self._sudden_warning_type
            elif view_mode == "standing_depart_warning":
                self._viewController.standing_warning_type = "depart"
            else:
                self._viewController.standing_warning_type = ""

            self._cvm.set_current_view_mode(view_mode)
            self._viewController.cvm_display_mode_id = (
                self._cvm.get_display_mode_id_override() or ""
            )
            self._viewController.view_mode = view_mode
        except Exception as e:
            self._node.get_logger().error("Error in updating the view mode: " + str(e))
