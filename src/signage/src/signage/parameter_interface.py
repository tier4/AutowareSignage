# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

from dataclasses import dataclass, field


@dataclass
class SignageParameter:
    debug_mode: bool = False
    use_external_signage: bool = False
    signage_stand_alone: bool = False
    ignore_manual_driving: bool = False
    ignore_disconnected: bool = False
    ignore_emergency: bool = False
    set_goal_by_distance: bool = False
    freeze_emergency: bool = True
    goal_distance: float = 1.0
    check_fms_time: float = 5.0
    accept_start: float = 5.0
    arriving_distance: float = 10.0  # UC-05 到着接近の車内安全配慮しきい値 (m)
    emergency_ignore_period: float = 5.0
    emergency_repeat_period: float = 180.0
    monitor_width: int = 1920
    monitor_height: int = 540
    cvm_device_id: str = "in_vehicle_signage"
    standing_mode_default: bool = False
    sudden_decel_threshold: float = 0.8
    sudden_decel_jerk_threshold: float = 0.6
    # 急操舵は横加速度 (metric) と横ジャーク (計算) のいずれか超過で判定する (SYS2-UC03-01「いずれか」)
    sudden_lateral_accel_threshold: float = 0.8
    sudden_lateral_jerk_threshold: float = 0.5
    # 横ジャーク計算 (v^2 * steering_rate / wheel_base) 用のフォールバック値。
    # 起動時に /api/vehicle/dimensions サービスから取得でき次第そちらで上書きされる。
    wheel_base: float = 2.75
    # MRM (/api/fail_safe/mrm_state) の値域判定・緊急判定値 (SYS2-ERR-01 値域逸脱)。
    # behavior の値体系は pilot-auto のバージョンで変わるため config で可変にする。
    # state / behavior が有効値域外の場合は MRM 発生有無を不明として扱いリセットする。
    mrm_valid_states: list = field(default_factory=lambda: [1, 2, 3, 4])
    mrm_valid_behaviors: list = field(default_factory=lambda: [1, 2, 3, 4, 12])
    mrm_none_behavior: int = 1  # MRM 非作動 (NONE) を表す behavior 値
    mrm_emergency_behaviors: list = field(default_factory=lambda: [12])


@dataclass
class AnnounceParameter:
    emergency: bool = True
    restart_engage: bool = True
    door_close: bool = True
    door_open: bool = True
    engage: bool = True
    arrived: bool = True
    thank_you: bool = True
    in_emergency: bool = True
    going_to_depart: bool = True
    going_to_arrive: bool = True
    arrive_caution: bool = True
    temporary_stop: bool = True
    standing_depart: bool = True
    standing_sudden: bool = True


@dataclass
class AnnounceIntervalParameter:
    # 発話後にその期間だけ再発話を抑止する (秒)。VVAS の announce_interval と同方式。
    stop_reason: float = 20.0  # UC-04 停止案内 (SYS-HMI-04/05/06)
    arrived: float = 5.0  # UC-05 到着表示の表示秒数 (SYS-HMI-07)
    # standing_sudden は急減速/急操舵/複合の3種で共有する (VVAS の turn_signal と同様)。
    standing_depart: float = 5.0  # UC-02 発車警告 (SYS2-UC02-01)
    standing_sudden: float = 10.0  # UC-03 急減速・急操舵警告 (SYS2-UC03-06)


class ParameterInterface:
    def __init__(self, node):
        self.parameter = SignageParameter()
        self.announce_settings = AnnounceParameter()
        self.announce_interval = AnnounceIntervalParameter()

        node.declare_parameter("debug_mode", False)
        node.declare_parameter("use_external_signage", False)
        node.declare_parameter("signage_stand_alone", False)
        node.declare_parameter("ignore_disconnected", False)
        node.declare_parameter("ignore_manual_driving", False)
        node.declare_parameter("freeze_emergency", True)
        node.declare_parameter("check_fms_time", 5.0)
        node.declare_parameter("accept_start", 5.0)
        node.declare_parameter("arriving_distance", 10.0)
        node.declare_parameter("ignore_emergency_stoppped", False)
        node.declare_parameter("set_goal_by_distance", False)
        node.declare_parameter("goal_distance", 1.0)
        node.declare_parameter("emergency_ignore_period", 5.0)
        node.declare_parameter("emergency_repeat_period", 180.0)
        node.declare_parameter("monitor_width", 1920)
        node.declare_parameter("monitor_height", 540)
        node.declare_parameter("cvm_device_id", "in_vehicle_signage")
        node.declare_parameter("standing_mode_default", False)
        node.declare_parameter("sudden_decel_threshold", 0.8)
        node.declare_parameter("sudden_decel_jerk_threshold", 0.6)
        node.declare_parameter("sudden_lateral_accel_threshold", 0.8)
        node.declare_parameter("sudden_lateral_jerk_threshold", 0.5)
        node.declare_parameter("wheel_base", 2.75)
        node.declare_parameter("mrm.valid_states", [1, 2, 3, 4])
        node.declare_parameter("mrm.valid_behaviors", [1, 2, 3, 4, 12])
        node.declare_parameter("mrm.none_behavior", 1)
        node.declare_parameter("mrm.emergency_behaviors", [12])

        self.parameter.debug_mode = (
            node.get_parameter("debug_mode").get_parameter_value().bool_value
        )
        self.parameter.use_external_signage = (
            node.get_parameter("use_external_signage").get_parameter_value().bool_value
        )
        self.parameter.signage_stand_alone = (
            node.get_parameter("signage_stand_alone").get_parameter_value().bool_value
        )
        self.parameter.ignore_disconnected = (
            node.get_parameter("ignore_disconnected").get_parameter_value().bool_value
        )
        self.parameter.ignore_manual_driving = (
            node.get_parameter("ignore_manual_driving").get_parameter_value().bool_value
        )
        self.parameter.freeze_emergency = (
            node.get_parameter("freeze_emergency").get_parameter_value().bool_value
        )
        self.parameter.check_fms_time = (
            node.get_parameter("check_fms_time").get_parameter_value().double_value
        )
        self.parameter.accept_start = (
            node.get_parameter("accept_start").get_parameter_value().double_value
        )
        self.parameter.arriving_distance = (
            node.get_parameter("arriving_distance").get_parameter_value().double_value
        )
        self.parameter.ignore_emergency = (
            node.get_parameter("ignore_emergency_stoppped").get_parameter_value().bool_value
        )
        self.parameter.set_goal_by_distance = (
            node.get_parameter("set_goal_by_distance").get_parameter_value().bool_value
        )
        self.parameter.goal_distance = (
            node.get_parameter("goal_distance").get_parameter_value().double_value
        )
        self.parameter.emergency_ignore_period = (
            node.get_parameter("emergency_ignore_period").get_parameter_value().double_value
        )
        self.parameter.emergency_repeat_period = (
            node.get_parameter("emergency_repeat_period").get_parameter_value().double_value
        )
        self.parameter.monitor_width = (
            node.get_parameter("monitor_width").get_parameter_value().integer_value
        )
        self.parameter.monitor_height = (
            node.get_parameter("monitor_height").get_parameter_value().integer_value
        )
        self.parameter.cvm_device_id = (
            node.get_parameter("cvm_device_id").get_parameter_value().string_value
        )
        self.parameter.standing_mode_default = (
            node.get_parameter("standing_mode_default").get_parameter_value().bool_value
        )
        self.parameter.sudden_decel_threshold = (
            node.get_parameter("sudden_decel_threshold").get_parameter_value().double_value
        )
        self.parameter.sudden_decel_jerk_threshold = (
            node.get_parameter("sudden_decel_jerk_threshold").get_parameter_value().double_value
        )
        self.parameter.sudden_lateral_accel_threshold = (
            node.get_parameter("sudden_lateral_accel_threshold").get_parameter_value().double_value
        )
        self.parameter.sudden_lateral_jerk_threshold = (
            node.get_parameter("sudden_lateral_jerk_threshold").get_parameter_value().double_value
        )
        self.parameter.wheel_base = (
            node.get_parameter("wheel_base").get_parameter_value().double_value
        )
        # integer_array_value は array('i', ...) を返すため list へ変換して保持する
        self.parameter.mrm_valid_states = list(
            node.get_parameter("mrm.valid_states").get_parameter_value().integer_array_value
        )
        self.parameter.mrm_valid_behaviors = list(
            node.get_parameter("mrm.valid_behaviors").get_parameter_value().integer_array_value
        )
        self.parameter.mrm_none_behavior = (
            node.get_parameter("mrm.none_behavior").get_parameter_value().integer_value
        )
        self.parameter.mrm_emergency_behaviors = list(
            node.get_parameter("mrm.emergency_behaviors").get_parameter_value().integer_array_value
        )
        # 設定の自己整合チェック: 緊急/NONE の判定値が有効値域に無いと、その値は
        # 「値域逸脱」としてリセットされ緊急停止表示が永久に出なくなる。設定ミスで
        # 安全側の表示が落ちるのを避けるため、不足分は有効値へ補いログに残す。
        missing_behaviors = [
            behavior
            for behavior in [self.parameter.mrm_none_behavior]
            + self.parameter.mrm_emergency_behaviors
            if behavior not in self.parameter.mrm_valid_behaviors
        ]
        if missing_behaviors:
            node.get_logger().error(
                "mrm.valid_behaviors {} does not contain {}, add them automatically".format(
                    self.parameter.mrm_valid_behaviors, missing_behaviors
                )
            )
            self.parameter.mrm_valid_behaviors += missing_behaviors

        node.declare_parameter("announce.emergency", True)
        node.declare_parameter("announce.restart_engage", True)
        node.declare_parameter("announce.door_close", True)
        node.declare_parameter("announce.door_open", True)
        node.declare_parameter("announce.engage", True)
        node.declare_parameter("announce.thank_you", True)
        node.declare_parameter("announce.in_emergency", True)
        node.declare_parameter("announce.going_to_depart", True)
        node.declare_parameter("announce.going_to_arrive", True)
        node.declare_parameter("announce.arrive_caution", True)
        node.declare_parameter("announce.temporary_stop", True)
        node.declare_parameter("announce.standing_depart", True)
        node.declare_parameter("announce.standing_sudden", True)

        announce_prefix = node.get_parameters_by_prefix("announce")

        for key in announce_prefix.keys():
            setattr(
                self.announce_settings,
                key,
                announce_prefix[key].get_parameter_value().bool_value,
            )

        node.declare_parameter("announce_interval.stop_reason", 20.0)
        node.declare_parameter("announce_interval.arrived", 5.0)
        node.declare_parameter("announce_interval.standing_depart", 5.0)
        node.declare_parameter("announce_interval.standing_sudden", 10.0)

        announce_interval_prefix = node.get_parameters_by_prefix("announce_interval")

        for key in announce_interval_prefix.keys():
            setattr(
                self.announce_interval,
                key,
                announce_interval_prefix[key].get_parameter_value().double_value,
            )
