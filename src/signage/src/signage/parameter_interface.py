# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

from dataclasses import dataclass


@dataclass
class SignageParameter:
    signage_stand_alone: bool = False
    ignore_manual_driving: bool = False
    ignore_emergency: bool = False
    set_goal_by_distance: bool = False
    goal_distance: float = 1.0
    check_fms_time: float = 5.0
    accept_start: float = 5.0
    emergency_repeat_period: float = 180.0
    monitor_width: int = 1920
    monitor_height: int = 540

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

class ParameterInterface:
    def __init__(self, node):
        self.parameter = SignageParameter()
        self.announce_settings = AnnounceParameter()

        node.declare_parameter("signage_stand_alone", False)
        node.declare_parameter("ignore_manual_driving", False)
        node.declare_parameter("check_fms_time", 5.0)
        node.declare_parameter("accept_start", 5.0)
        node.declare_parameter("ignore_emergency_stoppped", False)
        node.declare_parameter("set_goal_by_distance", False)
        node.declare_parameter("goal_distance", 1.0)
        node.declare_parameter("emergency_repeat_period", 180.0)
        node.declare_parameter("monitor_width", 1920)
        node.declare_parameter("monitor_height", 540)

        node.declare_parameter("emergency", True)
        node.declare_parameter("restart_engage", True)
        node.declare_parameter("door_close", True)
        node.declare_parameter("door_open", True)
        node.declare_parameter("engage", True)
        node.declare_parameter("thank_you", True)
        node.declare_parameter("in_emergency", True)
        node.declare_parameter("going_to_depart", True)
        node.declare_parameter("going_to_arrive", True)


        self.parameter.signage_stand_alone = (
            node.get_parameter("signage_stand_alone").get_parameter_value().bool_value
        )
        self.parameter.ignore_manual_driving = (
            node.get_parameter("ignore_manual_driving").get_parameter_value().bool_value
        )
        self.parameter.check_fms_time = (
            node.get_parameter("check_fms_time").get_parameter_value().double_value
        )
        self.parameter.accept_start = (
            node.get_parameter("accept_start").get_parameter_value().double_value
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
        self.parameter.emergency_repeat_period = (
            node.get_parameter("emergency_repeat_period").get_parameter_value().double_value
        )
        self.parameter.monitor_width = (
            node.get_parameter("monitor_width").get_parameter_value().integer_value
        )
        self.parameter.monitor_height = (
            node.get_parameter("monitor_height").get_parameter_value().integer_value
        )

        self.announce_settings.emergency = (
            node.get_parameter("emergency").get_parameter_value().bool_value
        )
        self.announce_settings.restart_engage = (
            node.get_parameter("restart_engage").get_parameter_value().bool_value
        )
        self.announce_settings.door_close = (
            node.get_parameter("door_close").get_parameter_value().bool_value
        )
        self.announce_settings.door_open = (
            node.get_parameter("door_open").get_parameter_value().bool_value
        )
        self.announce_settings.engage = (
            node.get_parameter("engage").get_parameter_value().bool_value
        )
        self.announce_settings.thank_you = (
            node.get_parameter("thank_you").get_parameter_value().bool_value
        )
        self.announce_settings.in_emergency = (
            node.get_parameter("in_emergency").get_parameter_value().bool_value
        )
        self.announce_settings.going_to_depart = (
            node.get_parameter("going_to_depart").get_parameter_value().bool_value
        )
        self.announce_settings.going_to_arrive = (
            node.get_parameter("going_to_arrive").get_parameter_value().bool_value
        )

