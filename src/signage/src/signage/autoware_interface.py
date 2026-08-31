# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from dataclasses import dataclass
from typing import Optional
from autoware_adapi_v1_msgs.msg import (
    RouteState,
    MrmState,
    OperationModeState,
    MotionState,
    LocalizationInitializationState,
    Heartbeat,
)
from std_msgs.msg import String
import signage.signage_utils as utils
from autoware_internal_debug_msgs.msg import Float64Stamped
from tier4_external_api_msgs.msg import DoorStatus
from tier4_v2x_msgs.msg import BusStopSignageInfoArray

DISCONNECT_THRESHOLD = 2
SCHEDULE_SOURCE_FMS = "fms"
SCHEDULE_SOURCE_V2X = "v2x"


@dataclass
class AutowareInformation:
    autoware_control: bool = False
    operation_mode: int = 0
    mrm_behavior: int = 0
    route_state: int = 0
    door_status: int = 0
    goal_distance: float = 1000.0
    motion_state: int = 0
    localization_init_state: int = 0
    active_schedule: str = ""
    schedule_source: str = SCHEDULE_SOURCE_FMS
    bus_stop_signage_info: Optional[BusStopSignageInfoArray] = None


class AutowareInterface:
    def __init__(self, node, parameter_interface):
        self._node = node
        self.information = AutowareInformation()
        self._parameter = parameter_interface.parameter
        self.is_disconnected = False

        self.information.schedule_source = self._parameter.default_schedule_source

        sub_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.SYSTEM_DEFAULT,
        )
        api_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._sub_operation_mode = node.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self.sub_operation_mode_callback,
            api_qos,
        )
        self._sub_routing_state = node.create_subscription(
            RouteState,
            "/api/routing/state",
            self.sub_routing_state_callback,
            api_qos,
        )
        self._sub_mrm = node.create_subscription(
            MrmState,
            "/api/fail_safe/mrm_state",
            self.sub_mrm_callback,
            api_qos,
        )
        self._sub_vehicle_door = node.create_subscription(
            DoorStatus, "/api/external/get/door", self.sub_vehicle_door_callback, sub_qos
        )
        self._sub_path_distance = node.create_subscription(
            Float64Stamped,
            "/autoware_api/utils/path_distance_calculator/distance",
            self.sub_path_distance_callback,
            sub_qos,
        )
        self._sub_motion_state = node.create_subscription(
            MotionState, "/api/motion/state", self.sub_motion_state_callback, api_qos
        )
        self._sub_localiztion_initializtion_state = node.create_subscription(
            LocalizationInitializationState,
            "/api/localization/initialization_state",
            self.sub_localization_initialization_state_callback,
            api_qos,
        )
        self._sub_active_schedule = node.create_subscription(
            String,
            "/signage/active_schedule",
            self.sub_active_schedule_callback,
            sub_qos,
        )
        self._sub_heartbeat = node.create_subscription(
            Heartbeat,
            "/api/system/heartbeat",
            self.sub_heartbeat_callback,
            sub_qos,
        )
        self._sub_schedule_source = node.create_subscription(
            String,
            self._parameter.schedule_source_topic,
            self.sub_schedule_source_callback,
            sub_qos,
        )
        self._sub_bus_stop_signage_info = node.create_subscription(
            BusStopSignageInfoArray,
            self._parameter.bus_stop_signage_info_topic,
            self.sub_bus_stop_signage_info_callback,
            sub_qos,
        )
        if not self._parameter.debug_mode:
            self._autoware_connection_time = self._node.get_clock().now()
            self._node.create_timer(1, self.reset_timer)

    def reset_timer(self):
        if utils.check_timeout(
            self._node.get_clock().now(), self._autoware_connection_time, DISCONNECT_THRESHOLD
        ):
            self.information.mrm_behavior = MrmState.NONE
            self._node.get_logger().error(
                "Autoware disconnected", throttle_duration_sec=DISCONNECT_THRESHOLD
            )
            self.is_disconnected = True
        else:
            self.is_disconnected = False

    def sub_operation_mode_callback(self, msg):
        try:
            self.information.autoware_control = msg.is_autoware_control_enabled
            self.information.operation_mode = msg.mode
        except Exception as e:
            self._node.get_logger().error("Unable to get the operation mode, ERROR: " + str(e))

    def sub_routing_state_callback(self, msg):
        try:
            self.information.route_state = msg.state
        except Exception as e:
            self._node.get_logger().error("Unable to get the routing state, ERROR: " + str(e))

    def sub_mrm_callback(self, msg):
        try:
            self.information.mrm_behavior = msg.behavior
        except Exception as e:
            self._node.get_logger().error("Unable to get the mrm behavior, ERROR: " + str(e))

    def sub_vehicle_door_callback(self, msg):
        try:
            self.information.door_status = msg.status
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle door status, ERROR: " + str(e))

    def sub_path_distance_callback(self, msg):
        try:
            self.information.goal_distance = msg.data
        except Exception as e:
            self._node.get_logger().error("Unable to get the goal distance, ERROR: " + str(e))

    def sub_motion_state_callback(self, msg):
        try:
            self.information.motion_state = msg.state
        except Exception as e:
            self._node.get_logger().error("Unable to get the motion state, ERROR: " + str(e))

    def sub_localization_initialization_state_callback(self, msg):
        try:
            self.information.localization_init_state = msg.state
        except Exception as e:
            self._node.get_logger().error(
                "Unable to get the localization init state, ERROR: " + str(e)
            )

    def sub_active_schedule_callback(self, msg):
        try:
            self.information.active_schedule = msg.data
        except Exception as e:
            self._node.get_logger().error("Unable to get the active schedule, ERROR: " + str(e))

    def sub_heartbeat_callback(self, msg):
        try:
            self._autoware_connection_time = self._node.get_clock().now()
        except Exception as e:
            self._node.get_logger().error("Unable to get the heartbeat, ERROR: " + str(e))

    def sub_schedule_source_callback(self, msg):
        try:
            source = msg.data.strip().lower()
            if source not in (SCHEDULE_SOURCE_FMS, SCHEDULE_SOURCE_V2X):
                self._node.get_logger().warning(
                    "Unknown schedule source '{}', expected '{}' or '{}'".format(
                        msg.data, SCHEDULE_SOURCE_FMS, SCHEDULE_SOURCE_V2X
                    ),
                    throttle_duration_sec=5,
                )
                return
            if self.information.schedule_source != source:
                self._node.get_logger().info(
                    "Schedule source switched to '{}'".format(source)
                )
            self.information.schedule_source = source
        except Exception as e:
            self._node.get_logger().error(
                "Unable to get the schedule source, ERROR: " + str(e)
            )

    def sub_bus_stop_signage_info_callback(self, msg):
        try:
            self.information.bus_stop_signage_info = msg
        except Exception as e:
            self._node.get_logger().error(
                "Unable to get the bus stop signage info, ERROR: " + str(e)
            )

    def is_v2x_schedule_source(self):
        return self.information.schedule_source == SCHEDULE_SOURCE_V2X
