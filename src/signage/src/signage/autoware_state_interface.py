# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from dataclasses import dataclass
from autoware_adapi_v1_msgs.msg import RouteState, MrmState, OperationModeState
from tier4_debug_msgs.msg import Float64Stamped
from tier4_api_msgs.msg import AwapiVehicleStatus
from tier4_external_api_msgs.msg import DoorStatus


@dataclass
class AutowareInformation:
    autoware_control: bool = False
    operation_mode: int = 0
    mrm_behavior: int = 0
    route_state: int = 0
    turn_signal: int = 0
    velocity: float = 0.0
    door_status: int = 0
    goal_distance: float = 1000.0


class AutowareStateInterface:
    def __init__(self, node):
        self._node = node
        self.information = AutowareInformation()

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
            sub_qos,
        )
        self._sub_vehicle_state = node.create_subscription(
            AwapiVehicleStatus,
            "/awapi/vehicle/get/status",
            self.sub_vehicle_state_callback,
            sub_qos,
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

    def sub_vehicle_state_callback(self, msg):
        try:
            self.information.turn_signal = msg.turn_signal
            self.information.velocity = msg.velocity
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle state, ERROR: " + str(e))

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
