# !/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rclpy
from dataclasses import dataclass, field
from autoware_adapi_v1_msgs.msg import (
    RouteState,
    Route,
    MrmState,
    OperationModeState,
    MotionState,
    LocalizationInitializationState,
    VelocityFactorArray,
    VehicleKinematics,
    Heartbeat,
)
from std_msgs.msg import String
import signage.signage_utils as utils
from tier4_external_api_msgs.msg import DoorStatus

DISCONNECT_THRESHOLD = 2


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
    # UC-04: 停止種別判定に使用する velocity_factors (VelocityFactor のリスト)
    velocity_factors: list = field(default_factory=list)


class AutowareInterface:
    def __init__(self, node, parameter_interface):
        self._node = node
        self.information = AutowareInformation()
        self._parameter = parameter_interface.parameter
        self.is_disconnected = False
        # 現在のルートのゴール座標 (map系)。ルート未設定時は None
        self._goal_position = None

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
        # /autoware_api/utils/path_distance_calculator/distance が廃止予定のため、
        # ゴール姿勢 (/api/routing/route) と自車位置 (/api/vehicle/kinematics) から
        # ゴールまでの直線距離を算出して goal_distance を求める。
        # ゴール直前がカーブ / 経路がゴール近傍を通過する場合は実際の経路長と乖離するが、
        # ゴール近傍 (<100m) の到着判定用途としては許容範囲とする。
        self._sub_route = node.create_subscription(
            Route,
            "/api/routing/route",
            self.sub_route_callback,
            api_qos,
        )
        self._sub_kinematics = node.create_subscription(
            VehicleKinematics,
            "/api/vehicle/kinematics",
            self.sub_kinematics_callback,
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
        self._sub_velocity_factor = node.create_subscription(
            VelocityFactorArray,
            "/api/planning/velocity_factors",
            self.sub_velocity_factor_callback,
            sub_qos,
        )
        self._sub_heartbeat = node.create_subscription(
            Heartbeat,
            "/api/system/heartbeat",
            self.sub_heartbeat_callback,
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

    def sub_route_callback(self, msg):
        try:
            if msg.data:
                self._goal_position = msg.data[0].goal.position
            else:
                # ルートがクリアされた場合はゴール距離をデフォルトへ戻す
                self._goal_position = None
                self.information.goal_distance = 1000.0
        except Exception as e:
            self._node.get_logger().error("Unable to get the route goal, ERROR: " + str(e))

    def sub_kinematics_callback(self, msg):
        try:
            if self._goal_position is None:
                return
            ego = msg.pose.pose.pose.position
            self.information.goal_distance = math.hypot(
                self._goal_position.x - ego.x, self._goal_position.y - ego.y
            )
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

    def sub_velocity_factor_callback(self, msg):
        try:
            self.information.velocity_factors = msg.factors
        except Exception as e:
            self._node.get_logger().error("Unable to get the velocity factors, ERROR: " + str(e))

    def sub_heartbeat_callback(self, msg):
        try:
            self._autoware_connection_time = self._node.get_clock().now()
        except Exception as e:
            self._node.get_logger().error("Unable to get the heartbeat, ERROR: " + str(e))
