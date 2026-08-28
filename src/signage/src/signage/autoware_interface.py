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
    VehicleKinematics,
    Heartbeat,
)
from autoware_adapi_v1_msgs.srv import GetVehicleDimensions
from std_msgs.msg import String
import signage.signage_utils as utils
from tier4_external_api_msgs.msg import DoorStatus
from tier4_metric_msgs.msg import MetricArray

DISCONNECT_THRESHOLD = 2

# /control/control_evaluator/metrics の metric名 -> AutowareInformation の属性名
# (UC-03: 急減速・急操舵判定に使用)
CONTROL_METRIC_ATTR_MAP = {
    "acceleration": "longitudinal_acceleration",
    "jerk": "longitudinal_jerk",
    "lateral_acceleration_abs": "lateral_acceleration_abs",
    "steering_angle_abs": "steering_angle_abs",
    "steering_rate": "steering_rate",
    "steering_acceleration": "steering_acceleration",
}


@dataclass
class AutowareInformation:
    autoware_control: bool = False
    operation_mode: int = 0
    # 起動時 (MRM メッセージ未受信) は MRM 非作動として扱う (NORMAL / NONE)
    mrm_state: int = 1  # MrmState.NORMAL
    mrm_behavior: int = 1  # MrmState.NONE
    route_state: int = 0
    door_status: int = 0
    goal_distance: float = 1000.0
    motion_state: int = 0
    localization_init_state: int = 0
    active_schedule: str = ""
    # UC-04: 停止種別判定に使用する velocity_factors (VelocityFactor のリスト)
    velocity_factors: list = field(default_factory=list)
    longitudinal_acceleration: float = 0.0
    longitudinal_jerk: float = 0.0
    lateral_acceleration_abs: float = 0.0
    steering_angle_abs: float = 0.0
    steering_rate: float = 0.0
    steering_acceleration: float = 0.0
    velocity: float = 0.0  # 縦速度 [m/s] (横ジャーク計算に使用)
    wheel_base: float = 2.75  # ホイールベース [m] (起動時に dimensions サービスで上書き)


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
        self._sub_heartbeat = node.create_subscription(
            Heartbeat,
            "/api/system/heartbeat",
            self.sub_heartbeat_callback,
            sub_qos,
        )
        self._sub_control_metrics = node.create_subscription(
            MetricArray,
            "/control/control_evaluator/metrics",
            self.sub_control_metrics_callback,
            sub_qos,
        )
        # wheelbase は横ジャーク計算 (v^2 * steering_rate / wheel_base) に使う静的値。
        # /api/vehicle/dimensions はトピックではなくサービスのため、起動時に1回だけ取得する。
        # signage 起動時にサービスがまだ立ち上がっていないことがあるので、利用可能になるまで
        # タイマーでリトライし、取得できるまでは param のフォールバック値を使う。
        self.information.wheel_base = self._parameter.wheel_base
        self._dimensions_client = node.create_client(
            GetVehicleDimensions, "/api/vehicle/dimensions"
        )
        self._dimensions_future = None
        self._dimensions_timer = node.create_timer(2.0, self.fetch_vehicle_dimensions)
        if not self._parameter.debug_mode:
            self._autoware_connection_time = self._node.get_clock().now()
            self._node.create_timer(1, self.reset_timer)

    def reset_timer(self):
        if utils.check_timeout(
            self._node.get_clock().now(), self._autoware_connection_time, DISCONNECT_THRESHOLD
        ):
            self.information.mrm_state = MrmState.NORMAL
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
            # 値域逸脱 (SYS2-ERR-01): state / behavior が想定値域外の場合は
            # MRM 発生有無を不明として扱い、MRM 状態を非作動へリセットする。
            # (有効値域は pilot-auto のバージョン差を吸収するため config で可変)
            if (
                msg.state not in self._parameter.mrm_valid_states
                or msg.behavior not in self._parameter.mrm_valid_behaviors
            ):
                # 値域外が継続する構成 (待機中に UNKNOWN を publish し続ける等) では
                # トピックレートでログが流れるため throttle する。
                self._node.get_logger().error(
                    "MRM state out of range (state={}, behavior={}), reset MRM".format(
                        msg.state, msg.behavior
                    ),
                    throttle_duration_sec=5,
                )
                self.information.mrm_state = MrmState.NORMAL
                self.information.mrm_behavior = self._parameter.mrm_none_behavior
                return
            self.information.mrm_state = msg.state
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
        # 縦速度 [m/s] を保持する (横ジャーク計算 v^2 * steering_rate / wheel_base に使用)
        try:
            self.information.velocity = msg.twist.twist.twist.linear.x
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle velocity, ERROR: " + str(e))
        # ゴールまでの直線距離を算出する (UC-05 到着判定用)
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

    def sub_heartbeat_callback(self, msg):
        try:
            self._autoware_connection_time = self._node.get_clock().now()
        except Exception as e:
            self._node.get_logger().error("Unable to get the heartbeat, ERROR: " + str(e))

    def sub_control_metrics_callback(self, msg):
        # MetricArray の value は文字列のため float に変換して保持する
        try:
            for metric in msg.metric_array:
                attr = CONTROL_METRIC_ATTR_MAP.get(metric.name)
                if attr is None:
                    continue
                try:
                    setattr(self.information, attr, float(metric.value))
                except ValueError:
                    continue
        except Exception as e:
            self._node.get_logger().error("Unable to get the control metrics, ERROR: " + str(e))

    def fetch_vehicle_dimensions(self):
        # /api/vehicle/dimensions サービスから wheelbase を1回だけ取得する。
        # サービス未起動なら次のタイマーで再試行し、応答待ち中は多重リクエストしない。
        try:
            if self._dimensions_future is not None:
                return
            if not self._dimensions_client.service_is_ready():
                self._node.get_logger().warn(
                    "Waiting for /api/vehicle/dimensions service ...",
                    throttle_duration_sec=10,
                )
                return
            self._dimensions_future = self._dimensions_client.call_async(
                GetVehicleDimensions.Request()
            )
            self._dimensions_future.add_done_callback(self.on_vehicle_dimensions_response)
        except Exception as e:
            self._node.get_logger().error(
                "Unable to request vehicle dimensions, ERROR: " + str(e)
            )

    def on_vehicle_dimensions_response(self, future):
        try:
            wheel_base = future.result().dimensions.wheel_base
            if wheel_base > 0.0:
                self.information.wheel_base = wheel_base
                self._node.get_logger().info(
                    "Got wheelbase from /api/vehicle/dimensions: {:.3f} m".format(wheel_base)
                )
                # 取得できたのでリトライタイマーを停止する
                self._dimensions_timer.cancel()
            else:
                # 不正値のときは param のフォールバック値のまま次回リトライする
                self._node.get_logger().warn(
                    "Invalid wheelbase from service ({}), keep fallback".format(wheel_base)
                )
                self._dimensions_future = None
        except Exception as e:
            self._node.get_logger().error("Unable to get vehicle dimensions, ERROR: " + str(e))
            self._dimensions_future = None
