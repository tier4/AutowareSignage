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
    DoorStatus,
    DoorStatusArray,
)
from autoware_adapi_v1_msgs.srv import GetVehicleDimensions
from std_msgs.msg import String
from tier4_external_api_msgs.msg import RouteDistance
import signage.signage_utils as utils
from tier4_metric_msgs.msg import MetricArray

DISCONNECT_THRESHOLD = 2

# ゴールまでの残り距離が不明なときの既定値 [m] (到着判定が成立しない十分大きな値)
DEFAULT_GOAL_DISTANCE = 1000.0
# ルート受信後、この時間 [s] は /api/external/get/route_distance を採用しない。
# 残り距離の算出元 (path_distance_calculator) は 1Hz で、最後に受信した behavior path と
# 自車位置から距離を出す。距離の stamp は自車位置の時刻であり、ルートの stamp も
# ルート設定リクエスト由来でどちらも「新ルートで算出された距離か」の判定には使えない。
# そのため calculator 1 周期 + planner の新 path 出力遅延を見込んだ猶予期間で旧ルート分を捨てる。
ROUTE_DISTANCE_SETTLE_SEC = 2.0
# ルート設定後この時間 [s] 経っても残り距離を 1 度も受信できなければ warning を出す
ROUTE_DISTANCE_MISSING_WARN_SEC = 10.0
# 新旧ルートのゴール位置の差がこれ以下 [m] なら同じ目的地への reroute
# (goal_planner のゴール修正等) とみなし、猶予期間中も直前の残り距離を保持する
SAME_GOAL_THRESHOLD = 20.0

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
    goal_distance: float = DEFAULT_GOAL_DISTANCE
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
        # 現在のルートを受信した時刻 [s] (signage ノードの時計)。ルート未設定時は None
        self._route_received_sec = None
        # 現在のルートのゴール位置 (map系)。ルート未設定時は None
        self._goal_position = None
        # 現在のルートで残り距離を採用済みか (未受信 warning 用)
        self._route_distance_received = False

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
        # /api/external/get/door (tier4_external_api) は廃止されたため AD API の
        # Vehicle Doors API (/api/vehicle/doors/status, DoorStatusArray) へ移行。
        # AD API 側は RELIABLE + TRANSIENT_LOCAL で publish されるため api_qos を使う。
        self._sub_vehicle_door = node.create_subscription(
            DoorStatusArray,
            "/api/vehicle/doors/status",
            self.sub_vehicle_door_callback,
            api_qos,
        )
        # ゴールまでの経路上の残り距離は /api/external/get/route_distance から取得する。
        # 残り距離は path_distance_calculator が 1Hz で算出しており、ルート解除後も
        # 直前ルートの距離を publish し続け、ルート切替直後は旧ルートの距離が届きうる。
        # そのため /api/routing/route でルートの有無と受信時刻を保持し、
        # ルート未設定中・ルート受信から ROUTE_DISTANCE_SETTLE_SEC 以内の距離は無視する。
        # 更新は 1Hz のため、到着接近 (arriving_distance) の判定は最大 1 周期分遅れうる (許容)。
        self._sub_route = node.create_subscription(
            Route,
            "/api/routing/route",
            self.sub_route_callback,
            api_qos,
        )
        self._sub_route_distance = node.create_subscription(
            RouteDistance,
            "/api/external/get/route_distance",
            self.sub_route_distance_callback,
            sub_qos,
        )
        self._route_distance_check_timer = node.create_timer(
            1.0, self.check_route_distance_received
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
        # /api/system/heartbeat が DISCONNECT_THRESHOLD 秒以上途絶したら Autoware 切断とみなす。
        # ログは事後診断のため、切断への遷移時と復帰時にそれぞれ1回だけ残す (継続中は再出力しない)。
        if utils.check_timeout(
            self._node.get_clock().now(), self._autoware_connection_time, DISCONNECT_THRESHOLD
        ):
            self.information.mrm_state = MrmState.NORMAL
            self.information.mrm_behavior = MrmState.NONE
            if not self.is_disconnected:
                self._node.get_logger().error(
                    "Autoware disconnected: /api/system/heartbeat not received for {}s".format(
                        DISCONNECT_THRESHOLD
                    )
                )
            self.is_disconnected = True
        else:
            if self.is_disconnected:
                self._node.get_logger().info("Autoware reconnected: /api/system/heartbeat resumed")
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
            # 値域逸脱 (SYS2-ERR-01): state が想定値域外の場合は
            # MRM 発生有無を不明として扱い、MRM 状態を非作動へリセットする。
            # behavior は車両プラグインごとに独自値 (X2 は 11/12/20/21 等) を取るため値域判定しない。
            if msg.state not in self._parameter.mrm_valid_states:
                # 値域外が継続する構成 (待機中に UNKNOWN を publish し続ける等) では
                # トピックレートでログが流れるため throttle する。
                self._node.get_logger().error(
                    "MRM state out of range (state={}, behavior={}), reset MRM".format(
                        msg.state, msg.behavior
                    ),
                    throttle_duration_sec=5,
                )
                self.information.mrm_state = MrmState.NORMAL
                self.information.mrm_behavior = MrmState.NONE
                return
            self.information.mrm_state = msg.state
            self.information.mrm_behavior = msg.behavior
        except Exception as e:
            self._node.get_logger().error("Unable to get the mrm behavior, ERROR: " + str(e))

    def sub_vehicle_door_callback(self, msg):
        # DoorStatusArray は複数ドアの状態リストを持つため、乗客への開閉警告用に
        # 1つの代表状態へ集約する。開扉/閉扉アナウンスの契機となる遷移状態を優先し、
        # いずれかのドアが OPENING/CLOSING ならその状態を採用、それ以外は先頭ドアの状態を使う。
        # この集約 + route_handler 側の重複抑制 (_pre_door_announce_status) により、
        # 複数ドアが同時に開閉中でも「開扉/閉扉」発話は1回に集約される。
        # TODO(留意事項): 将来的にはどのドアが開閉中かを個別に発話したいが、
        # ドア構成 (index の対応) が現状不明なため、今は代表状態への集約のみとする。
        try:
            statuses = [door.status for door in msg.doors]
            if DoorStatus.OPENING in statuses:
                self.information.door_status = DoorStatus.OPENING
            elif DoorStatus.CLOSING in statuses:
                self.information.door_status = DoorStatus.CLOSING
            elif statuses:
                self.information.door_status = statuses[0]
            else:
                self.information.door_status = DoorStatus.UNKNOWN
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle door status, ERROR: " + str(e))

    def _now_sec(self):
        return self._node.get_clock().now().nanoseconds * 1e-9

    def sub_route_callback(self, msg):
        try:
            if not msg.data:
                self._route_received_sec = None
                self._goal_position = None
                self.information.goal_distance = DEFAULT_GOAL_DISTANCE
                return

            goal = msg.data[0].goal.position
            # 別の目的地へのルートに変わったときは旧ルートの残り距離を引き継がないよう
            # 既定値へ戻し、新ルートの距離が届くまで到着判定させない。
            # 同じ目的地への reroute では表示がちらつかないよう直前の値を保持する。
            if self._goal_position is None or (
                math.hypot(goal.x - self._goal_position.x, goal.y - self._goal_position.y)
                > SAME_GOAL_THRESHOLD
            ):
                self.information.goal_distance = DEFAULT_GOAL_DISTANCE
            self._goal_position = goal
            self._route_received_sec = self._now_sec()
            self._route_distance_received = False
        except Exception as e:
            self._node.get_logger().error("Unable to get the route, ERROR: " + str(e))

    def sub_route_distance_callback(self, msg):
        try:
            if self._route_received_sec is None:
                return
            # ルート受信直後は旧ルートの path で算出された距離が届きうるため破棄する
            if self._now_sec() - self._route_received_sec < ROUTE_DISTANCE_SETTLE_SEC:
                return
            self.information.goal_distance = msg.remaining_distance
            self._route_distance_received = True
        except Exception as e:
            self._node.get_logger().error("Unable to get the goal distance, ERROR: " + str(e))

    def check_route_distance_received(self):
        # 残り距離が届かない構成 (tier4_autoware_api_extension 未起動等) では
        # goal_distance が既定値のまま到着判定が働かないため、気付けるよう警告する
        try:
            if self._route_received_sec is None or self._route_distance_received:
                return
            if self._now_sec() - self._route_received_sec < ROUTE_DISTANCE_MISSING_WARN_SEC:
                return
            self._node.get_logger().warning(
                "No route distance received from /api/external/get/route_distance "
                "after the route was set. Arrival detection is disabled.",
                throttle_duration_sec=10,
            )
        except Exception as e:
            self._node.get_logger().error("Unable to check the route distance, ERROR: " + str(e))

    def sub_kinematics_callback(self, msg):
        # 縦速度 [m/s] を保持する (横ジャーク計算 v^2 * steering_rate / wheel_base に使用)
        try:
            self.information.velocity = msg.twist.twist.twist.linear.x
        except Exception as e:
            self._node.get_logger().error("Unable to get the vehicle velocity, ERROR: " + str(e))

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
            self._node.get_logger().error("Unable to request vehicle dimensions, ERROR: " + str(e))

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
