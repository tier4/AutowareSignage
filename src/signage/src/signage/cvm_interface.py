# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

from threading import Lock

from command_view_manager_msgs.msg import DeviceDisplayState, DisplayModeCommand
from command_view_manager_msgs.srv import GetManifest
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


COMMAND_TOPIC_TEMPLATE = "/command_view_manager/devices/{device_id}/command"
STATE_TOPIC_TEMPLATE = "/command_view_manager/devices/{device_id}/state"
GET_MANIFEST_SERVICE = "/command_view_manager/get_manifest"

# CVM コマンドを受けたときの内部 view_mode。
# 個別の表示は QML 側で cvm_display_mode_id を見て分岐する。
CVM_VIEW_MODE = "cvm_display"

# CVM コマンドとして受け付ける display_mode_id（cvm_display ページ内で表示分岐するもの）
SUPPORTED_CVM_DISPLAY_MODES = {"remote_emergency_display"}

# 受信すると override を解除し autoware 由来の view_mode に戻す display_mode_id
RELEASE_DISPLAY_MODES = {"idle_display"}

# signage internal view_mode → 候補 CVM display_mode_id（state 報告用）。
# マニフェストの supported_display_mode_ids に含まれる候補のみ採用し、
# 含まれていない場合は DEFAULT_REPORTED_DISPLAY_MODE にフォールバックする。
VIEW_MODE_TO_CVM = {
    "emergency_stopped": "mrm_emergency_display",
    "disconnected": "disconnected_display",
    "manual_driving": "manual_driving_display",
    "auto_driving": "auto_driving_display",
    "slowing": "slow_stop_display",
    "slow_stop": "slow_stop_display",
    "stopping": "arriving_display",
    "driving": "destination_display",
    "out_of_service": "idle_display",
}

# 候補がマニフェストに含まれない／マニフェスト未取得の時のフォールバック
DEFAULT_REPORTED_DISPLAY_MODE = "idle_display"

STATE_PUBLISH_INTERVAL_SEC = 1.0
MANIFEST_FETCH_RETRY_SEC = 5.0


class CvmInterface:
    """CommandViewManager とのインタフェース。

    - `/command_view_manager/devices/<device_id>/command` を購読し、
      指定された display_mode_id に対応する view_mode を上書き要求として保持する
    - `/command_view_manager/devices/<device_id>/state` に
      現在の view_mode を CVM 向けにマッピングして 1Hz で発信する
    """

    def __init__(self, node, device_id):
        self._node = node
        self._device_id = device_id
        self._lock = Lock()
        self._cvm_view_mode_override = None
        self._cvm_display_mode_id_override = None
        self._current_view_mode = ""
        # マニフェスト取得状態
        # None: 未取得（DEFAULT_REPORTED_DISPLAY_MODE 固定で報告）
        # set:  取得済み（含まれる候補のみ採用、それ以外は DEFAULT_REPORTED_DISPLAY_MODE）
        self._supported_modes = None
        self._manifest_request_pending = False

        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._command_sub = node.create_subscription(
            DisplayModeCommand,
            COMMAND_TOPIC_TEMPLATE.format(device_id=device_id),
            self._on_command,
            command_qos,
        )
        self._state_pub = node.create_publisher(
            DeviceDisplayState,
            STATE_TOPIC_TEMPLATE.format(device_id=device_id),
            state_qos,
        )
        self._timer = node.create_timer(
            STATE_PUBLISH_INTERVAL_SEC, self._publish_state
        )

        self._manifest_client = node.create_client(
            GetManifest, GET_MANIFEST_SERVICE
        )
        self._manifest_timer = node.create_timer(
            MANIFEST_FETCH_RETRY_SEC, self._try_fetch_manifest
        )
        # 起動直後にも一度試みる（タイマー初回発火を待たない）
        self._try_fetch_manifest()

        node.get_logger().info(
            "CvmInterface started: device_id={}".format(device_id)
        )

    def _on_command(self, msg):
        # オーバーライドの変更はクライアントから来た正当なコマンドでのみ行う。
        # サポート外の display_mode_id は WARN ログのみ出して既存の override は維持する
        if msg.display_mode_id in RELEASE_DISPLAY_MODES:
            self._node.get_logger().info(
                "CVM override released: display_mode_id={}, issuer={}".format(
                    msg.display_mode_id, msg.issuer_client_id
                )
            )
            with self._lock:
                self._cvm_view_mode_override = None
                self._cvm_display_mode_id_override = None
            return

        if msg.display_mode_id not in SUPPORTED_CVM_DISPLAY_MODES:
            self._node.get_logger().warning(
                "CVM command with unsupported display_mode_id ignored: "
                "display_mode_id={}, issuer={}".format(
                    msg.display_mode_id, msg.issuer_client_id
                )
            )
            return

        self._node.get_logger().info(
            "CVM command received: display_mode_id={}, issuer={}, priority={}".format(
                msg.display_mode_id, msg.issuer_client_id, msg.priority,
            )
        )
        with self._lock:
            self._cvm_view_mode_override = CVM_VIEW_MODE
            self._cvm_display_mode_id_override = msg.display_mode_id

    def get_view_mode_override(self):
        with self._lock:
            return self._cvm_view_mode_override

    def get_display_mode_id_override(self):
        with self._lock:
            return self._cvm_display_mode_id_override

    def set_current_view_mode(self, view_mode):
        with self._lock:
            self._current_view_mode = view_mode

    def _publish_state(self):
        with self._lock:
            view_mode = self._current_view_mode
            supported = self._supported_modes
            cvm_override_id = self._cvm_display_mode_id_override

        # CVM override 中は受け付けた display_mode_id をそのまま CVM に返す
        preferred = cvm_override_id if cvm_override_id else VIEW_MODE_TO_CVM.get(view_mode)
        if supported is not None and preferred is not None and preferred in supported:
            display_mode_id = preferred
        else:
            display_mode_id = DEFAULT_REPORTED_DISPLAY_MODE

        msg = DeviceDisplayState()
        msg.stamp = self._node.get_clock().now().to_msg()
        msg.device_id = self._device_id
        msg.display_mode_id = display_mode_id
        msg.summary_format = ""
        msg.summary_text = ""
        self._state_pub.publish(msg)

    def _try_fetch_manifest(self):
        with self._lock:
            already_loaded = self._supported_modes is not None
            pending = self._manifest_request_pending
        if already_loaded:
            self._manifest_timer.cancel()
            return
        if pending:
            return
        if not self._manifest_client.service_is_ready():
            self._node.get_logger().info(
                "CVM manifest service not available yet, "
                "retrying in {}s".format(MANIFEST_FETCH_RETRY_SEC),
                throttle_duration_sec=30,
            )
            return

        with self._lock:
            self._manifest_request_pending = True
        future = self._manifest_client.call_async(GetManifest.Request())
        future.add_done_callback(self._on_manifest_response)

    def _on_manifest_response(self, future):
        with self._lock:
            self._manifest_request_pending = False

        try:
            response = future.result()
        except Exception as e:
            self._node.get_logger().warning(
                "Failed to fetch CVM manifest: {}".format(e)
            )
            return

        if response is None:
            return

        for device in response.manifest.devices:
            if device.device_id == self._device_id:
                supported = set(device.supported_display_mode_ids)
                with self._lock:
                    self._supported_modes = supported
                self._node.get_logger().info(
                    "CVM manifest loaded: device={}, supported_modes={}".format(
                        self._device_id, sorted(supported)
                    )
                )
                self._manifest_timer.cancel()
                return

        self._node.get_logger().warning(
            "CVM manifest does not contain device '{}'; "
            "all states will be reported as '{}'".format(
                self._device_id, DEFAULT_REPORTED_DISPLAY_MODE
            )
        )
        with self._lock:
            self._supported_modes = set()
        self._manifest_timer.cancel()
