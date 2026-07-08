# !/usr/bin/env python3
# -*- coding: utf-8 -*-
# This Python file uses the following encoding: utf-8

from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from autoware_adapi_v1_msgs.msg import OperationModeState

from signage.settings_store import KEY_STANDING_MODE


class StandingModeInterface:
    """立席運用モード(着席/立席)の保持・永続化・切替を担う (UC-01)。

    立席運用モードは UC-02/UC-03 の転倒防止アナウンスのゲート条件となる。
    切替は ~/set/standing_mode (SetBool) で行い、停車中(operation_mode == STOP)
    のときのみ受け付ける。設定値は SettingsStore (統合 JSON) に保存し、再起動後も保持する。
    """

    def __init__(self, node, autoware_interface, parameter_interface, settings_store):
        self._node = node
        self._autoware = autoware_interface
        self._parameter = parameter_interface.parameter
        self._settings = settings_store

        self._standing_mode = self._load_standing_mode()

        self._set_srv = self._node.create_service(
            SetBool, "~/set/standing_mode", self.set_standing_mode
        )
        self._get_pub = self._node.create_publisher(Bool, "~/get/standing_mode", 1)
        self._node.create_timer(1.0, self.publish_standing_mode_callback)

    @property
    def is_standing_mode(self):
        return self._standing_mode

    def _load_standing_mode(self):
        value = self._settings.get(KEY_STANDING_MODE)
        if isinstance(value, bool):
            return value
        # 設定が無い場合は既定値(パラメータ)を使用する
        return self._parameter.standing_mode_default

    def _save_standing_mode(self, standing_mode):
        self._settings.set(KEY_STANDING_MODE, standing_mode)

    def set_standing_mode(self, request, response):
        # SYS2-UC01-01: 停車中(operation_mode == STOP)のときのみ受付
        if self._autoware.information.operation_mode != OperationModeState.STOP:
            response.success = False
            response.message = "Standing mode can only be changed while the vehicle is stopped"
            self._node.get_logger().warning(response.message)
            return response

        # 切替不要(同値)の場合も成功として扱う
        if request.data == self._standing_mode:
            response.success = True
            response.message = "Standing mode is already {}".format(request.data)
            return response

        # SYS2-UC01-02/03: 不揮発化に成功した場合のみモードを更新する。
        # 書込失敗時は切替前のモードを保持し、失敗を通知する。
        try:
            self._save_standing_mode(request.data)
        except Exception as e:
            response.success = False
            response.message = "Failed to persist standing mode: {}".format(str(e))
            self._node.get_logger().error(response.message)
            return response

        self._standing_mode = request.data
        response.success = True
        response.message = "Standing mode changed to {}".format(request.data)
        self._node.get_logger().info(response.message)
        return response

    def publish_standing_mode_callback(self):
        self._get_pub.publish(Bool(data=self._standing_mode))
