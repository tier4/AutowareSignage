# tests/signage/test_mrm_state.py
import sys
import types
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest


class _MrmState:
    UNKNOWN = 0
    NORMAL = 1
    MRM_OPERATING = 2
    MRM_SUCCEEDED = 3
    MRM_FAILED = 4
    NONE = 1
    EMERGENCY_STOP = 2
    COMFORTABLE_STOP = 3
    PULL_OVER = 4


class _OperationModeState:
    UNKNOWN = 0
    STOP = 1
    AUTONOMOUS = 2


class _MotionState:
    UNKNOWN = 0
    STOPPED = 1
    STARTING = 2
    MOVING = 3


# ROS メッセージが無い環境 (素の pytest) でも import できるようにする。
# 他のテストが先に置いたスタブ (MagicMock) は判定に使う定数を持たないため、定数を持つクラスで補う。
try:
    import autoware_adapi_v1_msgs.msg as _msg
except ImportError:
    sys.modules["autoware_adapi_v1_msgs"] = types.ModuleType("autoware_adapi_v1_msgs")
    _msg = sys.modules["autoware_adapi_v1_msgs.msg"] = types.ModuleType(
        "autoware_adapi_v1_msgs.msg"
    )
if not isinstance(getattr(getattr(_msg, "MrmState", None), "MRM_OPERATING", None), int):
    _msg.MrmState = _MrmState
    _msg.OperationModeState = _OperationModeState
    _msg.MotionState = _MotionState
    for _name in ("RouteState", "LocalizationInitializationState", "DoorStatus"):
        if not hasattr(_msg, _name):
            setattr(_msg, _name, MagicMock())
    sys.modules.pop("signage.route_handler", None)

from autoware_adapi_v1_msgs.msg import MotionState, MrmState, OperationModeState  # noqa: E402
import signage.route_handler as route_handler  # noqa: E402
from signage.route_handler import RouteHandler  # noqa: E402

X2_MAIN_MODERATE_STOP = 11
X2_MAIN_EMERGENCY_STOP = 12
X2_SUB_STANDBY = 20
X2_SUB_MODERATE_STOP = 21


@pytest.fixture
def handler(monkeypatch):
    # コンストラクタはタイマー等を生成するため通さず、判定に必要な属性だけ用意する
    monkeypatch.setattr(route_handler.utils, "check_timeout", lambda *_args: False)
    h = RouteHandler.__new__(RouteHandler)
    h._node = MagicMock()
    h._parameter = SimpleNamespace(
        ignore_emergency=False,
        freeze_emergency=True,
        emergency_ignore_period=5.0,
        emergency_repeat_period=180.0,
        mrm_emergency_behaviors=[X2_MAIN_EMERGENCY_STOP],
    )
    h._autoware = SimpleNamespace(
        information=SimpleNamespace(
            operation_mode=OperationModeState.AUTONOMOUS,
            mrm_state=MrmState.NORMAL,
            mrm_behavior=MrmState.NONE,
            motion_state=MotionState.MOVING,
        )
    )
    h._announce_interface = MagicMock()
    h._in_emergency_state = False
    h._in_slowing_state = False
    h._in_slow_stop_state = False
    h._emergency_trigger_time = None
    return h


def _set_mrm(handler, state, behavior):
    handler._autoware.information.mrm_state = state
    handler._autoware.information.mrm_behavior = behavior


class TestEmergencyChecker:
    @pytest.mark.parametrize(
        "state", [MrmState.MRM_OPERATING, MrmState.MRM_SUCCEEDED, MrmState.MRM_FAILED]
    )
    def test_emergency_behavior_in_mrm_is_emergency(self, handler, state):
        _set_mrm(handler, state, X2_MAIN_EMERGENCY_STOP)
        handler.emergency_checker_callback()
        assert handler._in_emergency_state is True
        assert handler._in_slowing_state is False
        handler._announce_interface.announce_emergency.assert_called_once_with("emergency")

    @pytest.mark.parametrize(
        "behavior",
        [X2_MAIN_MODERATE_STOP, X2_SUB_MODERATE_STOP, MrmState.COMFORTABLE_STOP, MrmState.NONE],
    )
    def test_other_behavior_in_mrm_is_comfortable_stop(self, handler, behavior):
        _set_mrm(handler, MrmState.MRM_OPERATING, behavior)
        handler.emergency_checker_callback()
        assert handler._in_emergency_state is False
        assert handler._in_slowing_state is True
        handler._announce_interface.announce_emergency.assert_not_called()

    def test_comfortable_stop_after_stopped(self, handler):
        _set_mrm(handler, MrmState.MRM_SUCCEEDED, X2_MAIN_MODERATE_STOP)
        handler._autoware.information.motion_state = MotionState.STOPPED
        handler.emergency_checker_callback()
        assert handler._in_slow_stop_state is True
        assert handler._in_slowing_state is False

    @pytest.mark.parametrize("behavior", [X2_SUB_STANDBY, X2_MAIN_EMERGENCY_STOP])
    def test_normal_state_is_not_mrm(self, handler, behavior):
        _set_mrm(handler, MrmState.NORMAL, behavior)
        handler.emergency_checker_callback()
        assert handler._in_emergency_state is False
        assert handler._in_slowing_state is False
        assert handler._in_slow_stop_state is False

    def test_stop_operation_mode_ignores_mrm(self, handler):
        _set_mrm(handler, MrmState.MRM_OPERATING, X2_MAIN_EMERGENCY_STOP)
        handler._autoware.information.operation_mode = OperationModeState.STOP
        handler.emergency_checker_callback()
        assert handler._in_emergency_state is False
