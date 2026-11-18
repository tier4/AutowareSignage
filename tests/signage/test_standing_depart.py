# tests/signage/test_standing_depart.py
import sys
import types
from types import SimpleNamespace
from unittest.mock import MagicMock


class _OperationModeState:
    UNKNOWN = 0
    STOP = 1
    AUTONOMOUS = 2


# ROS メッセージが無い環境 (素の pytest) でも import できるようにする。
try:
    import autoware_adapi_v1_msgs.msg as _msg
except ImportError:
    sys.modules["autoware_adapi_v1_msgs"] = types.ModuleType("autoware_adapi_v1_msgs")
    _msg = sys.modules["autoware_adapi_v1_msgs.msg"] = types.ModuleType(
        "autoware_adapi_v1_msgs.msg"
    )
if not isinstance(getattr(getattr(_msg, "OperationModeState", None), "AUTONOMOUS", None), int):
    _msg.OperationModeState = _OperationModeState
    for _name in (
        "RouteState",
        "MotionState",
        "MrmState",
        "LocalizationInitializationState",
        "DoorStatus",
    ):
        if not hasattr(_msg, _name):
            setattr(_msg, _name, MagicMock())
    sys.modules.pop("signage.route_handler", None)

from autoware_adapi_v1_msgs.msg import OperationModeState  # noqa: E402
from signage.route_handler import RouteHandler  # noqa: E402


def _make_handler(is_standing_mode, in_interval):
    # コンストラクタはタイマー等を生成するため通さず、発車判定に必要な属性だけ用意する
    h = RouteHandler.__new__(RouteHandler)
    h._node = MagicMock()
    h._parameter = SimpleNamespace(signage_stand_alone=True, check_fms_time=2.0)
    h._autoware = SimpleNamespace(
        information=SimpleNamespace(
            operation_mode=OperationModeState.AUTONOMOUS,
            autoware_control=True,
            route_state=MagicMock(),
        )
    )
    h._standing_mode = SimpleNamespace(is_standing_mode=is_standing_mode)
    h._announce_interface = MagicMock()
    h._announce_interface.in_interval.return_value = in_interval
    h._service_interface = MagicMock()
    h._announce_engage = False
    h._trigger_external_signage = True
    h._prev_route_state = MagicMock()
    h._fms_check_time = 0
    h._in_emergency_state = True  # 以降の到着判定は本テストの対象外
    h.process_station_list_from_fms = MagicMock()
    return h


def _announced(h):
    return [c.args[0] for c in h._announce_interface.send_announce.call_args_list]


def test_standing_mode_on_plays_standing_depart():
    h = _make_handler(is_standing_mode=True, in_interval=False)
    h.route_checker_callback()
    assert _announced(h) == ["standing_depart"]
    h._announce_interface.set_timeout.assert_called_once_with("standing_depart")


def test_standing_mode_on_in_interval_announces_nothing():
    # インターバル中の再発車では standing_depart をスキップし、engage にもフォールバックしない
    h = _make_handler(is_standing_mode=True, in_interval=True)
    h.route_checker_callback()
    assert _announced(h) == []
    h._announce_interface.set_timeout.assert_not_called()
    assert h._announce_engage is True


def test_standing_mode_off_plays_engage():
    h = _make_handler(is_standing_mode=False, in_interval=False)
    h.route_checker_callback()
    assert _announced(h) == ["engage"]
