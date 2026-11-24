# tests/signage/test_final_out_of_service.py
# 終点到着後は FMS のスケジュール完了反映を待たずに「回送中」(out_of_service) を表示する。
import sys
import types
from types import SimpleNamespace
from unittest.mock import MagicMock

# ROS メッセージが無い環境 (素の pytest) でも import できるようにする。
try:
    import autoware_adapi_v1_msgs.msg as _msg
except ImportError:
    sys.modules["autoware_adapi_v1_msgs"] = types.ModuleType("autoware_adapi_v1_msgs")
    _msg = sys.modules["autoware_adapi_v1_msgs.msg"] = types.ModuleType(
        "autoware_adapi_v1_msgs.msg"
    )
for _name in (
    "RouteState",
    "OperationModeState",
    "MotionState",
    "MrmState",
    "LocalizationInitializationState",
    "DoorStatus",
):
    if not hasattr(_msg, _name):
        setattr(_msg, _name, MagicMock())

import signage.signage_utils as utils  # noqa: E402
from signage.route_handler import RouteHandler  # noqa: E402


def _make_handler(task_list):
    # コンストラクタはタイマー等を生成するため通さず、判定に必要な属性だけ用意する
    h = RouteHandler.__new__(RouteHandler)
    h._node = MagicMock()
    h._viewController = SimpleNamespace()
    h._parameter = SimpleNamespace(ignore_disconnected=True, ignore_manual_driving=True)
    h._autoware = SimpleNamespace(
        is_disconnected=False,
        information=SimpleNamespace(autoware_control=True),
    )
    h._cvm = MagicMock()
    h._cvm.get_view_mode_override.return_value = None
    h._cvm.get_display_mode_id_override.return_value = None
    h._announce_interface = MagicMock()
    h._announce_interface.in_interval.return_value = False
    h._display_details = utils.init_DisplayDetails()
    h._display_phrase = ""
    h._sudden_warning_type = ""
    h._in_emergency_state = False
    h._in_slowing_state = False
    h._in_slow_stop_state = False
    h._announced_arrive_caution = False
    h._arrived_station = ["", ""]
    h._reach_final = False
    h._announced_depart = False
    h._is_stopping = True
    h._is_driving = False
    h.task_list = task_list
    h._current_task_details = utils.CurrentTask(["A", "a"], ["B", "b"], 0)
    return h


def test_final_arrival_shows_out_of_service_immediately():
    # 最終区間 (todo なし) で到着 → FMS 未反映でも即 回送中
    h = _make_handler(utils.TaskList([], [], []))
    h.arrived_goal()
    assert h._reach_final
    # 終点名が departure_station に残っていても stopping (空白枠を含む) にしない
    assert h._current_task_details.departure_station == ["B", "b"]
    h.view_mode_callback()
    assert h._viewController.view_mode == "out_of_service"


def test_intermediate_arrival_still_shows_stopping():
    # 途中停留所の到着では従来どおり stopping
    next_task = {
        "task_type": "move",
        "status": "todo",
        "origin": {"name": "B;b"},
        "destination": {"name": "C;c"},
        "plan_start_time": "",
    }
    h = _make_handler(utils.TaskList([], [next_task], []))
    h.arrived_goal()
    assert not h._reach_final
    h.view_mode_callback()
    assert h._viewController.view_mode == "stopping"
