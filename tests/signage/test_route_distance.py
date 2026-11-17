# tests/signage/test_route_distance.py
import importlib
import sys
import types
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

# ROS メッセージが無い環境 (素の pytest) でも import できるよう、無いモジュールだけスタブする
for _pkg, _sub, _names in (
    ("autoware_adapi_v1_msgs", "msg", ()),
    ("autoware_adapi_v1_msgs", "srv", ("GetVehicleDimensions",)),
    ("std_msgs", "msg", ("String",)),
    ("tier4_external_api_msgs", "msg", ("RouteDistance",)),
    ("tier4_metric_msgs", "msg", ("MetricArray",)),
):
    try:
        importlib.import_module(f"{_pkg}.{_sub}")
    except ImportError:
        sys.modules.setdefault(_pkg, types.ModuleType(_pkg))
        sys.modules[f"{_pkg}.{_sub}"] = types.ModuleType(f"{_pkg}.{_sub}")
    _mod = sys.modules[f"{_pkg}.{_sub}"]
    for _name in _names:
        if not hasattr(_mod, _name):
            setattr(_mod, _name, MagicMock())
_adapi = sys.modules["autoware_adapi_v1_msgs.msg"]
for _name in (
    "RouteState",
    "Route",
    "MrmState",
    "OperationModeState",
    "MotionState",
    "LocalizationInitializationState",
    "VehicleKinematics",
    "Heartbeat",
    "DoorStatus",
    "DoorStatusArray",
):
    if not hasattr(_adapi, _name):
        setattr(_adapi, _name, MagicMock())

from signage.autoware_interface import (  # noqa: E402
    DEFAULT_GOAL_DISTANCE,
    ROUTE_DISTANCE_MISSING_WARN_SEC,
    ROUTE_DISTANCE_SETTLE_SEC,
    SAME_GOAL_THRESHOLD,
    AutowareInformation,
    AutowareInterface,
)


class _Clock:
    def __init__(self):
        self.sec = 100.0

    def now(self):
        return SimpleNamespace(nanoseconds=int(self.sec * 1e9))


@pytest.fixture
def clock():
    return _Clock()


@pytest.fixture
def iface(clock):
    # コンストラクタは購読・タイマーを生成するため通さず、判定に必要な属性だけ用意する
    i = AutowareInterface.__new__(AutowareInterface)
    i._node = MagicMock()
    i._node.get_clock.return_value = clock
    i.information = AutowareInformation()
    i._route_received_sec = None
    i._goal_position = None
    i._route_distance_received = False
    return i


def _route(x=0.0, y=0.0):
    goal = SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=0.0))
    return SimpleNamespace(data=[SimpleNamespace(goal=goal)])


def _no_route():
    return SimpleNamespace(data=[])


def _distance(d):
    return SimpleNamespace(remaining_distance=d)


def _settle(clock):
    clock.sec += ROUTE_DISTANCE_SETTLE_SEC + 0.1


def test_default_goal_distance():
    assert AutowareInformation().goal_distance == DEFAULT_GOAL_DISTANCE


def test_distance_ignored_without_route(iface):
    iface.sub_route_distance_callback(_distance(5.0))
    assert iface.information.goal_distance == DEFAULT_GOAL_DISTANCE


def test_distance_ignored_during_settle_period(iface, clock):
    iface.sub_route_callback(_route())
    clock.sec += ROUTE_DISTANCE_SETTLE_SEC - 0.1
    iface.sub_route_distance_callback(_distance(0.0))
    assert iface.information.goal_distance == DEFAULT_GOAL_DISTANCE


def test_distance_applied_after_settle_period(iface, clock):
    iface.sub_route_callback(_route())
    _settle(clock)
    iface.sub_route_distance_callback(_distance(42.0))
    assert iface.information.goal_distance == 42.0


def test_route_clear_resets_and_ignores_distance(iface, clock):
    iface.sub_route_callback(_route())
    _settle(clock)
    iface.sub_route_distance_callback(_distance(3.0))
    iface.sub_route_callback(_no_route())
    assert iface.information.goal_distance == DEFAULT_GOAL_DISTANCE
    _settle(clock)
    iface.sub_route_distance_callback(_distance(3.0))
    assert iface.information.goal_distance == DEFAULT_GOAL_DISTANCE


def test_route_to_other_goal_resets(iface, clock):
    iface.sub_route_callback(_route(0.0, 0.0))
    _settle(clock)
    iface.sub_route_distance_callback(_distance(0.5))
    iface.sub_route_callback(_route(SAME_GOAL_THRESHOLD + 1.0, 0.0))
    assert iface.information.goal_distance == DEFAULT_GOAL_DISTANCE
    # 旧ルートの path で算出された距離は猶予期間中に破棄される
    iface.sub_route_distance_callback(_distance(0.5))
    assert iface.information.goal_distance == DEFAULT_GOAL_DISTANCE


def test_reroute_to_same_goal_keeps_distance(iface, clock):
    iface.sub_route_callback(_route(0.0, 0.0))
    _settle(clock)
    iface.sub_route_distance_callback(_distance(30.0))
    iface.sub_route_callback(_route(3.0, 1.0))
    assert iface.information.goal_distance == 30.0
    iface.sub_route_distance_callback(_distance(29.0))
    assert iface.information.goal_distance == 30.0
    _settle(clock)
    iface.sub_route_distance_callback(_distance(25.0))
    assert iface.information.goal_distance == 25.0


def test_warn_when_distance_missing(iface, clock):
    iface.sub_route_callback(_route())
    clock.sec += ROUTE_DISTANCE_MISSING_WARN_SEC - 0.1
    iface.check_route_distance_received()
    iface._node.get_logger.return_value.warning.assert_not_called()
    clock.sec += 0.2
    iface.check_route_distance_received()
    iface._node.get_logger.return_value.warning.assert_called_once()


def test_no_warn_when_distance_received(iface, clock):
    iface.sub_route_callback(_route())
    _settle(clock)
    iface.sub_route_distance_callback(_distance(10.0))
    clock.sec += ROUTE_DISTANCE_MISSING_WARN_SEC
    iface.check_route_distance_received()
    iface._node.get_logger.return_value.warning.assert_not_called()


def test_no_warn_without_route(iface, clock):
    clock.sec += ROUTE_DISTANCE_MISSING_WARN_SEC + 1
    iface.check_route_distance_received()
    iface._node.get_logger.return_value.warning.assert_not_called()
