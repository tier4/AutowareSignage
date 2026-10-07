# tests/external_signage/test_driving_status.py
import sys
import types
from unittest.mock import MagicMock

import pytest


def _stub_module(name, **attrs):
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


# ROS メッセージ/パッケージが無い環境 (素の pytest) でも import できるようにする
try:
    from tier4_external_api_msgs.msg import DrivingStatus
except ImportError:

    class DrivingStatus:
        UNKNOWN = 0
        STOP = 1
        LEVEL2 = 2
        LEVEL4 = 3

        def __init__(self, mode=0):
            self.mode = mode

    _stub_module("tier4_external_api_msgs")
    _stub_module("tier4_external_api_msgs.msg", DrivingStatus=DrivingStatus)

for _name, _attrs in (
    ("ament_index_python", {}),
    ("ament_index_python.packages", {"get_package_share_directory": MagicMock()}),
    ("std_srvs", {}),
    ("std_srvs.srv", {"SetBool": MagicMock()}),
    ("std_msgs", {}),
    ("std_msgs.msg", {"Bool": MagicMock(), "String": MagicMock()}),
    ("autoware_adapi_v1_msgs", {}),
    ("autoware_adapi_v1_msgs.msg", {"MrmState": MagicMock()}),
):
    try:
        __import__(_name)
    except ImportError:
        _stub_module(_name, **_attrs)

from external_signage import display_mode  # noqa: E402
from external_signage.external_signage_core import ExternalSignage  # noqa: E402


def _driving_status(mode):
    msg = DrivingStatus()
    msg.mode = mode
    return msg


@pytest.fixture
def signage():
    # コンストラクタはシリアルポートや ~/settings.json に触れるため通さず、必要な属性だけ用意する
    s = ExternalSignage.__new__(ExternalSignage)
    s.node = MagicMock()
    s._settings = {"in_experiment": True, "airport": False, "display_mode": display_mode.AUTONOMOUS}
    s.autoware_status = {"driving": False, "mrm": False}
    s._last_driving_mode = None
    s.display_signage = MagicMock()
    s.pub_mode_status = MagicMock()
    s._save_settings = MagicMock()
    return s


class TestSubDrivingStatus:
    def test_level4_switches_to_l4(self, signage):
        signage.sub_driving_status(_driving_status(DrivingStatus.LEVEL4))
        assert signage._settings["in_experiment"] is False
        signage.display_signage.assert_called_once_with("null")
        signage._save_settings.assert_called_once()

    def test_stop_and_unknown_keep_previous_level(self, signage):
        signage.sub_driving_status(_driving_status(DrivingStatus.LEVEL4))
        signage.display_signage.reset_mock()
        for mode in (DrivingStatus.STOP, DrivingStatus.UNKNOWN):
            signage.sub_driving_status(_driving_status(mode))
        assert signage._settings["in_experiment"] is False
        signage.display_signage.assert_not_called()

    def test_same_level_is_processed_once(self, signage):
        # availability/route の変化でも同じ mode が届くので、二度目は無視する
        signage.sub_driving_status(_driving_status(DrivingStatus.LEVEL4))
        signage.sub_driving_status(_driving_status(DrivingStatus.LEVEL4))
        assert signage._save_settings.call_count == 1

    def test_failed_display_is_retried_on_next_message(self, signage):
        # 描画に失敗したレベルは記録せず、同じレベルの次メッセージで再試行する
        signage.display_signage.side_effect = [OSError("serial write failed"), None]
        signage.sub_driving_status(_driving_status(DrivingStatus.LEVEL2))
        assert signage._last_driving_mode is None
        signage._save_settings.assert_not_called()

        signage.sub_driving_status(_driving_status(DrivingStatus.LEVEL2))
        assert signage._last_driving_mode == DrivingStatus.LEVEL2
        signage._save_settings.assert_called_once()

    def test_failed_save_is_retried_on_next_message(self, signage):
        signage._save_settings.side_effect = [OSError("disk full"), None]
        signage.sub_driving_status(_driving_status(DrivingStatus.LEVEL4))
        assert signage._last_driving_mode is None

        signage.sub_driving_status(_driving_status(DrivingStatus.LEVEL4))
        assert signage._last_driving_mode == DrivingStatus.LEVEL4

    def test_manual_mode_change_is_kept_until_level_changes(self, signage):
        # /signage/mode_change で L2 にした後、同じ LEVEL4 の再通知では L4 に戻さない (旧トピックと同等)
        signage.sub_driving_status(_driving_status(DrivingStatus.LEVEL4))
        request = MagicMock(data=True)  # True is L2
        signage.change_mode(request, MagicMock())
        assert signage._settings["in_experiment"] is True

        signage.sub_driving_status(_driving_status(DrivingStatus.LEVEL4))
        assert signage._settings["in_experiment"] is True

        # 実際のレベル変化 (L2 -> L4) では追従する
        signage.sub_driving_status(_driving_status(DrivingStatus.LEVEL2))
        signage.sub_driving_status(_driving_status(DrivingStatus.LEVEL4))
        assert signage._settings["in_experiment"] is False


class TestInitialDisplay:
    def test_level4_blanks_display(self, signage):
        # LED は前回フレームを保持するため、Lv4 起動時も明示的に null を描画する
        signage._settings["in_experiment"] = False
        signage._initial_display()
        signage.pub_mode_status.assert_called_once_with(False)
        signage.display_signage.assert_called_once_with("null")

    def test_level4_does_not_show_auto_before_driving_is_confirmed(self, signage):
        signage._settings["in_experiment"] = False
        signage.autoware_status["driving"] = True
        signage._initial_display()
        signage.display_signage.assert_called_once_with("null")

    def test_level2_shows_experiment(self, signage):
        signage._initial_display()
        signage.pub_mode_status.assert_called_once_with(True)
        signage.display_signage.assert_called_once_with("experiment")
