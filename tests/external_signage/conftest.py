# tests/external_signage/conftest.py
import sys
import types
from unittest.mock import MagicMock


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
    ("serial", {}),
    ("yaml", {}),
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
