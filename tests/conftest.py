import sys
from pathlib import Path
from unittest.mock import MagicMock

# Mock rclpy module before any signage module imports it
rclpy_mock = MagicMock()
sys.modules["rclpy"] = rclpy_mock
sys.modules["rclpy.duration"] = rclpy_mock.duration
sys.modules["rclpy.node"] = rclpy_mock.node
sys.modules["rclpy.qos"] = rclpy_mock.qos

# Add source directories to sys.path for module resolution
# (pythonpath in pytest.ini requires pytest >= 7.0)
_base = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_base / "src" / "signage" / "src"))
sys.path.insert(0, str(_base / "src" / "external_signage" / "src"))
