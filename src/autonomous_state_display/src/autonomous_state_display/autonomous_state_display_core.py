from dataclasses import dataclass
import datetime
import time
import json
import os
import uuid
import rclpy
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
from autoware_adapi_v1_msgs.msg import MrmState, OperationModeState
from std_msgs.msg import String
from level4_mode_manager_msgs.msg import Level4DrivingStatus
import subprocess

class AutonomousStateDisplay:
    def __init__(self, node):
        self.node = node

        self.autoware_status = {
            "driving": False,
            "mrm": False,
        }
        self.is_autoware_launch = False

        # ros interface
        api_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        node.create_service(SetBool, "/signage/trigger_external", self.trigger_external_signage)
        node.create_service(SetBool, "/signage/mode_change", self.change_mode)
        node.create_service(SetBool, "/signage/airport_mode_change", self.change_airport_mode)
        self.mode_status_pub_ = node.create_publisher(Bool, "/signage/mode_status", api_qos)
        self.setting_pub_ = node.create_publisher(String, "/signage/external/settings", api_qos)
        self._sub_mrm = node.create_subscription(
            MrmState,
            "/api/fail_safe/mrm_state",
            self.sub_mrm_callback,
            api_qos,
        )
        self._sub_operation_mode = node.create_subscription(
            OperationModeState,
            "/api/operation_mode/state",
            self.sub_operation_mode_callback,
            api_qos,
        )
        self._sub_is_driving_level = node.create_subscription(
            Level4DrivingStatus,
            "/level4_mode_manager/is_level4_driving",
            self.sub_is_driving_level,
            api_qos,
        )

        # read settings.If not, creatte settings.
        self._settings_file = "/home/" + os.environ.get("USER") + "/settings.json"
        if os.path.exists(self._settings_file):
            with open(self._settings_file, "r") as f:
                self._settings = json.load(f)
        else:
            self._settings = {"in_experiment": True, "airport": False}
            with open(self._settings_file, "w") as f:
                json.dump(self._settings, f, indent=4)

        # initial display 何も表示しない
        self.display_signage("null", True)

        # 状態出力するタイマー
        self.timer = node.create_timer(1, self.pub_setting)

    def pub_setting(self):
        setting = json.dumps(self._settings)
        msg = String()
        msg.data = setting
        self.setting_pub_.publish(msg)

    def pub_mode_status(self, status):
        msg = Bool()
        msg.data = status
        self.mode_status_pub_.publish(msg)

    def send_data(self, data_key):
        try:
            command_data = []
            if data_key == "auto":
                command_data = ["sudo", "i2cset", "-y", "0", "0x40", "0x01", "0x07"]
            elif data_key == "mrm":
                command_data = ["sudo", "i2cset", "-y", "0", "0x40", "0x01", "0x01"]
            elif data_key == "experiment":
                command_data = ["sudo", "i2cset", "-y", "0", "0x40", "0x01", "0x03"]
            elif data_key == "null":
                command_data = ["sudo", "i2cset", "-y", "0", "0x40", "0x01", "0x00"]

            self.node.get_logger().info(str(command_data))

            if len(command_data) == 0:
                return
            result = subprocess.run(
                command_data,
                capture_output=True,
                text=True
            )

            self.node.get_logger().info(str(result.stdout))
            self.node.get_logger().info(str(result.stderr))
            self.node.get_logger().info(str(result.returncode))
        except Exception as e:
            self.node.get_logger().error(str(e))

        # 出力コマンド一覧
        # sudo i2cset -y 0 0x40 0x01 0x00 #空欄
        # sudo i2cset -y 0 0x40 0x01 0x01 #緊急停止中
        # sudo i2cset -y 0 0x40 0x01 0x02 #乗降中
        # sudo i2cset -y 0 0x40 0x01 0x03 #実証実験中
        # sudo i2cset -y 0 0x40 0x01 0x04 #SOS
        # sudo i2cset -y 0 0x40 0x01 0x05 #低速走行中
        # sudo i2cset -y 0 0x40 0x01 0x06 #回送中
        # sudo i2cset -y 0 0x40 0x01 0x07 #自動運行中




    # Lv4のときは自動運転中かどうかで表示を変更する。Lv2のときは「実験中」を固定で表示する
    def trigger_external_signage(self, request, response):
        try:
            # operation modeが変わったときにautowareが起動したと判断する
            self.autoware_status ["driving"] = request.data
            if self._settings["in_experiment"]:
                self.display_signage("experiment")
                return response
            elif self._settings["airport"]:
                if self.autoware_status["mrm"]:
                    self.display_signage("mrm")
                else:
                    if request.data:
                        self.display_signage("auto")
                    else:
                        self.display_signage("null")
            else:            
                if request.data:
                    self.display_signage("auto")
                else:
                    self.display_signage("null")
            response.success = True
        except Exception as e:
            self.node.get_logger().error(str(e))
        return response

    # MRMが発生していて空港モードの場合は「緊急停止中」表示にする
    def sub_mrm_callback(self, msg):
        try:
            if self._settings["in_experiment"]:
                return            
            self.autoware_status["mrm"] = msg.state in [2,3,4]
            if self._settings["airport"] and self.autoware_status["mrm"]:
                self.display_signage("mrm")
            else:
                if self.autoware_status["driving"]:
                    self.display_signage("auto")
                else:
                    self.display_signage("null")
        except Exception as e:
            self.node.get_logger().error("Unable to get the mrm, ERROR: " + str(e))

    # operation modeを受け取ったとき起動したと判断する
    def sub_operation_mode_callback(self, msg):
        try:
            # operation modeが変わったときにautowareが起動したと判断する
            self.is_autoware_launch = True
            self.node.get_logger().info(str(msg.mode))
        except Exception as e:
            self.node.get_logger().error("Unable to get the operation mode, ERROR: " + str(e))

    # l4かどうかのtopicを受け取り走行モードを変更する
    def sub_is_driving_level(self, msg):
        try:
            self.node.get_logger().info(str(msg.is_level4_driving))
            if msg.is_level4_driving: # True is L4, False is L2.
                self.pub_mode_status(True)
                self._settings["in_experiment"] = False
                if self.autoware_status["driving"]:
                    self.display_signage("auto")
                else:
                    self.display_signage("null")
            else:
                self.pub_mode_status(False)
                self._settings["in_experiment"] = True
                self.display_signage("experiment")

            with open(self._settings_file, "w") as f:
                json.dump(self._settings, f, indent=4)
        except Exception as e:
            self.node.get_logger().error(str(e))

    # l4かどうかのサービスを受け取り走行モードを変更する
    def change_mode(self, request, response):
        try:
            if request.data: # True is L2, False is L4.
                self.pub_mode_status(True)
                self._settings["in_experiment"] = True
                self.display_signage("experiment")
            else:
                self.pub_mode_status(False)
                self._settings["in_experiment"] = False
                self.display_signage("null")

            with open(self._settings_file, "w") as f:
                json.dump(self._settings, f, indent=4)
            response.success = True
        except Exception as e:
            self.node.get_logger().error(str(e))
            response.success = False
        return response

    # 空港モードにするかどうかのトピックを受け取り空港モードを変更する
    def change_airport_mode(self, request, response):
        try:
            if request.data:
                self._settings["airport"] = True
            else:
                self._settings["airport"] = False
            with open(self._settings_file, "w") as f:
                json.dump(self._settings, f, indent=4)
            response.success = True
        except Exception as e:
            self.node.get_logger().error(str(e))
            response.success = False
        return response


    def display_signage(self, display_file, force=False):
        # 車外サイネージが準備中かautowareが起動してない場合は表示を変更しない。forceフラグがあるときはautowareの起動関係なく更新する
        if not self.is_autoware_launch and not force:
            return

        self.send_data(display_file)
