from dataclasses import dataclass
import datetime
import time
import serial
import json
import os
import uuid
import rclpy
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
import external_signage.packet_tools as packet_tools
from autoware_adapi_v1_msgs.msg import MrmState, OperationModeState
from std_msgs.msg import String
from level4_mode_manager_msgs.msg import Level4DrivingStatus

@dataclass
class Display:
    address1: int
    address2: int
    height: int
    width: int
    ack_query_ack: list
    ack_data_chunk: list


class Protocol:
    SOT = 0xAA
    EOT = 0x55
    SEND_COLOR = "\x1B[34;1m"
    RECV_COLOR = "\x1B[32;1m"
    ERR_COLOR = "\x1B[31;1m"

    def __init__(self):
        self.front = Display(
            address1=0x70,
            address2=0x8F,
            height=16,
            width=128,
            ack_query_ack=[0xAA, 0x70, 0x8F, 0x07, 0x12, 0x02, 0x1A, 0x01, 0x55],
            ack_data_chunk=[0xAA, 0x70, 0x8F, 0x07, 0x20, 0x30, 0x56, 0x01, 0x55],
        )
        self.back = Display(
            address1=0x80,
            address2=0x7F,
            height=16,
            width=128,
            ack_query_ack=[0xAA, 0x80, 0x7F, 0x07, 0x12, 0x02, 0x1A, 0x01, 0x55],
            ack_data_chunk=[0xAA, 0x80, 0x7F, 0x07, 0x20, 0x30, 0x56, 0x01, 0x55],
        )
        self.side = Display(
            address1=0x90,
            address2=0x6F,
            height=24,
            width=80,
            ack_query_ack=[0xAA, 0x90, 0x6F, 0x07, 0x12, 0x02, 0x1A, 0x01, 0x55],
            ack_data_chunk=[0xAA, 0x90, 0x6F, 0x07, 0x20, 0x30, 0x56, 0x01, 0x55],
        )


class DataSender:
    def __init__(self, bus, parser, protocol, node_logger):
        self._bus = bus
        self._parser = parser
        self._protocol = protocol
        self._logger = node_logger
        self._delay_time = 0.02

    def randomname(self):
        u = uuid.uuid4()
        return u.bytes

    def _send_heartbeat(self, data, ACK_QueryACK):
        timestamp = datetime.datetime.now()
        name_time_packet = packet_tools.gen_name_time_packet(self.randomname(), timestamp, False)
        self._bus.write(name_time_packet)
        packet_tools.dump_packet(name_time_packet, None, self._protocol.SEND_COLOR)
        time.sleep(self._delay_time)
        self._bus.write(data.heartbeat_packet)
        packet_tools.dump_packet(data.heartbeat_packet, None, self._protocol.SEND_COLOR)
        buf = self._parser.wait_ack()
        if not packet_tools.lists_match(buf, ACK_QueryACK):
            if len(buf) == 0:
                self._logger.error("No ACK received for heartbeat.")
            else:
                packet_tools.dump_packet(buf, None, self._protocol.ERR_COLOR)

    def _send_data_packets(self, data, ACK_DataChunk):
        for packet in data.data_packets:
            packet_tools.dump_packet(packet, None, self._protocol.SEND_COLOR)
            self._bus.write(packet)
            buf = self._parser.wait_ack()
            if not packet_tools.lists_match(buf, ACK_DataChunk):
                if len(buf) == 0:
                    self._logger.error("No ACK received for data packet.")
                else:
                    packet_tools.dump_packet(buf, None, self._protocol.ERR_COLOR)

    def send(self, data, ACK_QueryACK, ACK_DataChunk):
        self._send_heartbeat(data, ACK_QueryACK)
        self._send_data_packets(data, ACK_DataChunk)
        return  # Exit after sending all data packets


class ExternalSignage:
    def __init__(self, node):
        self.node = node
        self.protocol = Protocol()

        package_path = get_package_share_directory("external_signage") + "/resource/td5_file/"
        node.declare_parameter("serial_port", "/dev/ttyS0")
        self._serial_port = node.get_parameter("serial_port").get_parameter_value().string_value

        try:
            self.bus = serial.Serial(
                self._serial_port,
                baudrate=38400,
                parity=serial.PARITY_EVEN,
                timeout=0.2,
                exclusive=False,
            )
            self.parser = packet_tools.Parser(self.bus)
            self._external_signage_available = True
        except Exception as e:
            self.node.get_logger().error(str(e))
            self._external_signage_available = False

        try:
            self.displays = {
                "front": self._load_display_data(self.protocol.front, package_path),
                "back": self._load_display_data(self.protocol.back, package_path),
                "side": self._load_display_data(self.protocol.side, package_path),
            }
        except Exception as e:
            self.node.get_logger().error(str(e))

        self.autoware_status = {
            "driving": True,
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

    def _load_display_data(self, display, package_path):
        auto_path = package_path + f"automatic_{display.width}x{display.height}.td5"
        experiment_path = package_path + f"experiment_{display.width}x{display.height}.td5"
        mrm_path = package_path + f"mrm_{display.width}x{display.height}.td5"
        null_path = package_path + f"null_{display.width}x{display.height}.td5"
        return {
            "auto": packet_tools.TD5Data(
                auto_path, display.address1, display.address2, display.height, display.width
            ),
            "experiment": packet_tools.TD5Data(
                experiment_path, display.address1, display.address2, display.height, display.width
            ),
            "mrm": packet_tools.TD5Data(
                mrm_path, display.address1, display.address2, display.height, display.width
            ),            
            "null": packet_tools.TD5Data(
                null_path, display.address1, display.address2, display.height, display.width
            ),
        }

    def send_data(self, display_key, data_key):
        display = self.displays[display_key]
        data = display[data_key]
        ack_query_ack = self.protocol.__dict__[display_key].ack_query_ack
        ack_data_chunk = self.protocol.__dict__[display_key].ack_data_chunk
        sender = DataSender(self.bus, self.parser, self.protocol, self.node.get_logger())
        sender.send(data, ack_query_ack, ack_data_chunk)

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
            self.node.get_logger().info(str(msg.data))
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
        if not self._external_signage_available or (not self.is_autoware_launch and not force):
            return

        for display_key in self.displays:
            self.send_data(display_key, display_file)
            time.sleep(1)