from dataclasses import dataclass
import datetime
import time
import serial
import json
import os
import uuid
import yaml
import rclpy
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory
import external_signage.packet_tools as packet_tools
from autoware_adapi_v1_msgs.msg import MrmState
from std_msgs.msg import String
from level4_mode_manager_msgs.msg import Level4DrivingStatus

# SYS-HMI-03: destination.point_id -> td5ファイル名prefix の紐付けを記載する外部ファイル。
# signage_settings.json と同様に /opt/autoware 下へ置き、リビルド不要で運用時に編集できる。
# 存在しなければパッケージ同梱テンプレート (config/destination_mapping.yaml) から生成する。
DESTINATION_MAPPING_PATH = "/opt/autoware/destination_mapping.yaml"


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
    SEND_COLOR = "\x1b[34;1m"
    RECV_COLOR = "\x1b[32;1m"
    ERR_COLOR = "\x1b[31;1m"

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
        self.current_state = ""

        package_path = get_package_share_directory("external_signage") + "/resource/td5_file/"
        node.declare_parameter("serial_port", "/dev/ttyS0")
        self._serial_port = node.get_parameter("serial_port").get_parameter_value().string_value

        # SYS-HMI-03: 行先表示用の destination.point_id -> td5ファイル名prefix 対応を起動時にロードする
        self._destination_mapping = self._load_destination_mapping()
        self._last_schedule_raw = ""

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

        # ros interface
        api_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        airport_mode_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        )

        node.create_service(SetBool, "/signage/trigger_external", self.trigger_external_signage)
        node.create_service(SetBool, "/signage/mode_change", self.change_mode)
        self._sub_airport_mode = node.create_subscription(
            Bool, "/signage/airport_mode", self.change_airport_mode, airport_mode_qos
        )
        self.mode_status_pub_ = node.create_publisher(Bool, "/signage/mode_status", api_qos)
        self.setting_pub_ = node.create_publisher(String, "/signage/external/settings", api_qos)
        self._sub_mrm = node.create_subscription(
            MrmState,
            "/api/fail_safe/mrm_state",
            self.sub_mrm_callback,
            api_qos,
        )

        self._sub_is_driving_level = node.create_subscription(
            Level4DrivingStatus,
            "/level4_mode_manager/is_level4_driving",
            self.sub_is_driving_level,
            api_qos,
        )

        # SYS-HMI-03: 行先表示モードの切替サービスと、行先決定に用いる active_schedule 購読
        # active_schedule の publisher (signage_fms_client) は RELIABLE + VOLATILE (depth10) のため
        # durability を VOLATILE で合わせる (api_qos の TRANSIENT_LOCAL だと不整合で受信できない)。
        schedule_qos = rclpy.qos.QoSProfile(
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
        )
        node.create_service(SetBool, "/signage/destination_mode", self.set_destination_mode)
        self._sub_active_schedule = node.create_subscription(
            String, "/signage/active_schedule", self.sub_active_schedule, schedule_qos
        )

        # read settings.If not, creatte settings.
        self._settings_file = "/home/" + os.environ.get("USER") + "/settings.json"
        if os.path.exists(self._settings_file):
            with open(self._settings_file, "r") as f:
                self._settings = json.load(f)
        else:
            self._settings = {"in_experiment": True, "airport": False}
        # SYS-HMI-03: 行先表示モード (in_experiment/airport と排他) の既定値を補完し永続化する
        self._settings.setdefault("destination_mode", False)
        self._save_settings()

        # initial display
        if self._settings.get("destination_mode"):
            # 行先表示モード: active_schedule 受信までは null (空白) を表示
            self.pub_mode_status(False)
            self._render_destination()
        elif self._settings.get("in_experiment", True):
            self.pub_mode_status(True)
            self.display_signage("experiment")
        else:
            self.pub_mode_status(False)
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

    def _load_destination_mapping(self):
        # destination.point_id -> td5ファイル名prefix の紐付けを外部ファイル (/opt/autoware) から読む。
        # 無ければパッケージ同梱テンプレートから生成する。壊れている場合は空マッピングで継続する
        # (フォールバックで null 表示)。運用時はこの外部ファイルを編集すればリビルド不要で反映される。
        # point_id は整数だが YAML キーの引用有無で int/str が揺れるため、キーを str に正規化して保持する。
        try:
            if not os.path.isfile(DESTINATION_MAPPING_PATH):
                self._seed_destination_mapping()
            with open(DESTINATION_MAPPING_PATH, "r") as f:
                data = yaml.safe_load(f) or {}
            raw_mapping = data.get("destination_mapping", {}) or {}
            mapping = {str(k): v for k, v in raw_mapping.items()}
            self.node.get_logger().info(
                "loaded destination_mapping from {}: {} entries".format(
                    DESTINATION_MAPPING_PATH, len(mapping)
                )
            )
            return mapping
        except Exception as e:
            self.node.get_logger().warning(
                "destination_mapping load failed ({}), use empty mapping: {}".format(
                    DESTINATION_MAPPING_PATH, str(e)
                )
            )
            return {}

    def _seed_destination_mapping(self):
        # 初回のみ: パッケージ同梱テンプレートを /opt/autoware へコピーして初期化する。
        template = (
            get_package_share_directory("external_signage") + "/config/destination_mapping.yaml"
        )
        os.makedirs(os.path.dirname(DESTINATION_MAPPING_PATH), exist_ok=True)
        with open(template, "r") as src, open(DESTINATION_MAPPING_PATH, "w") as dst:
            dst.write(src.read())
        self.node.get_logger().info(
            "seeded destination_mapping template to {}".format(DESTINATION_MAPPING_PATH)
        )

    def _load_td5(self, path, display):
        return packet_tools.TD5Data(
            path, display.address1, display.address2, display.height, display.width
        )

    def _load_display_data(self, display, package_path):
        auto_path = package_path + f"automatic_{display.width}x{display.height}.td5"
        experiment_path = package_path + f"experiment_{display.width}x{display.height}.td5"
        mrm_path = package_path + f"mrm_{display.width}x{display.height}.td5"
        null_path = package_path + f"null_{display.width}x{display.height}.td5"
        data = {
            "auto": self._load_td5(auto_path, display),
            "experiment": self._load_td5(experiment_path, display),
            "mrm": self._load_td5(mrm_path, display),
            "null": self._load_td5(null_path, display),
        }

        # SYS-HMI-03: 行先td5(mappingのprefix)と回送中(kaiso)td5を一括ロードしキャッシュする。
        # 未配置/ロード失敗のファイルはスキップし、表示時に null へフォールバックする。
        suffix = f"_{display.width}x{display.height}.td5"
        for dest_id, prefix in self._destination_mapping.items():
            path = package_path + "destination/" + f"{prefix}{suffix}"
            try:
                data[prefix] = self._load_td5(path, display)
            except Exception as e:
                self.node.get_logger().warning(
                    "destination td5 load failed ({} -> {}, {}x{}): {}".format(
                        dest_id, prefix, display.width, display.height, str(e)
                    )
                )
        try:
            data["kaiso"] = self._load_td5(package_path + f"kaiso{suffix}", display)
        except Exception as e:
            self.node.get_logger().warning(
                "kaiso td5 load failed ({}x{}): {}".format(display.width, display.height, str(e))
            )
        return data

    def send_data(self, display_key, data_key):
        display = self.displays[display_key]
        data = display[data_key]
        ack_query_ack = self.protocol.__dict__[display_key].ack_query_ack
        ack_data_chunk = self.protocol.__dict__[display_key].ack_data_chunk
        sender = DataSender(self.bus, self.parser, self.protocol, self.node.get_logger())
        sender.send(data, ack_query_ack, ack_data_chunk)

    # Lv4のときは自動運転中かどうかで表示を変更する。Lv2のときは変更しない
    def trigger_external_signage(self, request, response):
        try:
            self.autoware_status["driving"] = request.data
            # SYS-HMI-03: 行先表示モード中は既存表示を出さない (状態のみ更新)
            if self._settings.get("destination_mode"):
                response.success = True
                return response
            if self._settings["in_experiment"]:
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
            self.autoware_status["mrm"] = msg.state in [2, 3, 4]
            # SYS-HMI-03: 行先表示モード中は既存表示(mrm含む)を出さない (状態のみ更新)
            if self._settings.get("destination_mode"):
                return
            if self._settings["in_experiment"]:
                return
            if self._settings["airport"] and self.autoware_status["mrm"]:
                self.display_signage("mrm")
            else:
                if self.autoware_status["driving"]:
                    self.display_signage("auto")
                else:
                    self.display_signage("null")
        except Exception as e:
            self._node.get_logger().error("Unable to get the mrm, ERROR: " + str(e))

    # l4かどうかのtopicを受け取り走行モードを変更する
    def sub_is_driving_level(self, msg):
        try:
            self.node.get_logger().info(str(msg.is_level4_driving))
            # SYS-HMI-03: 行先表示モード中は状態更新のみ行い既存表示は出さない
            skip_display = self._settings.get("destination_mode")
            if msg.is_level4_driving:  # True is L4, False is L2.
                self.pub_mode_status(True)
                self._settings["in_experiment"] = False
                if not skip_display:
                    if self.autoware_status["driving"]:
                        self.display_signage("auto")
                    else:
                        self.display_signage("null")
            else:
                self.pub_mode_status(False)
                self._settings["in_experiment"] = True
                if not skip_display:
                    self.display_signage("experiment")

            self._save_settings()
        except Exception as e:
            self.node.get_logger().error(str(e))

    # l4かどうかのサービスを受け取り走行モードを変更する
    def change_mode(self, request, response):
        try:
            skip_display = self._settings.get("destination_mode")
            if request.data:  # True is L2, False is L4.
                self.pub_mode_status(True)
                self._settings["in_experiment"] = True
                if not skip_display:
                    self.display_signage("experiment")
            else:
                self.pub_mode_status(False)
                self._settings["in_experiment"] = False
                if not skip_display:
                    self.display_signage("null")

            self._save_settings()
            response.success = True
        except Exception as e:
            self.node.get_logger().error(str(e))
            response.success = False
        return response

    # 空港モードにするかどうかのトピックを受け取り空港モードを変更する
    def change_airport_mode(self, msg):
        try:
            self._settings["airport"] = msg.data
            with open(self._settings_file, "w") as f:
                json.dump(self._settings, f, indent=4)
        except Exception as e:
            self.node.get_logger().error(str(e))

    def _save_settings(self):
        with open(self._settings_file, "w") as f:
            json.dump(self._settings, f, indent=4)

    # SYS-HMI-03: 行先表示モードの切替 (in_experiment/airport と排他)
    def set_destination_mode(self, request, response):
        try:
            self._settings["destination_mode"] = request.data
            self._save_settings()
            if request.data:
                # 行先表示モードON: 最新スケジュールから行先td5を描画する
                self._render_destination()
            else:
                # OFF: 現在の autoware_status / settings に従い既存表示へ復帰する
                self._restore_existing_display()
            response.success = True
        except Exception as e:
            self.node.get_logger().error(str(e))
            response.success = False
        return response

    def sub_active_schedule(self, msg):
        # active_schedule は常に保持し、行先表示モード時のみ描画に反映する
        self._last_schedule_raw = msg.data
        if not self._settings.get("destination_mode"):
            return
        self._render_destination()

    def _render_destination(self):
        # 最新の active_schedule から表示すべき td5 状態を決定して描画する。
        try:
            data = json.loads(self._last_schedule_raw) if self._last_schedule_raw else None
        except Exception as e:
            self.node.get_logger().warning("active_schedule JSON parse failed: " + str(e))
            self._display_state("null")
            return
        state_key, detail = self._resolve_destination(data)
        self.node.get_logger().info("destination display -> {} ({})".format(state_key, detail))
        self._display_state(state_key)

    def _resolve_destination(self, data):
        # 行先決定ロジック: doing[0] -> todo[0] -> 有効な行先なし。
        # 戻り値 (state_key, detail)。state_key は displays のデータキー (prefix / "kaiso" / "null")。
        # 有効な行先が取れない状態 (スケジュール無し/未登録/全完了) は回送中 (kaiso) とする (SYS-HMI-08)。
        # 行先表示モードでは走行中の自動運行中(auto)表示は行わない。
        if not data:
            # スケジュール無し/未受信 -> 回送中
            return "kaiso", "no schedule"
        tasks = data.get("tasks", [])
        doing = [t for t in tasks if t.get("task_type") == "move" and t.get("status") == "doing"]
        todo = [t for t in tasks if t.get("task_type") == "move" and t.get("status") == "todo"]
        task = doing[0] if doing else (todo[0] if todo else None)
        if task is None:
            # doing/todo の move タスクなし = スケジュール完了/未登録 -> 回送中
            return "kaiso", "schedule complete"
        dest = task.get("destination") or {}
        point_id = dest.get("point_id")
        dest_name = dest.get("name")
        if point_id is None:
            return "null", "no destination point_id"
        prefix = self._destination_mapping.get(str(point_id))
        if not prefix:
            self.node.get_logger().warning(
                "destination point_id not in mapping: {} ({})".format(point_id, dest_name)
            )
            return "null", "unmapped:{}".format(point_id)
        return prefix, "point_id={} name={}".format(point_id, dest_name)

    def _display_state(self, state_key):
        # 指定キーの td5 が全ディスプレイに揃っていなければ null (空白) にフォールバックする。
        if not all(state_key in self.displays.get(d, {}) for d in self.displays):
            if state_key != "null":
                self.node.get_logger().warning(
                    "td5 not available for '{}', falling back to null".format(state_key)
                )
            state_key = "null"
        self.display_signage(state_key)

    def _restore_existing_display(self):
        # 行先表示モードOFF時に、既存ロジック(実験/空港MRM/自動運転/停止)へ復帰する。
        if self._settings.get("in_experiment", True):
            self.display_signage("experiment")
        elif self._settings.get("airport") and self.autoware_status["mrm"]:
            self.display_signage("mrm")
        elif self.autoware_status["driving"]:
            self.display_signage("auto")
        else:
            self.display_signage("null")

    def display_signage(self, display_file):
        if not self._external_signage_available:
            return

        # 前回の状態と同じの場合更新をスキップする
        previous_state = self.current_state
        self.current_state = display_file
        if previous_state == display_file:
            return

        for display_key in self.displays:
            self.send_data(display_key, display_file)
            time.sleep(1)
