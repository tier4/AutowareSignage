import serial
import datetime
import time
import sys
import signage.packet_tools as packet_tools
import signal


@dataclass
class ConstantProtocol:
    # Start of Transmission, End of Transmission
    SOT = 0xAA
    EOT = 0x55

    # Address, ACK setting
    FRONT_ADDR1 = 0x70
    FRONT_ADDR2 = 0x8F
    FRONT_ACK_QueryACK = [0xAA, FRONT_ADDR1, FRONT_ADDR2, 0x07, 0x12, 0x02, 0x1A, 0x01, 0x55]
    FRONT_ACK_DataChunk = [0xAA, FRONT_ADDR1, FRONT_ADDR2, 0x07, 0x20, 0x30, 0x56, 0x01, 0x55]
    FRONT_HEIGHT = 16
    FRONT_WIDTH = 128

    BACK_ADDR1 = 0x80
    BACK_ADDR2 = 0x7F
    BACK_ACK_QueryACK = [0xAA, BACK_ADDR1, BACK_ADDR2, 0x07, 0x12, 0x02, 0x1A, 0x01, 0x55]
    BACK_ACK_DataChunk = [0xAA, BACK_ADDR1, BACK_ADDR2, 0x07, 0x20, 0x30, 0x56, 0x01, 0x55]
    BACK_HEIGHT = 16
    BACK_WIDTH = 128

    SIDE_ADDR1 = 0x90
    SIDE_ADDR2 = 0x6F
    SIDE_ACK_QueryACK = [0xAA, SIDE_ADDR1, SIDE_ADDR2, 0x07, 0x12, 0x02, 0x1A, 0x01, 0x55]
    SIDE_ACK_DataChunk = [0xAA, SIDE_ADDR1, SIDE_ADDR2, 0x07, 0x20, 0x30, 0x56, 0x01, 0x55]
    SIDE_HEIGHT = 16
    SIDE_WIDTH = 128

    # Color Code
    SEND_COLOR = "\x1B[34;1m"
    RECV_COLOR = "\x1B[32;1m"
    ERR_COLOR = "\x1B[31;1m"


count_hb = 2


def heartbeat_handler(signum, frame):
    global count_hb, count_fig
    count_hb += 1


class ExternalSignage:
    def __init__(self, node):
        self._node = node
        self.protocol = ConstantProtocol()
        package_path = get_package_share_directory("signage") + "/resource/td5_file/"
        self._bus = serial.Serial(
            "/dev/ttyUSB0", baudrate=38400, parity=serial.PARITY_EVEN, timeout=0.2, exclusive=False
        )
        self._parser = packet_tools.parser(bus)
        self._FRONT_AUTO = packet_tools.td5_data(
            package_path + "/automatic_128x16.td5",
            self.protocol.FRONT_ADDR1,
            self.protocol.FRONT_ADDR2,
            self.protocol.FRONT_HEIGHT,
            self.protocol.FRONT_WIDTH,
        )
        self._BACK_AUTO = packet_tools.td5_data(
            package_path + "/automatic_80x24.td5",
            self.protocol.BACK_ADDR1,
            self.protocol.BACK_ADDR2,
            self.protocol.BACK_HEIGHT,
            self.protocol.BACK_WIDTH,
        )
        self._SIDE_AUTO = packet_tools.td5_data(
            package_path + "/automatic_128x16.td5",
            self.protocol.SIDE_ADDR1,
            self.protocol.SIDE_ADDR2,
            self.protocol.SIDE_HEIGHT,
            self.protocol.SIDE_WIDTH,
        )

        self._FRONT_NULL = packet_tools.td5_data(
            package_path + "/null_128x16.td5",
            self.protocol.FRONT_ADDR1,
            self.protocol.FRONT_ADDR2,
            self.protocol.FRONT_HEIGHT,
            self.protocol.FRONT_WIDTH,
        )
        self._BACK_NULL = packet_tools.td5_data(
            package_path + "/null_128x16.td5",
            self.protocol.BACK_ADDR1,
            self.protocol.BACK_ADDR2,
            self.protocol.BACK_HEIGHT,
            self.protocol.BACK_WIDTH,
        )
        self._SIDE_NULL = packet_tools.td5_data(
            package_path + "/null_80x24.td5",
            self.protocol.SIDE_ADDR1,
            self.protocol.SIDE_ADDR2,
            self.protocol.SIDE_HEIGHT,
            self.protocol.SIDE_WIDTH,
        )

        signal.signal(signal.SIGALRM, heartbeat_handler)
        signal.setitimer(signal.ITIMER_REAL, 1, 1)

    def send_data(self, data, ACK_QueryACK, ACK_DataChunk):
        stop_hb = False
        sent = False
        nightmode = False
        delaytime = 0.02
        while True:
            if (count_hb >= 2) and (stop_hb != True):
                timestamp = datetime.datetime.now()
                name_time_packet = packet_tools.gen_name_time_packet(
                    data.linename, timestamp, nightmode
                )
                self._bus.write(name_time_packet)
                packet_tools.dump_packet(name_time_packet, None, self.protocol.SEND_COLOR)
                time.sleep(delaytime)
                self._bus.write(data.heartbeat_packet)
                packet_tools.dump_packet(data.heartbeat_packet, None, self.protocol.SEND_COLOR)
                buf = self._parser.wait_ack()
                if packet_tools.lists_match(buf, ACK_QueryACK):
                    packet_tools.dump_packet(buf, None, self.protocol.RECV_COLOR)
                else:
                    if len(buf) == 0:
                        self._node.get_logger().error("error")
                    else:
                        packet_tools.dump_packet(buf, None, self.protocol.ERR_COLOR)
                count_hb = 0

            if sent != True:
                for packet in data.data_packets:
                    packet_tools.dump_packet(packet, None, self.protocol.SEND_COLOR)
                    self._bus.write(packet)
                    buf = self._parser.wait_ack()
                    if lists_match(buf, ACK_DataChunk):
                        packet_tools.dump_packet(buf, None, self.protocol.RECV_COLOR)
                    else:
                        if len(buf) == 0:
                            self._node.get_logger().error("error2")
                        else:
                            packet_tools.dump_packet(buf, None, self.protocol.ERR_COLOR)
                sent = True

            if sent:
                return

    def trigger(self):
        self.send_data(
            self._FRONT_AUTO, self.protocol.FRONT_ACK_QueryACK, self.protocol.FRONT_ACK_DataChunk
        )
        self.send_data(
            self._BACK_AUTO, self.protocol.BACK_ACK_QueryACK, self.protocol.BACK_ACK_DataChunk
        )
        self.send_data(
            self._SIDE_AUTO, self.protocol.SIDE_ACK_QueryACK, self.protocol.SIDE_ACK_DataChunk
        )

    def close(self):
        self.send_data(
            self._FRONT_NULL, self.protocol.FRONT_ACK_QueryACK, self.protocol.FRONT_ACK_DataChunk
        )
        self.send_data(
            self._BACK_NULL, self.protocol.BACK_ACK_QueryACK, self.protocol.BACK_ACK_DataChunk
        )
        self.send_data(
            self._SIDE_NULL, self.protocol.SIDE_ACK_QueryACK, self.protocol.SIDE_ACK_DataChunk
        )
