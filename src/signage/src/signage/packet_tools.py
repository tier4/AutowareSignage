import datetime
import operator
from functools import reduce

# Packet start and end markers
SOT = 0xAA
EOT = 0x55


def dump_packet(packet, timestamp=None, color=None):
    """
    Prints a packet's contents in a formatted manner, optionally with color and timestamp.

    Parameters:
    - packet: List[int], the packet data including start and end markers.
    - timestamp: datetime.datetime, optional timestamp to prepend to the output.
    - color: str, an optional terminal color code to apply to the output.
    """
    verified = verify_sum(packet)
    if color:
        print(color, end="")
    if timestamp:
        print(timestamp, end=" ")
    for p in packet:
        print(f"{p:02x}", end=" ")
    if color:
        print("\033[0m", end="")
    print()


def calc_sum(payload):
    """
    Calculates the checksum of a given payload.

    Parameters:
    - payload: List[int], the payload data from which to calculate the checksum.

    Returns:
    - List[int], a two-element list containing the checksum bytes.
    """
    s = reduce(operator.add, payload)
    return [s & 0xFF, (s & 0xFF00) >> 8]


def verify_sum(packet):
    """
    Verifies the checksum of a packet.

    Parameters:
    - packet: List[int], the complete packet including checksum and markers.

    Returns:
    - bool, True if the checksum is correct, False otherwise.
    """
    sum = calc_sum(packet[1:-3])
    return (packet[-3] == sum[0]) and (packet[-2] == sum[1])


def gen_data_packet(data, seq, addr1, addr2):
    """
    Generates a single data packet from given data and sequence number.

    Parameters:
    - data: List[int], the data to include in the packet.
    - seq: int, the sequence number of the packet.
    - addr1: int, the first part of the address.
    - addr2: int, the second part of the address.

    Returns:
    - List[int], the assembled packet.
    """
    length = len(data) + 8
    cmd = 0x20
    payload = [addr1, addr2, length, cmd, seq, 0x00] + list(data)
    checksum = calc_sum(payload)
    return [SOT] + payload + checksum + [EOT]


def gen_data_packets(data, addr1, addr2):
    """
    Generates a sequence of data packets from a larger dataset.

    Parameters:
    - data: List[int], the complete set of data to be sent in packets.
    - addr1: int, the first part of the address for the packets.
    - addr2: int, the second part of the address for the packets.

    Returns:
    - List[List[int]], a list of assembled packets.
    """
    packets = []
    seq = 0
    for i in range(0, len(data), 128):
        packet_data = data[i : i + 128]
        packets.append(gen_data_packet(packet_data, seq, addr1, addr2))
        seq += 1
    return packets


def gen_name_time_packet(linename, timestamp, nightmode):
    """
    Generates a packet containing a line name and the current time.

    Parameters:
    - linename: str, the name of the line, must be 16 characters.
    - timestamp: datetime.datetime, the current time.
    - nightmode: bool, whether night mode is enabled.

    Returns:
    - List[int], the assembled packet.
    """
    addr1 = 0x60
    addr2 = 0x9F
    length = 0x24
    cmd = 0x01
    if len(linename) != 16:
        raise ValueError("Line Name length invalid")
    payload = [addr1, addr2, length, cmd] + list(map(ord, linename))
    payload += map(lambda x: int(x, 16), timestamp.strftime("%M %H %d %w %m %y").split())
    payload.append(0x10 if nightmode else 0x00)
    payload.extend([0x00] * 7)
    checksum = calc_sum(payload)
    return [SOT] + payload + checksum + [EOT]


def lists_match(l1, l2):
    """
    Compares two lists for equality.

    Parameters:
    - l1: List, the first list to compare.
    - l2: List, the second list to compare.

    Returns:
    - bool, True if the lists are equal, False otherwise.
    """
    return len(l1) == len(l2) and all(x == y for x, y in zip(l1, l2))


class td5_data:
    """
    Represents data extracted from a .td5 file, including line name, data packets, and heartbeat packet.

    Attributes:
    - width: int, the width of the display.
    - height: int, the height of the display.
    - linename: bytes, the name of the line extracted from the file.
    - data_packets: List[List[int]], the data packets ready for transmission.
    - heartbeat_packet: List[int], the heartbeat packet for maintaining connection.
    """

    def __init__(self, filename, addr1, addr2, height, width):
        self.width = width
        self.height = height
        with open(filename, "rb") as f:
            f.seek(0x400)
            self.linename = f.read(16)
            f.seek(0x61C)
            f.seek(0x607)
            raw_linedatalen = f.read(2)
            self.linedatalen = int.from_bytes(raw_linedatalen, "little")
            f.seek(0x600)
            linedata = f.read(self.linedatalen)
            self.data_packets = gen_data_packets(linedata, addr1, addr2)
            f.seek(0x603)
            data = f.read(2)
            self.heartbeat_packet = self.gen_heartbeat_packet(
                data, raw_linedatalen, height, width, addr1, addr2
            )

    def gen_heartbeat_packet(self, data, linedatalen, height, width, addr1, addr2):
        """
        Generates a heartbeat packet based on the given parameters.

        Parameters:
        - data: bytes, part of the data extracted from the .td5 file.
        - linedatalen: bytes, the length of the line data.
        - height: int, the height of the display.
        - width: int, the width of the display.
        - addr1: int, the first part of the display address.
        - addr2: int, the second part of the display address.

        Returns:
        - List[int], the assembled heartbeat packet.
        """
        cmd1 = 0x16
        cmd2 = 0x12
        payload = [addr1, addr2, cmd1, cmd2]
        payload.extend([0x00, 0x00, height, 0x00])
        payload.extend(linedatalen)
        payload.extend([0x00, 0x00])
        payload.extend(data)
        payload.extend([0x00, 0x00, 0x53, 0x30, 0x30, 0x35])
        checksum = calc_sum(payload)
        return [SOT] + payload + checksum + [EOT]


class Parser:
    """
    Parses incoming data from a serial bus into complete packets.

    Attributes:
    - bus: serial.Serial, the serial bus from which to read data.
    """

    def __init__(self, bus):
        self.state = 0
        self.buf = []
        self.i = 0
        self.datalen = 0
        self.bus = bus

    def read(self):
        """
        Reads a single byte from the serial bus.

        Returns:
        - int, the byte read, or None if no data is available.
        """
        return self.bus.read(1)

    def parse(self, c):
        """
        Parse a single byte and update the state machine.

        Parameters:
        - c: int, the byte to parse.

        Returns:
        - int: 1 if a complete packet is parsed successfully, 0 otherwise.
        """
        if self.state == 0 and c == 0xAA:  # Start of packet
            self.buf = [c]
            self.state = 1
        elif self.state == 1:  # Reading packet header
            self.buf.append(c)
            self.i += 1
            if self.i == 2:  # Length byte next
                self.state = 2
        elif self.state == 2:  # Reading packet length
            self.buf.append(c)
            self.datalen = int(c)
            self.state = 3
            self.i = 0
        elif self.state == 3:  # Reading packet payload
            self.buf.append(c)
            self.i += 1
            if self.i == (self.datalen - 2):  # Packet complete
                self.state = 0
                return 1  # Indicate a complete packet has been parsed
        return 0  # Indicate parsing is ongoing

    def wait_ack(self):
        """
        Wait for an acknowledgement packet.

        Returns:
        - list: The received acknowledgement packet, or an empty list if none is received.
        """
        while True:
            received = self.read()
            if not received or len(received) == 0:
                break  # No more data available
            c = received[0]
            if self.parse(c):
                return self.buf  # Return the complete packet
        return []  # No acknowledgement received
