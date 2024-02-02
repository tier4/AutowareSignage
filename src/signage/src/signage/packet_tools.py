import datetime
import operator
from functools import reduce


def dump_packet(packet, timestamp=None, color=None):
    """
    Display packet on stdout. The data displayed is splited by spaces as follows.
    date time checksum_verified(True or False) data0 data1 ...

    Parameters
    ----------
    packet : list
        list of int. raw packet include from SOT to EOT
    timestamp : datetime.datetime
        timestamp to show
    color : str
        Escape sequence to change character color
    """
    verified = verify_sum(packet)

    if color != None:
        print(color, end="")
    if timestamp != None:
        print(timestamp, end=" ")
    # print(verified, end=" ")
    for p in packet:
        print(f"{p:02x}", end=" ")
    if color != None:
        print("\033[0m", end="")
    print()


def calc_sum(payload):
    """
    calculate checksum from payload.

    Parameters
    ----------
    payload : list
        list of int. from address to data.

    Returns
    -------
    sum : list
        list of int. 2 bytes checksum in network byte order.
    """
    s = reduce(operator.add, payload)
    sum = [s & 0xFF, (s & 0xFF00) >> 8]
    return sum


def verify_sum(packet):
    """
    verify the checksum of a packet.

    Parameters
    ----------
    packet : list
        list of int. raw packet include from SOT to EOT

    Returns
    -------
    verified : bool
        True if checksum is valid.
    """
    sum = calc_sum(packet[1 : len(packet) - 3])
    return (packet[len(packet) - 3] == sum[0]) & (packet[len(packet) - 2] == sum[1])


def gen_data_packet(data, seq, addr1, addr2):
    """
    Generate one of packets to send a image

    Parameters
    ----------
    data : list
        raw data. must be 128 bytes and lower.
    seq : int
        sequence number of packet.
    addr1 : int
        address of screen
    addr2 : int
        address of screen

    Returns
    -------
    packet : list
        generated packet to send.
    """
    length = len(data) + 8
    cmd = 0x20
    payload = [addr1, addr2, length, cmd, seq, 0x00]
    payload.extend(list(data))

    sum = calc_sum(payload)
    packet = [SOT]
    packet.extend(payload)
    packet.extend(sum)
    packet.append(EOT)
    return packet


def gen_data_packets(data, addr1, addr2):
    """
    Generate packets to send a image

    Parameters
    ----------
    data : list
        raw data. part of td5 file.
    addr1 : int
        address of screen
    addr2 : int
        address of screen

    Returns
    -------
    packets : list
        list of list. generated packets to send.
    """
    packets = []
    seq = 0
    i = 0
    while (len(data) - i) > 128:
        packets.append(gen_data_packet(data[i : i + 128], seq, addr1, addr2))
        seq += 1
        i += 128
    packets.append(gen_data_packet(data[i : len(data)], seq, addr1, addr2))
    return packets


def gen_name_time_packet(linename, timestamp, nightmode):
    """
    Generate a packet of Line Name and current time.

    Parameters
    ----------
    linename : str
        linename.
    timestamp : datetime.datetime
        current time to embed on packet.

    Returns
    -------
    packet : list
        generated packet to send.
    """
    addr1 = 0x60
    addr2 = 0x9F
    length = 0x24
    cmd = 0x01
    payload = [addr1, addr2, length, cmd]
    if len(linename) != 16:
        raise ValueError("Line Name length invalid")
    payload.extend(list(linename))
    payload.extend(map(lambda x: int(x, 16), timestamp.strftime("%M %H %d %w %m %y").split()))
    payload.append(0x10 if nightmode else 0x00)
    payload.extend([00, 00, 00, 00, 00, 00, 00])

    sum = calc_sum(payload)
    packet = [SOT]
    packet.extend(payload)
    packet.extend(sum)
    packet.append(EOT)
    return packet


class td5_data:
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

    def gen_heartbeat_packet(data, linedatalen, height, width, addr1, addr2):
        """
        Generate a packet of heartbeat

        Parameters
        ----------
        data : list
            raw data, which can be got from td5 file.
        addr1 : int
            address of screen
        addr2 : int
            address of screen

        Returns
        -------
        packet : list
            generated packet to send.
        """
        cmd1 = 0x16
        cmd2 = 0x12
        payload = [addr1, addr2, cmd1, cmd2]
        payload.extend([0x00, 0x00, height, 0x00])
        payload.extend(linedatalen)
        payload.extend([0x00, 0x00])
        payload.extend(data)
        payload.extend([0x00, 0x00, 0x53, 0x30, 0x30, 0x35])

        sum = calc_sum(payload)
        packet = [SOT]
        packet.extend(payload)
        packet.extend(sum)
        packet.append(EOT)
        return packet


class parser:
    def __init__(self, bus):
        self.state = 0
        self.buf = []
        self.i = 0
        self.datalen = 0
        self.bus = bus

    def read(self):
        return self.bus.read(1)

    def parse(self, c):
        if self.state == 0:
            if c == 0xAA:
                self.buf = []
                self.buf.append(c)
                self.state = 1
                self.i = 0
            else:
                pass
        elif self.state == 1:
            self.buf.append(c)
            self.i += 1
            if self.i == 2:
                self.state = 2
        elif self.state == 2:
            self.buf.append(c)
            self.datalen = int(c)
            self.state = 3
            self.i = 0
        elif self.state == 3:
            self.buf.append(c)
            self.i += 1
            if self.i == (self.datalen - 2):
                self.state = 0
                return 1
        return 0

    def wait_ack(self):
        while True:
            received = self.read()
            if (received == None) or (len(received) == 0):
                break
            c = received[0]
            if self.parse(c):
                timestamp = datetime.datetime.now()
                return self.buf
        return []


def lists_match(l1, l2):
    if len(l1) != len(l2):
        return False
    return all(x == y and type(x) == type(y) for x, y in zip(l1, l2))
