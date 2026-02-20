# tests/signage/test_packet_tools.py
import datetime
import pytest
from signage.packet_tools import (
    SOT,
    EOT,
    calc_sum,
    verify_sum,
    gen_data_packet,
    gen_data_packets,
    gen_name_time_packet,
    lists_match,
    Parser,
)


class TestCalcSum:
    def test_single_byte(self):
        result = calc_sum([0x10])
        assert result == [0x10, 0x00]

    def test_multiple_bytes(self):
        result = calc_sum([0x01, 0x02, 0x03])
        assert result == [0x06, 0x00]

    def test_overflow_low_byte(self):
        result = calc_sum([0xFF, 0x01])
        assert result == [0x00, 0x01]

    def test_large_sum(self):
        result = calc_sum([0xFF, 0xFF])
        assert result == [0xFE, 0x01]


class TestVerifySum:
    def test_valid_packet(self):
        payload = [0x01, 0x02, 0x03]
        checksum = calc_sum(payload)
        packet = [SOT] + payload + checksum + [EOT]
        assert verify_sum(packet) is True

    def test_invalid_packet(self):
        packet = [SOT, 0x01, 0x02, 0x03, 0xFF, 0xFF, EOT]
        assert verify_sum(packet) is False


class TestGenDataPacket:
    def test_basic_packet(self):
        data = [0x01, 0x02]
        result = gen_data_packet(data, seq=0, addr1=0x70, addr2=0x8F)
        assert result[0] == SOT
        assert result[-1] == EOT
        assert result[1] == 0x70  # addr1
        assert result[2] == 0x8F  # addr2
        assert result[3] == 10   # length = len(data) + 8
        assert result[4] == 0x20  # cmd
        assert result[5] == 0     # seq
        assert result[6] == 0x00  # padding
        assert result[7] == 0x01  # data[0]
        assert result[8] == 0x02  # data[1]
        assert verify_sum(result) is True

    def test_sequence_number(self):
        result = gen_data_packet([0x01], seq=5, addr1=0x70, addr2=0x8F)
        assert result[5] == 5

    def test_checksum_valid(self):
        result = gen_data_packet([0x10, 0x20, 0x30], seq=0, addr1=0x60, addr2=0x9F)
        assert verify_sum(result) is True


class TestGenDataPackets:
    def test_single_packet_small_data(self):
        data = list(range(10))
        result = gen_data_packets(data, addr1=0x70, addr2=0x8F)
        assert len(result) == 1

    def test_multiple_packets_large_data(self):
        data = list(range(256))
        result = gen_data_packets(data, addr1=0x70, addr2=0x8F)
        assert len(result) == 2

    def test_exact_128_boundary(self):
        data = list(range(128))
        result = gen_data_packets(data, addr1=0x70, addr2=0x8F)
        assert len(result) == 1

    def test_129_bytes_two_packets(self):
        data = list(range(129))
        result = gen_data_packets(data, addr1=0x70, addr2=0x8F)
        assert len(result) == 2

    def test_sequence_numbers_increment(self):
        data = list(range(256))
        result = gen_data_packets(data, addr1=0x70, addr2=0x8F)
        assert result[0][5] == 0  # first packet seq=0
        assert result[1][5] == 1  # second packet seq=1

    def test_all_packets_valid_checksum(self):
        data = list(range(300))
        for packet in gen_data_packets(data, addr1=0x70, addr2=0x8F):
            assert verify_sum(packet) is True


class TestGenNameTimePacket:
    def test_valid_linename(self):
        linename = b"1234567890123456"
        timestamp = datetime.datetime(2026, 1, 15, 10, 30, 0)
        result = gen_name_time_packet(linename, timestamp, nightmode=False)
        assert result[0] == SOT
        assert result[-1] == EOT
        assert verify_sum(result) is True

    def test_nightmode_on(self):
        linename = b"1234567890123456"
        timestamp = datetime.datetime(2026, 1, 15, 10, 30, 0)
        result = gen_name_time_packet(linename, timestamp, nightmode=True)
        assert verify_sum(result) is True

    def test_invalid_linename_length(self):
        with pytest.raises(ValueError, match="Line Name length invalid"):
            gen_name_time_packet(b"short", datetime.datetime.now(), False)


class TestListsMatch:
    def test_equal_lists(self):
        assert lists_match([1, 2, 3], [1, 2, 3]) is True

    def test_different_lists(self):
        assert lists_match([1, 2, 3], [1, 2, 4]) is False

    def test_different_lengths(self):
        assert lists_match([1, 2], [1, 2, 3]) is False

    def test_empty_lists(self):
        assert lists_match([], []) is True


class TestParser:
    def test_parse_complete_packet(self):
        """Parser should return 1 when a complete packet is parsed."""
        payload = [0x70, 0x8F, 0x07, 0x12, 0x02]
        checksum = calc_sum(payload)
        packet = [SOT] + payload + checksum + [EOT]

        parser = Parser(bus=None)
        results = []
        for byte in packet:
            results.append(parser.parse(byte))

        assert 1 in results

    def test_parse_incomplete_packet(self):
        """Parser should return 0 for incomplete packets."""
        parser = Parser(bus=None)
        assert parser.parse(SOT) == 0
        assert parser.parse(0x70) == 0

    def test_parse_ignores_non_sot_start(self):
        """Parser should ignore bytes that are not SOT when in initial state."""
        parser = Parser(bus=None)
        assert parser.parse(0x00) == 0
        assert parser.parse(0xFF) == 0
        assert parser.state == 0
