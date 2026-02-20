# tests/external_signage/test_packet_tools.py
import datetime
import pytest
from external_signage.packet_tools import (
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
        assert calc_sum([0x10]) == [0x10, 0x00]

    def test_multiple_bytes(self):
        assert calc_sum([0x01, 0x02, 0x03]) == [0x06, 0x00]

    def test_overflow(self):
        assert calc_sum([0xFF, 0x01]) == [0x00, 0x01]


class TestVerifySum:
    def test_valid(self):
        payload = [0x01, 0x02, 0x03]
        checksum = calc_sum(payload)
        packet = [SOT] + payload + checksum + [EOT]
        assert verify_sum(packet) is True

    def test_invalid(self):
        packet = [SOT, 0x01, 0x02, 0x03, 0xFF, 0xFF, EOT]
        assert verify_sum(packet) is False


class TestGenDataPacket:
    def test_structure(self):
        result = gen_data_packet([0x01], seq=0, addr1=0x70, addr2=0x8F)
        assert result[0] == SOT
        assert result[-1] == EOT
        assert verify_sum(result) is True

    def test_length_field(self):
        data = [0x01, 0x02, 0x03]
        result = gen_data_packet(data, seq=0, addr1=0x70, addr2=0x8F)
        assert result[3] == len(data) + 8


class TestGenDataPackets:
    def test_single_packet(self):
        assert len(gen_data_packets(list(range(10)), 0x70, 0x8F)) == 1

    def test_multiple_packets(self):
        assert len(gen_data_packets(list(range(256)), 0x70, 0x8F)) == 2

    def test_all_valid_checksums(self):
        for p in gen_data_packets(list(range(300)), 0x70, 0x8F):
            assert verify_sum(p) is True


class TestGenNameTimePacket:
    def test_valid(self):
        result = gen_name_time_packet(
            b"1234567890123456", datetime.datetime(2026, 1, 15, 10, 30), False
        )
        assert result[0] == SOT
        assert result[-1] == EOT
        assert verify_sum(result) is True

    def test_invalid_length(self):
        with pytest.raises(ValueError):
            gen_name_time_packet(b"short", datetime.datetime.now(), False)


class TestListsMatch:
    def test_equal(self):
        assert lists_match([1, 2], [1, 2]) is True

    def test_not_equal(self):
        assert lists_match([1, 2], [1, 3]) is False

    def test_different_length(self):
        assert lists_match([1], [1, 2]) is False

    def test_empty(self):
        assert lists_match([], []) is True


class TestParser:
    def test_complete_packet(self):
        payload = [0x70, 0x8F, 0x07, 0x12, 0x02]
        checksum = calc_sum(payload)
        packet = [SOT] + payload + checksum + [EOT]
        parser = Parser(bus=None)
        results = [parser.parse(b) for b in packet]
        assert 1 in results

    def test_non_sot_ignored(self):
        parser = Parser(bus=None)
        assert parser.parse(0x00) == 0
        assert parser.state == 0
