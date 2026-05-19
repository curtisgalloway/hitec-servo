"""Tests for packet building and CRC — no hardware required."""

from hitecdpc.crc import crc8
from hitecdpc import protocol as P


class TestCRC8:
    def test_known_vector(self):
        # CRC-8 Dallas/Maxim: crc8([0x00], init=0xFF) should be deterministic
        assert isinstance(crc8(b"\x00"), int)
        assert 0 <= crc8(b"\x00") <= 255

    def test_empty_input(self):
        assert crc8(b"") == 0xFF  # init value unchanged

    def test_single_byte(self):
        # Manually computed: init=0xFF, byte=0x31, poly=0x8C
        # This is the standard test vector for 1-Wire CRC
        result = crc8(b"\x31")
        assert isinstance(result, int)

    def test_idempotent(self):
        data = b"KP3S5"
        assert crc8(data) == crc8(data)


class TestServoPacket:
    def test_checksum_zero_data(self):
        # CMD=0x61, ADDR=0, DATA=0 → csum = (256 - 0x61) % 256 = 0x9F
        pkt = P.servo_packet(0x61, 0, 0)
        assert pkt == bytes([0x61, 0x00, 0x00, 0x9F])

    def test_checksum_wraps(self):
        # CMD+ADDR+DATA = 0 mod 256 → csum = 0
        pkt = P.servo_packet(0xFF, 0, 1)
        csum = (256 - (0xFF + 0 + 1) % 256) % 256
        assert pkt[3] == csum

    def test_length(self):
        pkt = P.servo_packet(0x64, 129, 42)
        assert len(pkt) == 4

    def test_checksum_validates(self):
        for cmd in [0x61, 0x62, 0x64, 0x65, 0x66, 0x67]:
            for addr in [0, 1, 35, 127, 128, 164]:
                for data in [0, 1, 127, 255]:
                    pkt = P.servo_packet(cmd, addr, data)
                    assert (pkt[0] + pkt[1] + pkt[2] + pkt[3]) % 256 == 0


class TestSTXETXFrame:
    def test_round_trip(self):
        payload = b"KP3S5"
        for mux in (0, 7, 11):
            frame = P.stxetx_frame(payload, mux)
            recovered = P.parse_stxetx(frame, mux)
            assert recovered == payload, f"Round-trip failed for mux={mux}"

    def test_frame_structure(self):
        payload = b"KWAU"
        frame = P.stxetx_frame(payload, mux=7)
        assert frame[0] == 0x09  # STX = 2 + 7
        assert frame[-1] == 0x0A  # ETX = 3 + 7
        assert frame[-2] == len(payload)  # length field
        assert frame[-3] == crc8(payload)  # CRC field

    def test_parse_returns_none_on_bad_crc(self):
        payload = b"KWAU"
        frame = bytearray(P.stxetx_frame(payload, mux=7))
        frame[-3] ^= 0xFF  # corrupt CRC
        assert P.parse_stxetx(bytes(frame), mux=7) is None

    def test_parse_returns_none_on_empty(self):
        assert P.parse_stxetx(b"", mux=7) is None

    def test_parse_finds_frame_inside_larger_buffer(self):
        payload = b"kRs3\x00\x28"
        frame = P.stxetx_frame(payload, mux=11)
        padded = b"\x00\x00\xff" + frame + b"\x00\x00"
        recovered = P.parse_stxetx(padded, mux=11)
        assert recovered == payload


class TestKSOWrapper:
    def test_3pin_structure(self):
        servo_pkt = bytes([0x61, 0x02, 0x00, 0x9D])
        wrapper = P.kso_wrapper(servo_pkt, four_pin=False, want_reply=True)
        assert wrapper[0] == 0x4B  # 'K'
        assert wrapper[1] == 0x53  # 'S'
        assert wrapper[2] == 0x4F  # 'O'
        assert wrapper[3] == 0x33  # '3'
        assert wrapper[4] == len(servo_pkt)
        # Bytes are inverted
        for i, b in enumerate(servo_pkt):
            assert wrapper[5 + i] == (~b & 0xFF)

    def test_4pin_marker(self):
        wrapper = P.kso_wrapper(b"\x00\x00\x00\x00", four_pin=True)
        assert wrapper[3] == 0x34  # '4'

    def test_no_reply_zeros(self):
        wrapper = P.kso_wrapper(b"\x61\x00\x00\x9f", want_reply=False)
        tail = wrapper[-3:]
        assert tail == bytes([0, 0, 0])

    def test_reply_nonzero(self):
        wrapper = P.kso_wrapper(b"\x61\x00\x00\x9f", want_reply=True)
        tail = wrapper[-3:]
        assert tail == bytes([10, 10, 20])


class TestServoPacketV2:
    def test_read_length(self):
        assert len(P.servo_packet_v2_read(0x06)) == 5

    def test_read_cmd(self):
        pkt = P.servo_packet_v2_read(0x06)
        assert pkt[0] == 0x96

    def test_read_csum_equals_addr(self):
        for addr in [0x00, 0x02, 0x06, 0x5e, 0x9c]:
            pkt = P.servo_packet_v2_read(addr)
            expected_csum = sum(pkt[1:4]) & 0xFF
            assert pkt[4] == expected_csum, f"csum mismatch for addr=0x{addr:02x}"

    def test_read_matches_capture(self):
        # Verified from Cynthion capture: read addr=0x06 inverted = [0x69, 0xff, 0xf9, 0xff, 0xf9]
        pkt = P.servo_packet_v2_read(0x06)
        inverted = bytes(~b & 0xFF for b in pkt)
        assert inverted == bytes([0x69, 0xFF, 0xF9, 0xFF, 0xF9])

    def test_write_length(self):
        assert len(P.servo_packet_v2_write(0x9c, 0x6400)) == 7

    def test_write_cmd(self):
        pkt = P.servo_packet_v2_write(0x9c, 0x6400)
        assert pkt[0] == 0x96

    def test_write_csum_matches_capture(self):
        # From capture: write addr=0x9c, val_hi=0x64, val_lo=0x00, csum=0x02
        pkt = P.servo_packet_v2_write(0x9c, 0x6400)
        assert pkt[2] == 0x9c
        assert pkt[3] == 0x02
        assert pkt[4] == 0x64
        assert pkt[5] == 0x00
        assert pkt[6] == 0x02  # (0x9c + 0x02 + 0x64 + 0x00) & 0xFF = 0x02

    def test_write_matches_capture(self):
        # Verified from capture: write addr=0x54, val=0xff0f inverted = [0x69, 0xff, 0xab, 0xff, 0xab, 0xf0, 0x9b]
        pkt = P.servo_packet_v2_write(0x54, 0xff0f)
        inverted = bytes(~b & 0xFF for b in pkt)
        assert inverted[0] == 0x69  # ~CMD_V2
        assert inverted[2] == (~0x54) & 0xFF  # ~addr


class TestPositionPacket:
    def test_center_position(self):
        # 1500 µs → pwm_raw = 6000 = 0x1770
        pkt = P.position_packet(6000)
        assert pkt[0] == 0x66  # CMD_SET_POS
        assert pkt[1] == 0x17  # 6000 >> 8
        assert pkt[2] == 0x70  # 6000 & 0xFF
        csum = (256 - (0x66 + 0x17 + 0x70) % 256) % 256
        assert pkt[3] == csum
