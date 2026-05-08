"""Tests for D-series direct protocol (no hardware required)."""

import pytest
from hitecdpc.dseries import (
    BAUD,
    BAUDRATE_CODES,
    BAUDRATE_TO_CODE,
    HDR_QUERY,
    HDR_REPLY,
    OP_READ,
    OP_WRITE,
    build_read_query,
    build_write_cmd,
    decode_model_name,
    get_reg,
    parse_read_response,
)


# ── build_read_query ──────────────────────────────────────────────────────────


def test_read_query_header_and_op():
    q = build_read_query(servo_id=0, addr=0x0C)
    assert q[0] == HDR_QUERY
    assert q[3] == OP_READ


def test_read_query_servo_id_and_addr():
    q = build_read_query(servo_id=3, addr=0x32)
    assert q[1] == 3
    assert q[2] == 0x32


def test_read_query_checksum():
    q = build_read_query(servo_id=0, addr=0x0C)
    assert q[4] == (0x00 + 0x0C) & 0xFF


def test_read_query_length():
    assert len(build_read_query(0, 0)) == 5


def test_read_query_matches_read_position_py():
    # Matches _build_read_query(0x0C) from servo-test/examples/read_position.py
    assert build_read_query(0, 0x0C) == bytes([0x96, 0x00, 0x0C, 0x00, 0x0C])


# ── build_write_cmd ───────────────────────────────────────────────────────────


def test_write_cmd_header_and_op():
    c = build_write_cmd(servo_id=0, addr=0x32, value=1)
    assert c[0] == HDR_QUERY
    assert c[3] == OP_WRITE


def test_write_cmd_length():
    assert len(build_write_cmd(0, 0, 0)) == 7


def test_write_cmd_little_endian():
    # byte[4] = LSB, byte[5] = MSB (same order as read responses)
    c = build_write_cmd(servo_id=0, addr=0x0A, value=0x0102)
    assert c[4] == 0x02, "byte[4] should be LSB"
    assert c[5] == 0x01, "byte[5] should be MSB"


def test_write_cmd_small_value():
    # value=2 (e.g. deadband): lsb=2, msb=0
    c = build_write_cmd(servo_id=0, addr=78, value=2)
    assert c[4] == 2
    assert c[5] == 0


def test_write_cmd_large_value():
    # value=0x0BB8 (3000): lsb=0xB8, msb=0x0B (verified against DPC source)
    c = build_write_cmd(servo_id=0, addr=30, value=0x0BB8)
    assert c[4] == 0xB8
    assert c[5] == 0x0B


def test_write_cmd_checksum():
    c = build_write_cmd(servo_id=0, addr=0x0A, value=0x0102)
    # cs = (servo_id + addr + OP_WRITE + lsb + msb) & 0xFF
    expected = (0 + 0x0A + 0x02 + 0x02 + 0x01) & 0xFF
    assert c[6] == expected


def test_write_cmd_servo_id_in_checksum():
    c0 = build_write_cmd(servo_id=0, addr=0x0A, value=100)
    c1 = build_write_cmd(servo_id=1, addr=0x0A, value=100)
    assert c0[6] != c1[6]


# ── parse_read_response ───────────────────────────────────────────────────────


def _make_response(addr, value, servo_id=0, mystery=0):
    """Build a valid synthetic read response."""
    low = value & 0xFF
    high = (value >> 8) & 0xFF
    cs = (mystery + addr + OP_WRITE + low + high) & 0xFF
    return bytes([HDR_REPLY, mystery, addr, OP_WRITE, low, high, cs])


def test_parse_valid_response_zero():
    resp = _make_response(0x0C, 0)
    value, err = parse_read_response(0x0C, resp)
    assert err is None
    assert value == 0


def test_parse_valid_response_nonzero():
    resp = _make_response(0x0C, 2048)
    value, err = parse_read_response(0x0C, resp)
    assert err is None
    assert value == 2048


def test_parse_little_endian_reconstruction():
    resp = _make_response(0x0C, 0x0102)
    value, err = parse_read_response(0x0C, resp)
    assert err is None
    assert value == 0x0102


def test_parse_bad_header():
    resp = bytes([0x00, 0, 0x0C, OP_WRITE, 0, 0, 0])
    _, err = parse_read_response(0x0C, resp)
    assert err is not None
    assert "header" in err


def test_parse_short_response():
    _, err = parse_read_response(0x0C, b"\x69\x00\x0c")
    assert err is not None
    assert "short" in err


def test_parse_addr_mismatch():
    resp = _make_response(0x0C, 100)
    _, err = parse_read_response(0x0E, resp)  # wrong addr expected
    assert err is not None
    assert "mismatch" in err.lower()


def test_parse_bad_checksum():
    resp = bytearray(_make_response(0x0C, 100))
    resp[-1] ^= 0xFF  # corrupt checksum
    _, err = parse_read_response(0x0C, bytes(resp))
    assert err is not None
    assert "checksum" in err


# ── register map ─────────────────────────────────────────────────────────────


def test_get_reg_position():
    assert get_reg("position") == 12


def test_get_reg_servo_id():
    assert get_reg("servo_id") == 50


def test_get_reg_unknown():
    with pytest.raises(KeyError, match="Unknown D-series register"):
        get_reg("nonexistent_xyz")


def test_baud_constant():
    assert BAUD == 115200


def test_all_reg_addrs_even():
    from hitecdpc.dseries import REGS

    for name, addr in REGS.items():
        assert addr % 2 == 0, (
            f"{name}: address {addr} is not even (all D-series regs are 16-bit)"
        )


def test_all_reg_names_unique():
    from hitecdpc.dseries import REGS

    assert len(REGS) == len(set(REGS.keys()))


def test_all_reg_addrs_unique():
    from hitecdpc.dseries import REGS

    addrs = list(REGS.values())
    assert len(addrs) == len(set(addrs)), "duplicate register addresses"


# ── decode_model_name ─────────────────────────────────────────────────────────


def test_model_name_d646wp():
    # product_no=34646 (0x8756), product_version=4097 (0x1001)
    # num5=3, num6=4646, num7=4 → "D" + "646" + "_"
    assert decode_model_name(34646, 4097) == "D646_"


def test_model_name_variant_flag():
    # product_version high nibble nonzero → text3="_"
    # product_no=10646 → num5=1 → "MD646_"
    assert decode_model_name(10646, 0x1000) == "MD646_"


def test_model_name_b_series():
    # product_version & 0xF000 == 0x2000 and & 0xF0 == 0x30 → flag → "B" appended
    assert decode_model_name(10646, 0x2030) == "MDB646_"


def test_model_name_unknown():
    assert decode_model_name(99999, 0) == "Unknown(99999)"


# ── baudrate constants ────────────────────────────────────────────────────────


def test_baudrate_default_code():
    assert BAUDRATE_CODES[5] == 115_200


def test_baudrate_roundtrip():
    for code, baud in BAUDRATE_CODES.items():
        assert BAUDRATE_TO_CODE[baud] == code
