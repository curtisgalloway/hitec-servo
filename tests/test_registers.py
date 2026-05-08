"""Tests for the register map."""

import pytest
from hitecdpc import registers as R


def test_all_registers_have_valid_sram_addr():
    for name, reg in R.all_registers().items():
        assert 0 <= reg.sram_addr <= 127, f"{name}: sram_addr out of range"


def test_ee_addrs_in_upper_bank():
    for name, reg in R.all_registers().items():
        if reg.ee_addr is not None:
            assert 128 <= reg.ee_addr <= 255, (
                f"{name}: ee_addr {reg.ee_addr} not in EEPROM bank"
            )


def test_16bit_registers_have_even_sram_addr():
    for name, reg in R.all_registers().items():
        if reg.is_16bit:
            # Pairs must not overlap other entries
            assert reg.sram_addr + 1 <= 127, (
                f"{name}: 16-bit register overflows SRAM bank"
            )


def test_no_sram_addr_collisions():
    seen: dict[int, str] = {}
    for name, reg in R.all_registers().items():
        addrs = [reg.sram_addr, reg.sram_addr + 1] if reg.is_16bit else [reg.sram_addr]
        for addr in addrs:
            assert addr not in seen, (
                f"SRAM addr {addr} used by both {seen[addr]} and {name}"
            )
            seen[addr] = name


def test_lookup_by_name():
    reg = R.get("deadband")
    assert reg.sram_addr == 2


def test_lookup_alias():
    reg = R.get("Dead Band")
    assert reg.sram_addr == 2


def test_lookup_unknown_raises():
    with pytest.raises(KeyError, match="Unknown register"):
        R.get("nonexistent_register_xyz")


def test_scale_and_unit_on_position_registers():
    for name in ("smin", "smax", "sref", "fpos", "ioffset"):
        reg = R.get(name)
        assert reg.scale == 0.25
        assert reg.unit == "µs"
        assert reg.is_16bit
