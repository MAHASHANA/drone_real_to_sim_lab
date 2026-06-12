#!/usr/bin/env python3
"""Parser tests for lota_osc_receiver.py."""

from __future__ import annotations

import struct

from lota_osc_receiver import parse_osc_message


def pad4(data: bytes) -> bytes:
    return data + b"\x00" * ((4 - len(data) % 4) % 4)


def osc_string(value: str) -> bytes:
    return pad4(value.encode("utf-8") + b"\x00")


def test_float_message() -> None:
    data = osc_string("/lota/camera/position")
    data += osc_string(",fff")
    data += struct.pack(">fff", 1.0, 2.0, 3.0)
    messages = parse_osc_message(data)
    assert messages == [{"address": "/lota/camera/position", "values": [1.0, 2.0, 3.0]}]


def test_string_message() -> None:
    data = osc_string("/lota/mode")
    data += osc_string(",s")
    data += osc_string("Depth")
    messages = parse_osc_message(data)
    assert messages == [{"address": "/lota/mode", "values": ["Depth"]}]


if __name__ == "__main__":
    test_float_message()
    test_string_message()
    print("LOTA OSC parser tests passed")

