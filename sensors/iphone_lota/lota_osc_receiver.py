#!/usr/bin/env python3
"""Receive LOTA OSC camera tracking messages over UDP."""

from __future__ import annotations

import argparse
import json
import socket
import struct
import time
from datetime import datetime
from pathlib import Path
from typing import Any


def align4(offset: int) -> int:
    return (offset + 3) & ~3


def read_osc_string(data: bytes, offset: int) -> tuple[str, int]:
    end = data.find(b"\x00", offset)
    if end < 0:
        raise ValueError("OSC string is not null-terminated")
    return data[offset:end].decode("utf-8", errors="replace"), align4(end + 1)


def read_osc_atom(data: bytes, offset: int, tag: str) -> tuple[Any, int]:
    if tag == "i":
        return struct.unpack_from(">i", data, offset)[0], offset + 4
    if tag == "f":
        return struct.unpack_from(">f", data, offset)[0], offset + 4
    if tag == "s":
        return read_osc_string(data, offset)
    if tag == "T":
        return True, offset
    if tag == "F":
        return False, offset
    if tag == "N":
        return None, offset
    raise ValueError(f"Unsupported OSC type tag: {tag!r}")


def parse_osc_message(data: bytes) -> list[dict]:
    if data.startswith(b"#bundle\x00"):
        return parse_osc_bundle(data)

    address, offset = read_osc_string(data, 0)
    type_tags, offset = read_osc_string(data, offset)
    if not type_tags.startswith(","):
        raise ValueError(f"Bad OSC type tag string: {type_tags!r}")

    values = []
    for tag in type_tags[1:]:
        value, offset = read_osc_atom(data, offset, tag)
        values.append(value)
    return [{"address": address, "values": values}]


def parse_osc_bundle(data: bytes) -> list[dict]:
    if len(data) < 16:
        raise ValueError("OSC bundle too short")
    offset = 16
    messages = []
    while offset < len(data):
        size = struct.unpack_from(">i", data, offset)[0]
        offset += 4
        if size <= 0 or offset + size > len(data):
            raise ValueError("Bad OSC bundle element size")
        messages.extend(parse_osc_message(data[offset : offset + size]))
        offset += size
    return messages


def update_state(state: dict, msg: dict) -> None:
    address = msg["address"]
    values = msg["values"]
    if address == "/lota/camera/position":
        state["position"] = values
    elif address == "/lota/camera/rotation":
        state["rotation_quat"] = values
    elif address == "/lota/camera/euler":
        state["euler"] = values
    elif address == "/lota/fps":
        state["fps"] = values[0] if values else None
    elif address == "/lota/mode":
        state["mode"] = values[0] if values else None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--out", default="captures")
    parser.add_argument("--print-every", type=float, default=0.25)
    parser.add_argument("--max-packets", type=int, default=0, help="0 means run until Ctrl+C.")
    args = parser.parse_args()

    capture_dir = Path(args.out) / datetime.now().strftime("lota_osc_%Y%m%d_%H%M%S")
    capture_dir.mkdir(parents=True, exist_ok=True)
    log_path = capture_dir / "osc_messages.jsonl"

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))

    print(f"Listening for LOTA OSC UDP on {args.host}:{args.port}")
    print(f"Writing OSC log to {log_path.resolve()}")

    packet_count = 0
    last_print = 0.0
    state: dict[str, Any] = {}

    try:
        with log_path.open("a") as f:
            while args.max_packets <= 0 or packet_count < args.max_packets:
                data, addr = sock.recvfrom(65535)
                packet_count += 1
                now = time.time()
                try:
                    messages = parse_osc_message(data)
                except Exception as exc:
                    print(f"Bad OSC packet from {addr}: {exc}")
                    continue

                for msg in messages:
                    record = {"time": now, "remote": [addr[0], addr[1]], **msg}
                    f.write(json.dumps(record) + "\n")
                    update_state(state, msg)
                f.flush()

                if now - last_print >= args.print_every:
                    last_print = now
                    print(
                        "pose",
                        "pos=", state.get("position"),
                        "quat=", state.get("rotation_quat"),
                        "euler=", state.get("euler"),
                        "fps=", state.get("fps"),
                        "mode=", state.get("mode"),
                    )
    except KeyboardInterrupt:
        print("Stopping OSC receiver")
    finally:
        sock.close()


if __name__ == "__main__":
    main()

