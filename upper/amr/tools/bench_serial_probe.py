#!/usr/bin/env python3
"""Bench serial probe for the Mick binary chassis protocol.

Runs on a PC (or the A7A) with the USB/UART link to the ESP32, so the MCU
firmware can be validated before wiring the board into ROS 2.

Frames mirror amr_dispatcher_core/chassis/chassis_packet.cpp exactly:

    0xAE 0xEA | len | type | payload... | sum8 | 0xEF 0xFE

    len   = payload size + 2 (type byte included in the checksum span)
    sum8  = uint8 wrapping sum over [len, type, payload...]

Downlink 0xF3: three int16 big-endian, x1000  (mm/s, mm/s, mrad/s) + seq uint8
Uplink   0xA1: x, y, yaw, vx, wz (int16 BE) + battery (uint16 BE, x100)
               optional:   flags uint8 (bit0 fault, bit1 uart_timeout,
                           bit2 estop_input, bit3 over_current)
               optional:   last_cmd_seq uint8
Uplink   0xA2: status byte + battery (uint16 BE, x100)

Backward compatibility (both directions, mirrors chassis_packet.cpp):
  * the probe accepts 12-byte 0xA1 frames (no flags / seq) -- older firmware
  * the probe accepts 6-byte 0xF3 frames -- older host builds
  * --no-seq keeps emitting 6-byte 0xF3 for firmware that has not added the field

Examples:
    python3 tools/bench_serial_probe.py --port /dev/ttyUSB0 --baud 460800 --mode listen
    python3 tools/bench_serial_probe.py --port /dev/ttyUSB0 --mode drive --speed 0.15 --seconds 3
    python3 tools/bench_serial_probe.py --port /dev/ttyUSB0 --mode corrupt --seconds 3
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass
from typing import Iterable

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - dependency hint only
    sys.exit("pyserial is required: pip install pyserial")


MAGIC = b"\xae\xea"
TAIL = b"\xef\xfe"

TYPE_SPEED_COMMAND = 0xF3
TYPE_CLEAR_ODOMETRY = 0xE1
TYPE_ODOMETRY = 0xA1
TYPE_STATE = 0xA2

MAX_FRAME_BYTES = 512


def _sum8(data: bytes) -> int:
    return sum(data) & 0xFF


def encode_frame(frame_type: int, payload: bytes) -> bytes:
    body = bytes([len(payload) + 2, frame_type]) + payload
    return MAGIC + body + bytes([_sum8(body)]) + TAIL


def _i16(value: float) -> bytes:
    clamped = max(-32768.0, min(32767.0, value))
    return struct.pack(">h", int(round(clamped)))


def encode_speed_command(
    linear_x: float, linear_y: float, angular_z: float, seq: int = 0, include_seq: bool = True
) -> bytes:
    payload = _i16(linear_x * 1000.0) + _i16(linear_y * 1000.0) + _i16(angular_z * 1000.0)
    if include_seq:
        payload += bytes([seq & 0xFF])
    return encode_frame(TYPE_SPEED_COMMAND, payload)


def encode_clear_odometry() -> bytes:
    return encode_frame(TYPE_CLEAR_ODOMETRY, b"\x01\x00\x00\x00")


@dataclass
class Frame:
    frame_type: int
    payload: bytes


@dataclass
class Odometry:
    x_m: float
    y_m: float
    yaw_rad: float
    vx_mps: float
    wz_radps: float
    battery_v: float
    # Optional extension bytes (12-byte frames decode to 0/0).
    flags: int = 0
    last_cmd_seq: int = 0

    def fault_bits(self) -> list[str]:
        names = []
        if self.flags & 0x01:
            names.append("fault")
        if self.flags & 0x02:
            names.append("uart_timeout")
        if self.flags & 0x04:
            names.append("estop_input")
        if self.flags & 0x08:
            names.append("over_current")
        return names


@dataclass
class State:
    status: str
    battery_v: float | None


class FrameParser:
    """Byte-stream parser tolerant to partial frames, noise and corrupt checksums."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.bad_checksum = 0
        self.bad_tail = 0
        self.bad_length = 0

    def feed(self, chunk: bytes) -> list[Frame]:
        self.buffer.extend(chunk)
        if len(self.buffer) > MAX_FRAME_BYTES:
            del self.buffer[:-MAX_FRAME_BYTES]
        frames: list[Frame] = []
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                # keep a trailing 0xAE that may begin the next frame
                del self.buffer[: max(0, len(self.buffer) - 1)]
                return frames
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 7:
                return frames
            length = self.buffer[2]
            if length < 2:
                self.bad_length += 1
                del self.buffer[:1]
                continue
            total = 2 + 1 + length + 2
            if len(self.buffer) < total:
                return frames
            if self.buffer[total - 2 : total] != TAIL:
                self.bad_tail += 1
                del self.buffer[:1]
                continue
            body = self.buffer[2 : total - 3]
            if _sum8(body) != self.buffer[total - 3]:
                self.bad_checksum += 1
                del self.buffer[:total]
                continue
            frames.append(Frame(self.buffer[3], bytes(self.buffer[4 : total - 3])))
            del self.buffer[:total]


def decode_odometry(frame: Frame) -> Odometry | None:
    if len(frame.payload) < 12:
        return None
    x, y, yaw, vx, wz, _battery_i16 = struct.unpack(">hhhhhh", frame.payload[:12])
    # battery is unsigned: re-read bytes 10..12 as uint16 (mirrors ReadUint16).
    raw_battery = struct.unpack(">H", frame.payload[10:12])[0]
    flags = frame.payload[12] if len(frame.payload) >= 13 else 0
    last_cmd_seq = frame.payload[13] if len(frame.payload) >= 14 else 0
    return Odometry(
        x / 1000.0,
        y / 1000.0,
        yaw / 1000.0,
        vx / 1000.0,
        wz / 1000.0,
        raw_battery / 100.0,
        flags,
        last_cmd_seq,
    )


def decode_state(frame: Frame) -> State:
    status = "running" if not frame.payload or frame.payload[0] == 0 else "fault"
    battery = None
    if len(frame.payload) >= 3:
        battery = struct.unpack(">H", frame.payload[1:3])[0] / 100.0
    return State(status, battery)


def _corrupt(frame: bytes) -> bytes:
    """Flip one payload byte so the receiver must reject it on checksum."""
    if len(frame) < 8:
        return frame
    mutated = bytearray(frame)
    mutated[4] ^= 0xFF
    return bytes(mutated)


def run_listen(port: serial.Serial, seconds: float, show_corrupt: bool) -> int:
    parser = FrameParser()
    deadline = time.monotonic() + seconds
    seen = {"odom": 0, "state": 0, "other": 0}
    last_print = 0.0
    while time.monotonic() < deadline:
        chunk = port.read(max(1, port.in_waiting or 1))
        if not chunk:
            continue
        for frame in parser.feed(chunk):
            if frame.frame_type == TYPE_ODOMETRY:
                odom = decode_odometry(frame)
                if odom is None:
                    seen["other"] += 1
                    continue
                seen["odom"] += 1
                if show_corrupt or time.monotonic() - last_print > 0.2:
                    faults = odom.fault_bits()
                    extra = f" flags=0x{odom.flags:02X}({','.join(faults)})" if odom.flags else ""
                    extra += f" echo_seq={odom.last_cmd_seq}" if len(frame.payload) >= 14 else ""
                    print(
                        f"ODOM x={odom.x_m:+.3f} y={odom.y_m:+.3f} yaw={odom.yaw_rad:+.3f} "
                        f"vx={odom.vx_mps:+.3f} wz={odom.wz_radps:+.3f} batt={odom.battery_v:.2f}V"
                        f"{extra}"
                    )
                    last_print = time.monotonic()
            elif frame.frame_type == TYPE_STATE:
                seen["state"] += 1
                state = decode_state(frame)
                print(f"STATE {state.status} batt={state.battery_v if state.battery_v is not None else 'n/a'}")
            else:
                seen["other"] += 1
                print(f"TYPE 0x{frame.frame_type:02X} payload={frame.payload.hex()}")
    print(
        f"summary odom={seen['odom']} state={seen['state']} other={seen['other']} "
        f"bad_checksum={parser.bad_checksum} bad_tail={parser.bad_tail} bad_length={parser.bad_length}"
    )
    return 0 if seen["odom"] or seen["state"] else 1


def run_drive(port: serial.Serial, speed: float, yaw: float, seconds: float, rate_hz: float, include_seq: bool) -> int:
    period = 1.0 / rate_hz
    deadline = time.monotonic() + seconds
    sent = 0
    seq = 0
    try:
        while time.monotonic() < deadline:
            port.write(encode_speed_command(speed, 0.0, yaw, seq, include_seq))
            seq = (seq + 1) & 0xFF
            sent += 1
            time.sleep(max(0.0, period - 0.0005))
    finally:
        # Always leave the chassis at rest, even on Ctrl-C or an exception.
        for _ in range(5):
            port.write(encode_speed_command(0.0, 0.0, 0.0, seq, include_seq))
            seq = (seq + 1) & 0xFF
            time.sleep(0.02)
    layout = "7B with seq" if include_seq else "6B legacy (no seq)"
    print(f"sent {sent} speed frames ({layout}, linear_x={speed:+.3f} m/s, angular_z={yaw:+.3f} rad/s)")
    return 0


def run_corrupt(port: serial.Serial, seconds: float, rate_hz: float) -> int:
    """Alternate good and checksum-broken frames to prove the receiver drops only bad ones."""
    period = 1.0 / rate_hz
    deadline = time.monotonic() + seconds
    good = 0
    bad = 0
    while time.monotonic() < deadline:
        frame = encode_speed_command(0.0, 0.0, 0.0)
        port.write(frame)
        good += 1
        port.write(_corrupt(frame))
        bad += 1
        time.sleep(period)
    print(f"sent good={good} corrupt={bad}; receiver must keep working and count checksum errors")
    return 0


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Mick binary chassis protocol bench probe")
    parser.add_argument("--port", required=True, help="serial device, e.g. /dev/ttyUSB0 or COM5")
    parser.add_argument("--baud", type=int, default=460800, help="must match the ESP32 firmware")
    parser.add_argument("--mode", choices=("listen", "drive", "corrupt", "clear-odom"), default="listen")
    parser.add_argument("--speed", type=float, default=0.1, help="linear.x in m/s (drive mode)")
    parser.add_argument("--yaw", type=float, default=0.0, help="angular.z in rad/s (drive mode)")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=20.0, help="command rate in Hz")
    parser.add_argument("--raw", action="store_true", help="print every odometry frame")
    parser.add_argument(
        "--no-seq",
        action="store_true",
        help="emit 6-byte 0xF3 frames (legacy firmware without the seq field)",
    )
    return parser.parse_args(list(argv))


def main(argv: Iterable[str]) -> int:
    args = parse_args(argv)
    with serial.Serial(args.port, args.baud, timeout=0.05) as port:
        if args.mode == "listen":
            return run_listen(port, args.seconds, args.raw)
        if args.mode == "drive":
            return run_drive(
                port, args.speed, args.yaw, args.seconds, args.rate, not args.no_seq
            )
        if args.mode == "corrupt":
            return run_corrupt(port, args.seconds, args.rate)
        port.write(encode_clear_odometry())
        print("sent clear-odometry frame (0xE1)")
        return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
