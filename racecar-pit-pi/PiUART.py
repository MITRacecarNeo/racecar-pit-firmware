"""Standalone Pi-side reference for the NEO-PIT UART link.

Mirrors racecar-pit-teensy/include/packets.h. No ROS; the production Pi side
lives in racecar_neo_ros2_driver (pit_node, pit_protocol.py). Use this to bring
up or debug the wire link on its own.
"""

from __future__ import annotations

import argparse
import collections
import struct
import time

import numpy as np
import serial

DEFAULT_PORT = "/dev/ttyAMA0"
BAUD_RATE = 921600
TX_MAGIC = 0xDEADBEEF  # Teensy -> Pi, telemetry
RX_MAGIC = 0xBEEFDEAD  # Pi -> Teensy, command
COMMAND_HZ = 60
NUM_LEDS = 84
RC_CHANNELS = 8

# '<' little endian, matching the packed structs on the Teensy.
_T = {
    "uint8": "<u1",
    "int8": "<i1",
    "uint16": "<u2",
    "int16": "<i2",
    "uint32": "<u4",
    "int32": "<i4",
    "float": "<f4",
    "bool": "<?",
}

HEADER_SCHEMA = np.dtype(
    [
        ("MagicNum", _T["uint32"]),
        ("VersionNum", _T["uint8"]),
        ("IDNumber", _T["uint8"]),
        ("PacketBytes", _T["uint16"]),
        ("FrameBytes", _T["uint32"]),
        ("PacketIndex", _T["uint16"]),
        ("PacketsPerFrame", _T["uint16"]),
    ]
)

HEADER_NO_MAGIC_SCHEMA = np.dtype(
    [
        ("VersionNum", _T["uint8"]),
        ("IDNumber", _T["uint8"]),
        ("PacketBytes", _T["uint16"]),
        ("FrameBytes", _T["uint32"]),
        ("PacketIndex", _T["uint16"]),
        ("PacketsPerFrame", _T["uint16"]),
    ]
)

# Teensy -> Pi. The 4 magic bytes are consumed by the sync window, so this
# schema starts at the byte after them: TXPacket minus MagicNum.
TELEMETRY_SCHEMA = np.dtype(
    [
        ("header", HEADER_NO_MAGIC_SCHEMA),
        ("Timestamp", _T["int32"]),
        ("VoltAndCurr", _T["uint16"], 2),
        ("RxReceiver", _T["uint16"], RC_CHANNELS),
        ("IMU", _T["float"], 9),
        ("Encoder", _T["float"]),
        ("EKF", _T["float"], 3),
        ("Checksum", _T["uint16"]),
    ]
)

# Pi -> Teensy, whole frame including the magic word.
COMMAND_SCHEMA = np.dtype(
    [
        ("header", HEADER_SCHEMA),
        ("ServoAng", _T["float"]),
        ("MotorSpeed", _T["float"]),
        ("DotMatrix", _T["bool"], 192),
        ("LED", _T["uint8"], 255),
        ("SystemState", _T["uint8"]),
        ("Checksum", _T["uint16"]),
    ]
)

TELEMETRY_BYTES = TELEMETRY_SCHEMA.itemsize
COMMAND_BYTES = COMMAND_SCHEMA.itemsize


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE. Check value over b"123456789" is 0x29B1."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_command(
    servo: float = 0.0,
    motor: float = 0.0,
    system_state: int = 0,
) -> np.ndarray:
    """Command frame with the header filled and the checksum applied.

    servo and motor are normalized to [-1, 1]; the Teensy maps them onto the
    PWM bounds in config.h.
    """
    pkt = np.zeros(1, dtype=COMMAND_SCHEMA)
    pkt[0]["header"]["MagicNum"] = RX_MAGIC
    pkt[0]["header"]["VersionNum"] = 1
    pkt[0]["header"]["PacketBytes"] = COMMAND_BYTES - 4  # struct size on the Teensy
    pkt[0]["header"]["PacketsPerFrame"] = 1
    pkt[0]["ServoAng"] = servo
    pkt[0]["MotorSpeed"] = motor
    pkt[0]["SystemState"] = system_state
    pkt[0]["Checksum"] = crc16_ccitt(pkt.tobytes()[:-2])
    return pkt


def telemetry_crc_ok(frame: bytes) -> bool:
    """Check a telemetry body against its trailing checksum.

    The Teensy checksums the magic word too, so it is prepended back on here.
    """
    expected = struct.unpack("<H", frame[-2:])[0]
    return crc16_ccitt(struct.pack("<I", TX_MAGIC) + frame[:-2]) == expected


def run(port: str, verify_crc: bool) -> None:
    ser = serial.Serial(port, BAUD_RATE, timeout=1)
    window: collections.deque[int] = collections.deque(maxlen=4)
    command = build_command()
    interval = 1.0 / COMMAND_HZ
    last_tx = 0.0

    while True:
        byte = ser.read(1)
        if byte:
            window.append(byte[0])
            if len(window) == 4 and struct.unpack("<I", bytes(window))[0] == TX_MAGIC:
                frame = ser.read(TELEMETRY_BYTES)
                if len(frame) == TELEMETRY_BYTES and (not verify_crc or telemetry_crc_ok(frame)):
                    telemetry = np.frombuffer(frame, dtype=TELEMETRY_SCHEMA)[0]
                    print(f"speed {telemetry['Encoder']:+.2f} m/s  rc {telemetry['RxReceiver']}")
                window.clear()

        now = time.time()
        if now - last_tx >= interval:
            ser.write(command.tobytes())
            last_tx = now
        time.sleep(0.001)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--no-verify-crc", action="store_true")
    args = parser.parse_args()
    run(args.port, verify_crc=not args.no_verify_crc)


if __name__ == "__main__":
    main()
