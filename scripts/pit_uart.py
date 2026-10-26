"""Bench tool for the NEO-PIT UART link, run on the Pi.

Mirrors src/packets.h and docs/protocol.md. No ROS; the production Pi
side lives in racecar_neo_ros2_driver (pit_node, pit_protocol.py). Use this to
bring up or debug the link on its own, with the teleop service stopped.

    python3 scripts/pit_uart.py                 # stream speed and RC channels
    python3 scripts/pit_uart.py --info          # print the firmware info reply
    python3 scripts/pit_uart.py --info --yaml firmware.yaml
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from pathlib import Path
import re
import struct
import time

import numpy as np
import serial

DEFAULT_PORT = "/dev/neo-pit-pcb"
BAUD_RATE = 921600
TX_MAGIC = 0xDEADBEEF  # Teensy -> Pi, telemetry
RX_MAGIC = 0xBEEFDEAD  # Pi -> Teensy, command
INFO_MAGIC = 0xFEEDC0DE  # Teensy -> Pi, info reply chunk
PROTOCOL_VERSION = 1
COMMAND_HZ = 60
NUM_LEDS = 84
RC_CHANNELS = 8

# RxPacket.SystemState bits (cfg::SYS_STATE).
SYS_INFO_REQUEST = 0x20

INFO_CHUNK_BYTES = 128
INFO_TIMEOUT_SEC = 2.0
# Info values kept as text even when they look numeric.
INFO_STRING_KEYS = frozenset({"fw.version", "fw.label", "fw.git"})

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

# Info chunk header, magic included: magic, version, revision, packet bytes,
# payload bytes, chunk index, chunk count.
_INFO_HEADER = struct.Struct("<IBBHIHH")
INFO_HEADER_BYTES = _INFO_HEADER.size
INFO_PACKET_MAX_BYTES = INFO_HEADER_BYTES + INFO_CHUNK_BYTES + 2


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
    PWM bounds in config/config.h.
    """
    pkt = np.zeros(1, dtype=COMMAND_SCHEMA)
    pkt[0]["header"]["MagicNum"] = RX_MAGIC
    pkt[0]["header"]["VersionNum"] = PROTOCOL_VERSION
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


@dataclass(frozen=True)
class InfoChunk:
    """One decoded info packet."""

    revision: int
    payload_bytes: int
    index: int
    count: int
    data: bytes


def decode_info_chunk(packet: bytes) -> InfoChunk | None:
    """Decode one info packet, magic included; None when it is malformed."""
    if len(packet) < INFO_HEADER_BYTES + 2:
        return None
    magic, version, revision, packet_bytes, payload_bytes, index, count = _INFO_HEADER.unpack_from(
        packet
    )
    if magic != INFO_MAGIC or version != PROTOCOL_VERSION or packet_bytes != len(packet):
        return None
    if count == 0 or index >= count:
        return None
    (checksum,) = struct.unpack_from("<H", packet, len(packet) - 2)
    if crc16_ccitt(packet[:-2]) != checksum:
        return None
    return InfoChunk(revision, payload_bytes, index, count, packet[INFO_HEADER_BYTES:-2])


@dataclass
class InfoAssembler:
    """Collects info chunks until one revision of the payload is complete.

    Chunks of a newer revision, or a different chunk count or payload size,
    restart the collection, so a payload never mixes two revisions.
    """

    _key: tuple[int, int, int] | None = None
    _chunks: dict[int, bytes] = field(default_factory=dict)

    def add(self, chunk: InfoChunk) -> bytes | None:
        """Add a chunk; return the payload once every chunk of it is in."""
        key = (chunk.revision, chunk.count, chunk.payload_bytes)
        if key != self._key:
            self._key = key
            self._chunks = {}
        self._chunks[chunk.index] = chunk.data
        if len(self._chunks) < chunk.count:
            return None
        payload = b"".join(self._chunks[i] for i in range(chunk.count))
        if len(payload) != chunk.payload_bytes:
            self._key, self._chunks = None, {}
            return None
        return payload


_INT = re.compile(r"-?\d+")
_FLOAT = re.compile(r"-?\d+\.\d+")


def parse_info(payload: bytes) -> dict[str, int | float | str]:
    """Parse "key=value" lines into typed values (int, float, or str)."""
    info: dict[str, int | float | str] = {}
    for line in payload.decode("ascii").splitlines():
        key, sep, value = line.partition("=")
        if not sep:
            raise ValueError(f"info line without '=': {line!r}")
        if key in INFO_STRING_KEYS:
            info[key] = value
        elif _INT.fullmatch(value):
            info[key] = int(value)
        elif _FLOAT.fullmatch(value):
            info[key] = float(value)
        else:
            info[key] = value
    return info


def info_to_yaml(info: dict[str, int | float | str], root: str = "firmware") -> str:
    """Render parsed info as YAML, one mapping per key group (fw, speed, ...)."""
    groups: dict[str, list[tuple[str, int | float | str]]] = {}
    for key, value in info.items():
        group, _, name = key.partition(".")
        groups.setdefault(group, []).append((name, value))

    lines = [f"{root}:"]
    for group, items in groups.items():
        if len(items) == 1 and items[0][0] == "":
            lines.append(f"  {group}: {_yaml_scalar(items[0][1])}")
            continue
        lines.append(f"  {group}:")
        lines.extend(f"    {name}: {_yaml_scalar(value)}" for name, value in items)
    return "\n".join(lines) + "\n"


def _yaml_scalar(value: int | float | str) -> str:
    if isinstance(value, str):
        return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'
    return repr(value)


def iter_packets(read: Callable[[int], bytes]) -> Iterator[tuple[int, bytes]]:
    """Yield (magic, packet) for each telemetry body or info packet on the link.

    Telemetry is yielded without its magic (TELEMETRY_SCHEMA); info packets
    whole, magic included. `read(n)` returns up to n bytes, or b"" on timeout.
    """
    window = bytearray()
    while True:
        byte = read(1)
        if not byte:
            yield 0, b""
            continue
        window += byte
        del window[:-4]
        if len(window) < 4:
            continue
        magic = struct.unpack("<I", window)[0]
        if magic == TX_MAGIC:
            yield TX_MAGIC, read(TELEMETRY_BYTES)
            window.clear()
        elif magic == INFO_MAGIC:
            head = read(INFO_HEADER_BYTES - 4)
            window.clear()
            if len(head) != INFO_HEADER_BYTES - 4:
                continue
            packet_bytes = struct.unpack_from("<H", head, 2)[0]
            if not INFO_HEADER_BYTES + 2 <= packet_bytes <= INFO_PACKET_MAX_BYTES:
                continue
            rest = read(packet_bytes - INFO_HEADER_BYTES)
            yield INFO_MAGIC, struct.pack("<I", INFO_MAGIC) + head + rest


def request_info(ser: serial.Serial, timeout: float = INFO_TIMEOUT_SEC) -> bytes | None:
    """Hold the info request bit until a full payload arrives; None on timeout.

    Firmware without the info reply never answers, so the caller falls back
    to configured values.
    """
    command = build_command(system_state=SYS_INFO_REQUEST).tobytes()
    assembler = InfoAssembler()
    interval = 1.0 / COMMAND_HZ
    deadline = time.monotonic() + timeout
    last_tx = 0.0
    try:
        for magic, packet in iter_packets(ser.read):
            now = time.monotonic()
            if now >= deadline:
                return None
            if now - last_tx >= interval:
                ser.write(command)
                last_tx = now
            if magic != INFO_MAGIC:
                continue
            chunk = decode_info_chunk(packet)
            if chunk is None:
                continue
            payload = assembler.add(chunk)
            if payload is not None:
                return payload
    finally:
        ser.write(build_command().tobytes())
    return None


def stream(ser: serial.Serial, verify_crc: bool) -> None:
    command = build_command().tobytes()
    interval = 1.0 / COMMAND_HZ
    last_tx = 0.0
    for magic, frame in iter_packets(ser.read):
        if magic == TX_MAGIC and len(frame) == TELEMETRY_BYTES:
            if not verify_crc or telemetry_crc_ok(frame):
                telemetry = np.frombuffer(frame, dtype=TELEMETRY_SCHEMA)[0]
                print(f"speed {telemetry['Encoder']:+.2f} m/s  rc {telemetry['RxReceiver']}")
        now = time.monotonic()
        if now - last_tx >= interval:
            ser.write(command)
            last_tx = now


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--no-verify-crc", action="store_true")
    parser.add_argument("--info", action="store_true", help="print the firmware info reply")
    parser.add_argument("--yaml", type=Path, help="with --info: also write it as YAML")
    args = parser.parse_args()

    ser = serial.Serial(args.port, BAUD_RATE, timeout=0.05)
    if not args.info:
        stream(ser, verify_crc=not args.no_verify_crc)
        return

    payload = request_info(ser)
    if payload is None:
        raise SystemExit("No info reply. The firmware may predate it.")
    print(payload.decode("ascii"), end="")
    if args.yaml:
        args.yaml.write_text(info_to_yaml(parse_info(payload)))
        print(f"Wrote {args.yaml}")


if __name__ == "__main__":
    main()
