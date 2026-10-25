import struct

import numpy as np
import pit_uart
import pytest

# Sizes from src/packets.h, which static_asserts them.
TXPACKET_BYTES = 94
RXPACKET_BYTES = 470

# Byte-for-byte reference shared with tests/test_info: payload "info=1\n",
# revision 3, chunk 0 of 1.
REFERENCE_INFO_PACKET = bytes.fromhex("dec0edfe010319000700000000000100696e666f3d310ad460")


def make_info_packet(payload: bytes, revision: int, index: int) -> bytes:
    """Python mirror of buildInfoPacket in src/packets.cpp."""
    size = pit_uart.INFO_CHUNK_BYTES
    count = max(1, -(-len(payload) // size))
    chunk = payload[index * size : (index + 1) * size]
    total = pit_uart.INFO_HEADER_BYTES + len(chunk) + 2
    head = struct.pack(
        "<IBBHIHH", pit_uart.INFO_MAGIC, 1, revision, total, len(payload), index, count
    )
    body = head + chunk
    return body + struct.pack("<H", pit_uart.crc16_ccitt(body))


def test_telemetry_schema_matches_firmware_txpacket():
    # The Pi consumes the 4 magic bytes while syncing, so its schema is 4 short.
    assert pit_uart.TELEMETRY_BYTES == TXPACKET_BYTES - 4


def test_command_schema_matches_firmware_rxpacket():
    # The command frame carries the magic word inline.
    assert pit_uart.COMMAND_BYTES == RXPACKET_BYTES + 4


def test_header_sizes():
    assert pit_uart.HEADER_SCHEMA.itemsize == 16
    assert pit_uart.HEADER_NO_MAGIC_SCHEMA.itemsize == 12
    assert pit_uart.INFO_HEADER_BYTES == 16


def test_crc_check_value():
    assert pit_uart.crc16_ccitt(b"123456789") == 0x29B1


def test_crc_is_chunkable():
    payload = bytes(range(32))
    chunked = pit_uart.crc16_ccitt(payload[:8])
    chunked = pit_uart.crc16_ccitt(payload[8:], chunked)
    assert chunked == pit_uart.crc16_ccitt(payload)


@pytest.mark.parametrize("index", [0, 7, 31])
def test_crc_detects_bit_flip(index):
    payload = bytearray(32)
    clean = pit_uart.crc16_ccitt(bytes(payload))
    payload[index] ^= 0x01
    assert pit_uart.crc16_ccitt(bytes(payload)) != clean


def test_build_command_sets_magic_and_size():
    pkt = pit_uart.build_command()
    assert pkt[0]["header"]["MagicNum"] == pit_uart.RX_MAGIC
    assert pkt[0]["header"]["PacketBytes"] == RXPACKET_BYTES


def test_build_command_checksum_is_self_consistent():
    pkt = pit_uart.build_command(servo=0.5, motor=-0.25, system_state=0x0C)
    raw = pkt.tobytes()
    assert pit_uart.crc16_ccitt(raw[:-2]) == pkt[0]["Checksum"]


def test_build_command_round_trips_setpoints():
    pkt = pit_uart.build_command(servo=-1.0, motor=0.75)
    assert pkt[0]["ServoAng"] == pytest.approx(-1.0)
    assert pkt[0]["MotorSpeed"] == pytest.approx(0.75)


def test_build_command_carries_the_info_request_bit():
    pkt = pit_uart.build_command(system_state=pit_uart.SYS_INFO_REQUEST)
    assert pkt[0]["SystemState"] & 0x20


def test_telemetry_crc_ok_accepts_a_well_formed_frame():
    pkt = np.zeros(1, dtype=pit_uart.TELEMETRY_SCHEMA)
    pkt[0]["Encoder"] = 1.5
    body = pkt.tobytes()[:-2]
    magic = pit_uart.TX_MAGIC.to_bytes(4, "little")
    checksum = pit_uart.crc16_ccitt(magic + body)
    frame = body + checksum.to_bytes(2, "little")
    assert pit_uart.telemetry_crc_ok(frame)


def test_telemetry_crc_ok_rejects_a_corrupt_frame():
    frame = bytearray(pit_uart.TELEMETRY_BYTES)
    assert not pit_uart.telemetry_crc_ok(bytes(frame))


# ------------------------------- info ------------------------------- #


def test_reference_packet_matches_the_python_mirror():
    assert make_info_packet(b"info=1\n", revision=3, index=0) == REFERENCE_INFO_PACKET


def test_decode_info_chunk_reads_the_reference_packet():
    chunk = pit_uart.decode_info_chunk(REFERENCE_INFO_PACKET)
    assert chunk == pit_uart.InfoChunk(
        revision=3, payload_bytes=7, index=0, count=1, data=b"info=1\n"
    )


@pytest.mark.parametrize("offset", [0, 5, 18, 24])
def test_decode_info_chunk_rejects_corruption(offset):
    packet = bytearray(REFERENCE_INFO_PACKET)
    packet[offset] ^= 0x10
    assert pit_uart.decode_info_chunk(bytes(packet)) is None


def test_decode_info_chunk_rejects_a_truncated_packet():
    assert pit_uart.decode_info_chunk(REFERENCE_INFO_PACKET[:-1]) is None


PAYLOAD = b"".join(f"key.n{i}={i}\n".encode() for i in range(60))


def chunks_of(payload: bytes, revision: int) -> list[pit_uart.InfoChunk]:
    count = max(1, -(-len(payload) // pit_uart.INFO_CHUNK_BYTES))
    out = []
    for i in range(count):
        chunk = pit_uart.decode_info_chunk(make_info_packet(payload, revision, i))
        assert chunk is not None
        out.append(chunk)
    return out


def test_assembler_completes_in_any_order():
    parts = chunks_of(PAYLOAD, revision=1)
    assert len(parts) > 2
    assembler = pit_uart.InfoAssembler()
    results = [assembler.add(c) for c in reversed(parts)]
    assert results[:-1] == [None] * (len(parts) - 1)
    assert results[-1] == PAYLOAD


def test_assembler_survives_a_lost_chunk_by_waiting_for_the_next_cycle():
    parts = chunks_of(PAYLOAD, revision=1)
    assembler = pit_uart.InfoAssembler()
    for c in parts[:1] + parts[2:]:
        assert assembler.add(c) is None
    assert assembler.add(parts[1]) == PAYLOAD


def test_assembler_never_mixes_revisions():
    old = chunks_of(PAYLOAD, revision=1)
    new_payload = PAYLOAD.replace(b"=5\n", b"=6\n")
    new = chunks_of(new_payload, revision=2)
    assembler = pit_uart.InfoAssembler()
    for c in old[:-1]:
        assembler.add(c)
    for c in new[:-1]:
        assert assembler.add(c) is None
    assert assembler.add(old[-1]) is None
    assert assembler.add(new[-1]) is None
    for c in new[:-1]:
        result = assembler.add(c)
    assert result == new_payload


def test_parse_info_types_values():
    info = pit_uart.parse_info(
        b"info=1\nfw.version=0.9.0\nfw.git=1234567\nspeed.min_mps=0.750\n"
        b"link.cmd_crc=1\nimu.gyro_unit=rad/s\nrc.offset=-3\n"
    )
    assert info == {
        "info": 1,
        "fw.version": "0.9.0",
        "fw.git": "1234567",
        "speed.min_mps": 0.75,
        "link.cmd_crc": 1,
        "imu.gyro_unit": "rad/s",
        "rc.offset": -3,
    }


def test_parse_info_rejects_a_line_without_a_value():
    with pytest.raises(ValueError):
        pit_uart.parse_info(b"info=1\nbroken\n")


def test_info_to_yaml_groups_keys():
    text = pit_uart.info_to_yaml(
        {"info": 1, "fw.version": "0.9.0", "fw.git": "e330426", "speed.min_mps": 0.75}
    )
    assert text == (
        "firmware:\n"
        "  info: 1\n"
        "  fw:\n"
        '    version: "0.9.0"\n'
        '    git: "e330426"\n'
        "  speed:\n"
        "    min_mps: 0.75\n"
    )


class FakeLink:
    """Byte source for iter_packets: returns the stream, then b"" forever."""

    def __init__(self, data: bytes) -> None:
        self.data = bytearray(data)

    def read(self, n: int) -> bytes:
        out = bytes(self.data[:n])
        del self.data[:n]
        return out


def take(gen, n):
    out = []
    for item in gen:
        if item[0] == 0:
            break
        out.append(item)
        if len(out) == n:
            break
    return out


def test_iter_packets_separates_telemetry_and_info():
    telemetry_body = bytes(pit_uart.TELEMETRY_BYTES)
    stream = (
        b"\x00\x11"
        + struct.pack("<I", pit_uart.TX_MAGIC)
        + telemetry_body
        + REFERENCE_INFO_PACKET
        + struct.pack("<I", pit_uart.TX_MAGIC)
        + telemetry_body
    )
    packets = take(pit_uart.iter_packets(FakeLink(stream).read), 3)
    assert [m for m, _ in packets] == [pit_uart.TX_MAGIC, pit_uart.INFO_MAGIC, pit_uart.TX_MAGIC]
    assert packets[1][1] == REFERENCE_INFO_PACKET
    assert packets[0][1] == telemetry_body


def test_iter_packets_skips_an_info_header_with_a_bad_length():
    bad = bytearray(REFERENCE_INFO_PACKET)
    struct.pack_into("<H", bad, 6, 4000)
    stream = bytes(bad) + REFERENCE_INFO_PACKET
    packets = take(pit_uart.iter_packets(FakeLink(stream).read), 1)
    assert packets == [(pit_uart.INFO_MAGIC, REFERENCE_INFO_PACKET)]
