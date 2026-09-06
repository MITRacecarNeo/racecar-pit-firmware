import numpy as np
import pytest

import PiUART

# Sizes from racecar-pit-teensy/include/packets.h, which static_asserts them.
TXPACKET_BYTES = 94
RXPACKET_BYTES = 470


def test_telemetry_schema_matches_firmware_txpacket():
    # The Pi consumes the 4 magic bytes while syncing, so its schema is 4 short.
    assert PiUART.TELEMETRY_BYTES == TXPACKET_BYTES - 4


def test_command_schema_matches_firmware_rxpacket():
    # The command frame carries the magic word inline.
    assert PiUART.COMMAND_BYTES == RXPACKET_BYTES + 4


def test_header_sizes():
    assert PiUART.HEADER_SCHEMA.itemsize == 16
    assert PiUART.HEADER_NO_MAGIC_SCHEMA.itemsize == 12


def test_crc_check_value():
    assert PiUART.crc16_ccitt(b"123456789") == 0x29B1


def test_crc_is_chunkable():
    payload = bytes(range(32))
    chunked = PiUART.crc16_ccitt(payload[:8])
    chunked = PiUART.crc16_ccitt(payload[8:], chunked)
    assert chunked == PiUART.crc16_ccitt(payload)


@pytest.mark.parametrize("index", [0, 7, 31])
def test_crc_detects_bit_flip(index):
    payload = bytearray(32)
    clean = PiUART.crc16_ccitt(bytes(payload))
    payload[index] ^= 0x01
    assert PiUART.crc16_ccitt(bytes(payload)) != clean


def test_build_command_sets_magic_and_size():
    pkt = PiUART.build_command()
    assert pkt[0]["header"]["MagicNum"] == PiUART.RX_MAGIC
    assert pkt[0]["header"]["PacketBytes"] == RXPACKET_BYTES


def test_build_command_checksum_is_self_consistent():
    pkt = PiUART.build_command(servo=0.5, motor=-0.25, system_state=0x0C)
    raw = pkt.tobytes()
    assert PiUART.crc16_ccitt(raw[:-2]) == pkt[0]["Checksum"]


def test_build_command_round_trips_setpoints():
    pkt = PiUART.build_command(servo=-1.0, motor=0.75)
    assert pkt[0]["ServoAng"] == pytest.approx(-1.0)
    assert pkt[0]["MotorSpeed"] == pytest.approx(0.75)


def test_telemetry_crc_ok_accepts_a_well_formed_frame():
    pkt = np.zeros(1, dtype=PiUART.TELEMETRY_SCHEMA)
    pkt[0]["Encoder"] = 1.5
    body = pkt.tobytes()[:-2]
    magic = PiUART.TX_MAGIC.to_bytes(4, "little")
    checksum = PiUART.crc16_ccitt(magic + body)
    frame = body + checksum.to_bytes(2, "little")
    assert PiUART.telemetry_crc_ok(frame)


def test_telemetry_crc_ok_rejects_a_corrupt_frame():
    frame = bytearray(PiUART.TELEMETRY_BYTES)
    assert not PiUART.telemetry_crc_ok(bytes(frame))
