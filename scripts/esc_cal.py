"""Step the ESC through its throttle calibration, run on the Pi.

Needs the calibration build on the Teensy (pio run -e teensy41_esc_cal -t upload)
and the Teensy USB cable plugged into the Pi. The script holds the ESC at
neutral, full forward and full reverse in turn while the operator presses SET,
and the LED strip shows the held step as a purple bar. The steps are the
firmware's PWM bounds (cfg::ESC_10BL60_G2::speed_pwm_min/neutral/max).

    python3 scripts/esc_cal.py
    python3 scripts/esc_cal.py --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
import re
import sys
import threading
import time

import serial

KEEPALIVE_SEC = 0.5
REPLY_TIMEOUT_SEC = 2.0
FINISH_WAIT_SEC = 3.0
BY_ID = Path("/dev/serial/by-id")

STATUS = re.compile(r"^esc_cal step=(\w+) pwm=(\d+) aborted=([01])$")


@dataclass(frozen=True)
class Status:
    step: str
    pwm: int
    aborted: bool


@dataclass(frozen=True)
class Step:
    command: str
    expect: str
    prompt: str


STEPS = (
    Step("n", "neutral", "Press SET once. The motor beeps once."),
    Step("f", "full_forward", "Press SET once. The motor beeps twice."),
    Step("r", "full_reverse", "Press SET once. The motor beeps three times."),
)

SETUP = (
    "Put the car on a stand so all four wheels are off the ground.",
    "Turn the transmitter off.",
    "Turn the ESC switch off.",
    "Press and hold the SET button on the ESC.",
    "Turn the ESC switch on while you hold SET.",
    "Release SET as soon as the ESC LED starts flashing red.",
)


def parse_status(line: str) -> Status | None:
    match = STATUS.match(line.strip())
    if not match:
        return None
    return Status(match.group(1), int(match.group(2)), match.group(3) == "1")


def find_teensy_port() -> str | None:
    """The Teensy's USB serial port by its stable name, or None."""
    if not BY_ID.is_dir():
        return None
    ports = sorted(p for p in BY_ID.iterdir() if p.name.startswith("usb-Teensyduino"))
    return str(ports[0]) if ports else None


class EscCalLink:
    """USB serial to the calibration build: commands out, status lines back.

    A background thread sends '?' every KEEPALIVE_SEC so a held step stays
    held; the firmware returns to neutral when the link goes quiet.
    """

    def __init__(self, ser: serial.Serial) -> None:
        self._ser = ser
        self._lock = threading.Lock()
        self._last: Status | None = None
        self._buf = b""
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._keepalive, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._ser.write(b"n")
        self._ser.flush()

    def _keepalive(self) -> None:
        while not self._stop.wait(KEEPALIVE_SEC):
            self.send("?")

    def send(self, command: str) -> None:
        with self._lock:
            self._ser.write(command.encode())
            self._ser.flush()
            self._read_lines()

    def _read_lines(self) -> None:
        self._buf += self._ser.read(self._ser.in_waiting or 0)
        while b"\n" in self._buf:
            raw, self._buf = self._buf.split(b"\n", 1)
            status = parse_status(raw.decode(errors="replace"))
            if status is not None:
                self._last = status

    def last(self) -> Status | None:
        with self._lock:
            self._read_lines()
            return self._last

    def wait_for(self, step: str, timeout: float = REPLY_TIMEOUT_SEC) -> Status | None:
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            status = self.last()
            if status is not None and status.step == step:
                return status
            time.sleep(0.05)
        return None


def ask(question: str, read: Callable[[str], str] = input) -> bool:
    return read(f"{question} [y/N] ").strip().lower() in ("y", "yes")


def run(link: EscCalLink, read: Callable[[str], str] = input) -> int:
    link.send("?")
    first = link.wait_for("neutral") or link.last()
    if first is None:
        print("No reply. Flash the calibration build: pio run -e teensy41_esc_cal -t upload")
        return 1
    if first.step != "neutral":
        link.send("n")

    for line in SETUP:
        print(line)
    if not ask("Is the ESC LED flashing red?", read):
        print("Stopped. The ESC is not in calibration mode; no throttle was sent.")
        return 1

    for step in STEPS:
        link.send(step.command)
        status = link.wait_for(step.expect)
        if status is None:
            print(f"The Teensy did not confirm {step.expect}. Stopped at neutral.")
            return 1
        print(f"Holding {status.pwm} us ({step.expect.replace('_', ' ')}).")
        read(f"{step.prompt} Then press Enter. ")
        status = link.last()
        if status is None or status.aborted or status.step != step.expect:
            print("The Teensy fell back to neutral. Start again from the first step.")
            return 1

    link.send("n")
    if link.wait_for("done") is None:
        print("The Teensy did not confirm the return to neutral.")
        return 1
    print("Holding neutral.")
    time.sleep(FINISH_WAIT_SEC)
    print("Calibration sent. The ESC is ready when it plays its startup tones.")
    print("Flash the normal firmware next: pio run -e teensy41 -t upload")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="Teensy USB serial port (default: by-id lookup)")
    args = parser.parse_args()
    port = args.port or find_teensy_port()
    if port is None:
        print("No Teensy USB serial port found. Plug the Teensy USB cable into the Pi.")
        return 1
    with serial.Serial(port, 115200, timeout=0.1) as ser:
        link = EscCalLink(ser)
        link.start()
        try:
            return run(link)
        except KeyboardInterrupt:
            print("\nStopped at neutral.")
            return 1
        finally:
            link.close()


if __name__ == "__main__":
    sys.exit(main())
