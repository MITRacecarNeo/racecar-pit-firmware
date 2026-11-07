"""Read, change and save the car's speed constants over the Teensy's USB port.

The firmware keeps one set of per-car speed-loop constants in its EEPROM and
falls back to the built-in fleet values when none is saved. `set` changes the
running values at once without saving; `save` writes them to the EEPROM.
Changes are refused while the car is moving.

    python3 scripts/car_constants.py get
    python3 scripts/car_constants.py set pid.ff_rev_us=110 pid.kv_ff_rev=28
    python3 scripts/car_constants.py save            # dated today
    python3 scripts/car_constants.py export car.json # values and source as JSON
    python3 scripts/car_constants.py import car.json # set every value, then save
    python3 scripts/car_constants.py revert | defaults | erase
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
import datetime
import json
from pathlib import Path
import sys
import time
from typing import Protocol

from esc_cal import find_teensy_port

REPLY_TIMEOUT_SEC = 3.0


class Port(Protocol):
    def write(self, data: bytes) -> int | None: ...
    def readline(self) -> bytes: ...


class ConstantsError(RuntimeError):
    """The firmware answered with an error line."""


def command(
    port: Port,
    line: str,
    timeout: float = REPLY_TIMEOUT_SEC,
    clock: Callable[[], float] = time.monotonic,
) -> list[str]:
    """Sends one `cal ...` line; returns the reply lines up to and including `ok ...`."""
    port.write((line + "\n").encode())
    lines: list[str] = []
    end = clock() + timeout
    while clock() < end:
        raw = port.readline()
        if not raw:
            continue
        text = raw.decode(errors="replace").strip()
        if not text:
            continue
        if text.startswith("err "):
            raise ConstantsError(text[4:])
        lines.append(text)
        if text.startswith("ok "):
            return lines
    raise TimeoutError(f"no reply to {line!r}")


def parse_get(lines: Sequence[str]) -> dict[str, str]:
    values = {}
    for line in lines:
        if line.startswith("ok "):
            break
        key, sep, value = line.partition("=")
        if sep:
            values[key] = value
    return values


def constant_keys(values: dict[str, str]) -> dict[str, float]:
    """The settable pid.* values of a `get` reply."""
    return {k: float(v) for k, v in values.items() if k.startswith("pid.")}


def set_line(values: dict[str, float]) -> str:
    return "cal set " + " ".join(f"{k}={v:g}" for k, v in sorted(values.items()))


def today_ymd(today: datetime.date | None = None) -> str:
    return (today or datetime.date.today()).strftime("%Y%m%d")


def run(port: Port, args: argparse.Namespace, out=sys.stdout) -> int:
    verb = args.verb
    if verb == "get":
        for line in command(port, "cal get")[:-1]:
            print(line, file=out)
    elif verb == "set":
        if not args.pairs:
            raise SystemExit("set needs KEY=VALUE pairs")
        command(port, "cal set " + " ".join(args.pairs))
        print("set (not saved; run save to keep it)", file=out)
    elif verb == "save":
        reply = command(port, f"cal save {args.date or today_ymd()}")
        print(" ".join(reply), file=out)
    elif verb in ("revert", "defaults", "erase"):
        print(command(port, f"cal {verb}")[-1], file=out)
    elif verb == "export":
        values = parse_get(command(port, "cal get"))
        doc = {
            "source": values.get("cal.source"),
            "eeprom": values.get("cal.eeprom"),
            "crc": values.get("cal.crc"),
            "written": values.get("cal.written"),
            "constants": constant_keys(values),
        }
        Path(args.file).write_text(json.dumps(doc, indent=2) + "\n")
        print(f"wrote {args.file}", file=out)
    elif verb == "import":
        doc = json.loads(Path(args.file).read_text())
        constants = {k: float(v) for k, v in doc["constants"].items()}
        command(port, set_line(constants))
        reply = command(port, f"cal save {args.date or today_ymd()}")
        print(" ".join(reply), file=out)
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "verb", choices=["get", "set", "save", "revert", "defaults", "erase", "export", "import"]
    )
    ap.add_argument(
        "pairs", nargs="*", help="KEY=VALUE for set; the JSON file for export and import"
    )
    ap.add_argument("--port", help="USB serial port (default: the Teensyduino by-id link)")
    ap.add_argument("--date", help="YYYYMMDD stored with save and import (default: today)")
    args = ap.parse_args(argv)
    if args.verb in ("export", "import"):
        if len(args.pairs) != 1:
            ap.error(f"{args.verb} needs one file name")
        args.file = args.pairs[0]

    import serial

    port_name = args.port or find_teensy_port()
    if port_name is None:
        ap.error("no Teensy USB port found; pass --port")
    with serial.Serial(port_name, 115200, timeout=0.2) as port:
        port.reset_input_buffer()
        try:
            return run(port, args)
        except ConstantsError as e:
            print(f"firmware refused: {e}", file=sys.stderr)
            return 1


if __name__ == "__main__":
    raise SystemExit(main())
