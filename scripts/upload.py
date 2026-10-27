"""Flash a Teensy 4.1 over USB, retrying a failed write.

After a soft reboot from the running firmware, the first write to the HalfKay
bootloader fails ("error writing to Teensy"); a second write to the same
bootloader succeeds. PlatformIO runs this as the upload command
(platformio.ini), so `pio run -t upload` completes in one go.

    python3 scripts/upload.py --tools ~/.platformio/packages/tool-teensy firmware.hex
"""

from __future__ import annotations

import argparse
from collections.abc import Callable, Sequence
from pathlib import Path
import subprocess
import sys

MCU = "TEENSY41"
ATTEMPTS = 3
WRITE_TIMEOUT_SEC = 60.0


Runner = Callable[[Sequence[str], float], tuple[int, str]]


def run(cmd: Sequence[str], timeout: float) -> tuple[int, str]:
    try:
        out = subprocess.run(list(cmd), capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return 1, f"timed out after {timeout:g} s"
    return out.returncode, out.stdout + out.stderr


def write_ok(code: int, output: str) -> bool:
    return code == 0 and "error" not in output.lower()


def upload(tools: Path, hex_path: Path, attempts: int = ATTEMPTS, runner: Runner = run) -> int:
    """Soft reboot, then write until one attempt succeeds.

    Returns the attempt that succeeded, or 0 when every attempt failed.
    """
    # Fails harmlessly when the board is already in the bootloader.
    runner([str(tools / "teensy_reboot"), "-s"], 10.0)
    loader = [str(tools / "teensy_loader_cli"), f"-mmcu={MCU}", "-w", "-v", str(hex_path)]
    for attempt in range(1, attempts + 1):
        code, output = runner(loader, WRITE_TIMEOUT_SEC)
        if write_ok(code, output):
            return attempt
        print(f"Write attempt {attempt} failed: {output.strip().splitlines()[-1:]}")
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        "--tools", type=Path, required=True, help="PlatformIO tool-teensy directory"
    )
    parser.add_argument("--attempts", type=int, default=ATTEMPTS)
    parser.add_argument("hex", type=Path)
    args = parser.parse_args()
    attempts = upload(args.tools, args.hex, args.attempts)
    if attempts == 0:
        sys.exit(f"Upload failed after {args.attempts} attempts.")
    print(f"Uploaded {args.hex.name} on attempt {attempts}.")


if __name__ == "__main__":
    main()
