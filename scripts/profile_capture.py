"""Capture the profiling build's report from the Teensy USB serial port.

Bench tool, run on the Pi with the Teensy on USB and the `teensy41_profile`
firmware flashed. Reads one report line per stat per second, writes them to a
CSV under data/, and prints a summary against the timing targets. With
--probe it also runs scripts/telemetry_probe.py for the same time (ROS 2 and
the teleop service needed) and reports frames lost between the Teensy and the
Pi.

    python3 scripts/profile_capture.py 60 --tag baseline
    python3 scripts/profile_capture.py 60 --tag baseline --probe
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from collections.abc import Iterable
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
import statistics
import threading
import time
from typing import Any

DEFAULT_PORT = "/dev/ttyACM0"
DATA_DIR = Path(__file__).resolve().parents[1] / "data"
CSV_HEADER = "t_ms,kind,name,n,sum_us,max_us,p50_us,p99_us,value"


@dataclass(frozen=True)
class Target:
    """A limit on one summary value; `low` and `high` are inclusive."""

    kind: str
    name: str
    field: str
    low: float | None = None
    high: float | None = None
    label: str = ""


TARGETS = [
    Target("period", "tx_slot", "rate_hz", 199.0, 201.0, "telemetry rate"),
    Target("period", "tx_slot", "p99_us", None, 6000, "telemetry gap p99"),
    Target("period", "tx_slot", "max_us", None, 8000, "telemetry gap max"),
    Target("period", "drive_loop", "mean_us", None, 1050, "drive loop period mean"),
    Target("period", "drive_loop", "max_us", None, 1500, "drive loop period max"),
    Target("section", "cmd_to_pwm", "max_us", None, 1500, "command to PWM max"),
    Target("section", "irq_late", "max_us", None, 10, "interrupt lateness max"),
]


@dataclass
class Summary:
    windows: int = 0
    seconds: float = 0.0
    periods: dict[str, dict[str, float]] = field(default_factory=dict)
    sections: dict[str, dict[str, float]] = field(default_factory=dict)
    counters: dict[str, dict[str, float]] = field(default_factory=dict)


def parse_line(line: str) -> dict[str, Any] | None:
    """Parse one "P,..." report line; None for anything else."""
    parts = line.strip().split(",")
    if len(parts) < 5 or parts[0] != "P":
        return None
    try:
        t_ms, kind, name = int(parts[1]), parts[2], parts[3]
        nums = [int(p) for p in parts[4:]]
    except ValueError:
        return None
    keys: tuple[str, ...]
    if kind == "period" and len(nums) == 5:
        keys = ("n", "sum_us", "max_us", "p50_us", "p99_us")
    elif kind == "section" and len(nums) == 3:
        keys = ("n", "sum_us", "max_us")
    elif kind == "counter" and len(nums) == 1:
        keys = ("value",)
    else:
        return None
    return {"t_ms": t_ms, "kind": kind, "name": name, **dict(zip(keys, nums, strict=True))}


def covered_seconds(report_times_ms: list[int]) -> float:
    """Time the reports cover, in seconds.

    Each report closes a window that started at the previous report. Windows
    run slightly over 1 s (the reporter checks every 25 ms), so rates come from
    the report times rather than the window count. The first window's start is
    unknown; it is taken as the mean window length.
    """
    if not report_times_ms:
        return 0.0
    if len(report_times_ms) == 1:
        return 1.0
    span = report_times_ms[-1] - report_times_ms[0]
    return span * len(report_times_ms) / (len(report_times_ms) - 1) / 1000.0


def summarize(records: Iterable[dict[str, Any]]) -> Summary:
    """Combine per-second records into one summary per stat.

    p50 is the median of the per-second p50 values; p99 and max are the worst
    second, so a single bad second shows.
    """
    by_key: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    windows: set[int] = set()
    for r in records:
        by_key[(r["kind"], r["name"])].append(r)
        windows.add(r["t_ms"])
    out = Summary(windows=len(windows), seconds=covered_seconds(sorted(windows)))
    # Seconds per report window; a stat missing from some reports (lines
    # dropped while the port opened) is rated over the windows it has.
    per_window = (out.seconds / out.windows) if out.windows else 1.0
    for (kind, name), rows in sorted(by_key.items()):
        if kind == "counter":
            values = [r["value"] for r in rows]
            out.counters[name] = {
                "per_s": sum(values) / (len(rows) * per_window),
                "max": max(values),
            }
            continue
        n = sum(r["n"] for r in rows)
        stat = {
            "n": n,
            "rate_hz": n / (len(rows) * per_window),
            "mean_us": sum(r["sum_us"] for r in rows) / n if n else 0.0,
            "max_us": max(r["max_us"] for r in rows),
        }
        if kind == "period":
            stat["p50_us"] = statistics.median(r["p50_us"] for r in rows)
            stat["p99_us"] = max(r["p99_us"] for r in rows)
            out.periods[name] = stat
        else:
            out.sections[name] = stat
    return out


def check_targets(summary: Summary) -> list[tuple[Target, float | None, bool | None]]:
    """(target, value, passed) for each target; None when the stat is missing."""
    results: list[tuple[Target, float | None, bool | None]] = []
    for t in TARGETS:
        table = summary.periods if t.kind == "period" else summary.sections
        stat = table.get(t.name)
        if stat is None or t.field not in stat:
            results.append((t, None, None))
            continue
        value = stat[t.field]
        ok = (t.low is None or value >= t.low) and (t.high is None or value <= t.high)
        results.append((t, value, ok))
    return results


def format_summary(summary: Summary, pi_rate_hz: float | None = None) -> str:
    lines = [f"{summary.windows} reports over {summary.seconds:.1f} s"]
    lines.append(
        f"{'period':14s} {'rate Hz':>8s} {'mean us':>8s} {'p50':>6s} {'p99':>6s} {'max':>6s}"
    )
    for name, s in summary.periods.items():
        lines.append(
            f"{name:14s} {s['rate_hz']:8.1f} {s['mean_us']:8.0f} {s['p50_us']:6.0f} "
            f"{s['p99_us']:6.0f} {s['max_us']:6.0f}"
        )
    lines.append(f"{'section':14s} {'per s':>8s} {'mean us':>8s} {'max us':>8s}")
    for name, s in summary.sections.items():
        lines.append(f"{name:14s} {s['rate_hz']:8.1f} {s['mean_us']:8.1f} {s['max_us']:8.0f}")
    lines.append(f"{'counter':20s} {'per s':>8s} {'max':>6s}")
    for name, c in summary.counters.items():
        lines.append(f"{name:20s} {c['per_s']:8.1f} {c['max']:6.0f}")
    if pi_rate_hz is not None and "tx_frames" in summary.counters:
        sent = summary.counters["tx_frames"]["per_s"]
        lost = 100.0 * (1.0 - pi_rate_hz / sent) if sent else 0.0
        lines.append(f"Pi received {pi_rate_hz:.1f} Hz of {sent:.1f} Hz sent ({lost:.1f}% lost)")
    lines.append("targets:")
    for t, value, ok in check_targets(summary):
        verdict = "missing" if ok is None else ("pass" if ok else "FAIL")
        shown = "-" if value is None else f"{value:.1f}"
        lines.append(f"  {t.label:26s} {shown:>9s}  {verdict}")
    return "\n".join(lines)


def to_csv_row(r: dict[str, Any]) -> str:
    cols = ("n", "sum_us", "max_us", "p50_us", "p99_us", "value")
    return ",".join([str(r["t_ms"]), r["kind"], r["name"], *(str(r.get(c, "")) for c in cols)])


def capture(port: str, seconds: float) -> list[dict[str, Any]]:
    import serial

    records = []
    with serial.Serial(port, 115200, timeout=0.5) as ser:
        ser.reset_input_buffer()
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            raw = ser.readline()
            if not raw:
                continue
            record = parse_line(raw.decode("ascii", errors="replace"))
            if record is not None:
                records.append(record)
    return records


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("seconds", nargs="?", type=float, default=60.0)
    parser.add_argument("--tag", default="run", help="word for the CSV file name")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--probe", action="store_true", help="also count frames at the Pi")
    args = parser.parse_args()

    probe_result: dict[str, Any] = {}
    probe_thread = None
    if args.probe:
        import telemetry_probe

        probe_thread = threading.Thread(
            target=lambda: probe_result.update(telemetry_probe.probe(args.seconds)), daemon=True
        )
        probe_thread.start()

    records = capture(args.port, args.seconds)
    if probe_thread is not None:
        probe_thread.join(timeout=args.seconds + 10)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    DATA_DIR.mkdir(exist_ok=True)
    path = DATA_DIR / f"profile_{stamp}_{args.tag}.csv"
    path.write_text(CSV_HEADER + "\n" + "".join(to_csv_row(r) + "\n" for r in records))

    pi_rate = None
    if probe_result:
        others = {t: p for t, p in probe_result["publishers"].items() if p != ["pit_node"]}
        if others:
            print(f"probe INVALID: other publishers {others}")
        else:
            pi_rate = probe_result["rates"]["imu"]
    print(format_summary(summarize(records), pi_rate))
    print(f"Wrote {path}")


if __name__ == "__main__":
    main()
