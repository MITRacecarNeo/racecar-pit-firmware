"""Measure the firmware's telemetry as pit_node republishes it, on the Pi.

Bench tool; needs ROS 2 and the teleop service running. Subscribes best-effort
to the telemetry topics for a few seconds and prints message rates, the gaps
between Teensy-clock stamps on /imu/lsm9ds1/raw, distinct IMU and battery
samples per second, and the last sensor values. A run where any topic has a
publisher besides pit_node is marked invalid.

    python3 scripts/telemetry_probe.py            # 10 s
    python3 scripts/telemetry_probe.py 30 --runs 3
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import dataclass
import statistics
import time
from typing import Any

IMU_TOPIC = "/imu/lsm9ds1/raw"
TOPICS = {
    "imu": IMU_TOPIC,
    "mag": "/mag/raw",
    "encoder": "/encoder/speed",
    "volt": "/battery/voltage",
    "rc": "/rc/channels",
    "rc_link": "/rc/link",
}


@dataclass(frozen=True)
class GapStats:
    """Gaps between consecutive stamps, in milliseconds."""

    count: int
    median: float
    p95: float
    p99: float
    max: float
    rate_hz: float


def gap_stats(stamps: Sequence[float]) -> GapStats | None:
    """Gap statistics for stamps in seconds; None with fewer than three."""
    if len(stamps) < 3:
        return None
    gaps = sorted((b - a) * 1e3 for a, b in zip(stamps, stamps[1:], strict=False))

    def quantile(q: float) -> float:
        return gaps[min(len(gaps) - 1, int(q * len(gaps)))]

    span = stamps[-1] - stamps[0]
    return GapStats(
        count=len(gaps),
        median=statistics.median(gaps),
        p95=quantile(0.95),
        p99=quantile(0.99),
        max=gaps[-1],
        rate_hz=len(gaps) / span if span > 0 else 0.0,
    )


def count_changes(values: Sequence[Any]) -> int:
    """Number of values that differ from the one before (the first counts)."""
    changes = 0
    previous: Any = object()
    for value in values:
        if value != previous:
            changes += 1
            previous = value
    return changes


def probe(seconds: float) -> dict[str, Any]:
    import rclpy
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import Imu, MagneticField
    from std_msgs.msg import Bool, Float32, Float32MultiArray

    types = {
        "imu": Imu,
        "mag": MagneticField,
        "encoder": Float32,
        "volt": Float32,
        "rc": Float32MultiArray,
        "rc_link": Bool,
    }
    rclpy.init()
    node = rclpy.create_node("telemetry_probe")
    qos = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    counts = dict.fromkeys(TOPICS, 0)
    last: dict[str, Any] = {}
    stamps: list[float] = []
    imu_keys: list[tuple[float, float, float]] = []
    volts: list[float] = []

    def callback(name: str):
        def on_msg(msg: Any) -> None:
            counts[name] += 1
            last[name] = msg
            if name == "imu":
                stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
                a, g = msg.linear_acceleration, msg.angular_velocity
                imu_keys.append((a.x, a.y, g.z))
            elif name == "volt":
                volts.append(msg.data)

        return on_msg

    for name, topic in TOPICS.items():
        node.create_subscription(types[name], topic, callback(name), qos)

    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    publishers = {
        topic: [info.node_name for info in node.get_publishers_info_by_topic(topic)]
        for topic in TOPICS.values()
    }
    node.destroy_node()
    rclpy.try_shutdown()
    return {
        "seconds": seconds,
        "rates": {name: n / seconds for name, n in counts.items()},
        "gaps": gap_stats(stamps),
        "distinct_imu": count_changes(imu_keys) / seconds,
        "distinct_volt": count_changes(volts) / seconds,
        "publishers": publishers,
        "last": last,
    }


def report(result: dict[str, Any]) -> str:
    lines = []
    others = {t: p for t, p in result["publishers"].items() if p != ["pit_node"]}
    lines.append("INVALID: other publishers " + repr(others) if others else "publishers ok")
    lines.append(
        "rates Hz: " + "  ".join(f"{name} {rate:.1f}" for name, rate in result["rates"].items())
    )
    gaps = result["gaps"]
    if gaps:
        lines.append(
            f"stamp gaps ms: median {gaps.median:.2f}  p95 {gaps.p95:.2f}  "
            f"p99 {gaps.p99:.2f}  max {gaps.max:.2f}  (n={gaps.count}, {gaps.rate_hz:.1f} Hz)"
        )
    lines.append(
        f"distinct per s: imu {result['distinct_imu']:.1f}  volt {result['distinct_volt']:.1f}"
    )
    last = result["last"]
    if "imu" in last:
        a, g = last["imu"].linear_acceleration, last["imu"].angular_velocity
        lines.append(
            f"accel {a.x:+.2f} {a.y:+.2f} {a.z:+.2f}  gyro {g.x:+.4f} {g.y:+.4f} {g.z:+.4f}"
        )
    if "mag" in last:
        m = last["mag"].magnetic_field
        lines.append(f"mag uT {m.x * 1e6:+.1f} {m.y * 1e6:+.1f} {m.z * 1e6:+.1f}")
    if "volt" in last:
        lines.append(f"battery {last['volt'].data:.3f} V")
    if "rc_link" in last:
        lines.append(f"rc link {last['rc_link'].data}")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("seconds", nargs="?", type=float, default=10.0)
    parser.add_argument("--runs", type=int, default=1)
    args = parser.parse_args()
    for run in range(args.runs):
        if args.runs > 1:
            print(f"--- run {run + 1} of {args.runs}")
        print(report(probe(args.seconds)))


if __name__ == "__main__":
    main()
