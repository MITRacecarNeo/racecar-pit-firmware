# Test log

One entry per run. Record numbers, not adjectives.

---

## TL-003 - 2026-09-06

### Setup
Host only (no hardware attached). PlatformIO with the teensy platform, GCC 13 on
the host for the native environment. Python 3, numpy 2.4.0, pyserial 3.5,
pytest 9.0.2.

### Hypothesis
The cleanup refactor is behavior-preserving: the firmware still builds for
teensy41 and the extracted logic matches the original implementations.

### Method
`pio run`, `pio test -e native`, `python3 -m pytest tests -q` from
`racecar-pit-pi/`. Firmware flash and RAM compared against the pre-refactor
build.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| Build environments | teensy41, native | - |
| Native test cases | 10 | - |
| Pytest cases | 13 | - |

### Observations
- teensy41 build SUCCESS. Flash code 67028 -> 65748 B, data 9224 -> 8676 B,
  RAM1 variables 15136 -> 15104 B. RAM2 unchanged at 17568 B.
- native: 10/10 passed in 0.57 s.
- pytest: 13/13 passed in 0.06 s.
- `static_assert` confirms `sizeof(TXPacket) == 94` and
  `sizeof(RxPacket) == 470`, so the wire format did not move.

### Conclusion
Supported. No on-hardware run yet; TL-001 and TL-002 have not been repeated
against this build. Next action: re-run the bench check (TL-001) before the next
on-robot session.

---

## TL-002 - 2026-07-06

### Setup
Full ROS 2 stack on the Pi, car on stands (wheels up), Xbox controller through
the teleop mux. Pairs with `racecar_neo_ros2_driver` v0.5.0.

### Hypothesis
End-to-end control and telemetry work through the ROS driver, not only over a
direct serial link.

### Method
Bring the stack up, drive with the controller, watch `/imu/lsm9ds1` and the
driver's CRC and resync counters.

### Observations
- `/imu/lsm9ds1` at 151 Hz, accel z about -9.66 m/s^2.
- No CRC failures and no resync events over the session.
- Forward and reverse both actuate; steering follows the stick.
- Drive and steering polarity corrected Pi-side: `pit.yaml` `speed_sign` and
  `steering_sign` both -1.
- Dot-matrix glyph and LED strip respond to controller input.

### Conclusion
Supported.

---

## TL-001 - 2026-07 (bench)

### Setup
Wheels up, Teensy driven directly over the GPIO UART with no ROS stack.

### Hypothesis
Telemetry is framed correctly and every sensor reports real values.

### Observations
| Name | Value | Unit |
| ---- | ----- | ---- |
| Telemetry frames CRC-valid | 537/537 | frames |
| Bus voltage | 7.9 | V |
| Motor speed under Pi throttle | ~35000 | rpm |

- Accelerometer reads gravity; gyro and magnetometer return real values.
- Encoder and timestamp populate.
- Steering actuates.

### Conclusion
Supported.
