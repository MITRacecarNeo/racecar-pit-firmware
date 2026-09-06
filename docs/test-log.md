# Test log

One entry per run. Record numbers, not adjectives.

---

## TL-004 - 2026-09-06

### Setup
Host only (no hardware attached). PlatformIO teensy platform, GCC 13 for the
native environment. Python 3, pytest 9.0.2.

### Hypothesis
The state-of-charge bar and the comment pass leave the firmware building and the
logic tests green, and the SoC curve is monotonic across 7.0-8.4 V.

### Method
`pio run`, `pio test -e native`, `python3 -m pytest tests -q` from
`racecar-pit-pi/`. Firmware size compared against the TL-003 build.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| SoC curve span | 7000 to 8400 | mV |
| Curve breakpoints | 10 | - |
| Native test cases | 14 | - |
| Pytest cases | 13 | - |

### Observations
- teensy41 build SUCCESS. Flash code 65748 -> 65876 B (+128), data 8676 B
  unchanged, RAM1 variables 15104 B and RAM2 17568 B both unchanged.
- native: 14/14 passed in 0.62 s.
- pytest: 13/13 passed in 0.05 s.
- SoC is monotonic over the whole band at 10 mV steps and clamps to 0 and 100
  outside it.
- Curve vs a linear voltage map: 7600 mV reads 40% against 43%, 7400 mV reads
  13% against 29%. The gap is the point of the change.

### Conclusion
Supported on the host. Not yet run on hardware: the bar has not been watched
against a discharging pack, and the FS-iA6B channel map in
`docs/architecture.md` is derived from the code and the receiver spec, not from
a bench capture. Next action, before any wheels-down run: log CH1-CH8 over
telemetry with the transmitter on, confirm CH2 and CH4 rest near 1500 us, and
check that pushing the right stick forward drives forward and left stick left
steers left. Reverse the affected channel on the transmitter if not.

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
