# Test log

One entry per run. Record numbers, not adjectives.

---

## TL-010 - 2026-10-25

### Setup
As TL-009, new firmware flashed (`e330426` source plus the uncommitted tree,
`fw.git_dirty=1`). The teleop service could not be stopped without a password
(polkit), so `pit_node` kept the UART open and reading.

### Hypothesis
The Teensy answers the info request with the compiled configuration, and the
reassembly survives lost chunks.

### Method
`python3 scripts/pit_uart.py --info --yaml firmware.yaml` on `/dev/neo-pit-pcb`
while `pit_node` ran. Diff the reply against a host build of `writeFirmwareInfo`
with the same config and commit flags. Probe telemetry for 5 s afterward.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| Reply | 1514 | B |
| Keys | 72 | - |
| Telemetry after | 151.8 | Hz |

### Observations
- The reply came back complete and matched the host build byte for byte:
  `speed.min_mps=0.500`, `speed.max_mps=1.000`, `status.imu=ok`,
  `status.power=ok`. The YAML wrote cleanly.
- With two readers on the port, each process saw only part of the byte
  stream, so chunks were lost; the assembler completed from later cycles.
- `pit_node` telemetry continued at 151.8 Hz.

### Conclusion
Supported. In normal use run the tool with teleop stopped; sharing the port
drops bytes for both readers.

---

## TL-009 - 2026-10-25

### Setup
Reference car, wheels on the ground and parked, teleop service running
(driver v0.9.0, `pit_node`). Teensy on the Pi's USB for flashing; dot matrix
disconnected by that wiring. Firmware: `main` (`e330426` source, indoor speed
band) before, `release/v0.9.0` staged tree after. FlySky transmitter off.
Battery 7.42 to 7.47 V.

### Hypothesis
The new firmware keeps every telemetry topic and value as before and raises
the telemetry rate toward 200 Hz.

### Method
Flash with `pio run -t upload` (`teensy-cli`) while teleop runs. Before and
after, a best-effort rclpy probe subscribes for 15 s to `/imu/lsm9ds1/raw`,
`/mag/raw`, `/encoder/speed`, `/battery/voltage`, `/rc/channels`, `/rc/link`,
`/battery/state` and records message rates and the gaps between Teensy-clock
stamps on `/imu/lsm9ds1/raw`. `journalctl -u racecar-teleop` for link errors.

### Parameters
| Name | Before | After | Unit |
| ---- | ------ | ----- | ---- |
| Telemetry topics rate | 141.6 | 151.4 | Hz |
| Teensy-stamped rate | 143.1 | 151.3 | Hz |
| Stamp gap median | 5.81 | 6.27 | ms |
| Stamp gap p95 | 10.83 | 10.09 | ms |
| Stamp gap p99 | 10.93 | 11.54 | ms |
| Stamp gap max | 11.04 | 15.18 | ms |
| `/battery/state` | 1.0 | 1.0 | Hz |

### Observations
- Every topic publishes after the flash. Accel (-0.69, -0.03, +9.92 m/s^2),
  gyro, mag (about -87, +169, -156 uT raw) and battery voltage match the
  before values within noise. RC link false and all channels -1.0 in both
  runs (transmitter off).
- `pit_node` logged one Teensy clock restart and one odometry gap at the
  flash; no CRC, serial, or reconnect messages.
- The first upload stalled: the HalfKay bootloader's hidraw device was
  root-only until `00-teensy.rules` was installed, and the write that started
  during `udevadm trigger` failed. The retry flashed in 12.9 s.

### Conclusion
Existing telemetry unchanged. Rate +7%, not 200 Hz: the median gap of
6.27 ms shows `UARTTx` waking about every 6.3 ms, so the running deadline
falls behind and resyncs. Not yet checked: the info reply, Pi and RC drive
paths, slow mode, manual override, LED strip.

---

## TL-008 - 2026-10-25

### Setup
Host only, on the car's Pi 5 (no flash). PlatformIO 6.2.0, platform teensy
6.0.0, GCC 13 native. Python 3.12.3, pytest 7.4.4.

### Hypothesis
The subsystem split and the info reply leave the firmware building and the
existing logic tests green, and the info payload fits its buffer and one
telemetry slot.

### Method
`pio run`, `pio test -e native`, `pytest`, `ruff`, `black --check`, `mypy`
from the repository root. Baseline: `pio run` and the two test suites on
`main` (`e330426`) with the libraries the caret ranges resolved on
2026-10-25.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| Info payload | 1514 | B |
| Info chunks | 12 | - |
| Info cycle | 60 | ms |
| Telemetry + largest chunk | 240 | B |
| Native test cases | 40 | - |
| Pytest cases | 29 | - |

### Observations
- teensy41 build SUCCESS. Flash code 86060 -> 90224 B, data 13108 -> 14132 B,
  RAM1 variables 18912 -> 21952 B (2048 B payload, 512 B transmit buffer).
  RAM2 unchanged at 12448 B.
- The baseline build of `main` is 86060 B of code, against 66388 B in TL-007
  for the same source: the unpinned libraries resolved to newer versions.
- Baseline tests: native 21/21, pytest 13/13.
- native: 40/40 passed. pytest: 29/29 passed. ruff, black, mypy clean.
- The commit string is in the binary (`strings firmware.elf`).

### Conclusion
Supported on the host. Not run on hardware. Next action: flash the car, run
`python3 scripts/pit_uart.py --info` with teleop stopped and compare every key
against `config/config.h`, then repeat TL-002 (drive both directions through
the ROS mux, wheels up) and measure the telemetry rate on
`/imu/lsm9ds1/raw`.

---

## TL-007 - 2026-09-06

### Setup
Host only. PlatformIO teensy platform, GCC 13 native.

### Hypothesis
Rescaling the stick onto the slow-mode cap makes full throttle an exact
`slow_speed_mps` setpoint, and closing the loop on RC in slow mode leaves the
fast-mode RC path unchanged.

### Method
`pio run`, `pio test -e native`, `python3 -m pytest tests -q`.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| slow_speed_mps | 1.0 | m/s |
| max_speed_mps | 5.0 | m/s |
| min_speed_mps | 0.75 | m/s |
| Native test cases | 21 | - |

### Observations
- teensy41 build SUCCESS. Flash code 66196 -> 66388 B (+192); RAM1 15104 B and
  RAM2 17568 B unchanged.
- native: 21/21 passed in 0.57 s. Four new cases on `throttleToTargetMps`.
- Full stick returns the cap exactly in both modes: 5.0 and 1.0 m/s, and their
  negatives.
- Half stick in slow mode lands strictly between 0.75 and 1.0 m/s, so the map
  rescales rather than clipping; the clamp it replaced pinned everything above
  roughly 0.06 stick at the cap.

### Conclusion
Supported on the host. Not measured on hardware: whether the closed loop
actually settles at 1.0 m/s under RC, which is the point of the change. Next
action: drive in slow mode with `cfg::DEBUG::serial_plot` on and compare
Target_MPS against Measured_MPS.

---

## TL-006 - 2026-09-06 (bench)

### Setup
Same car and transmitter as TL-005, with `ch_manual_override` moved to index 7.

### Hypothesis
CH8 reaches the Teensy over PPM, so swD drives the manual override.

### Method
Flip swD and watch the dot matrix and the drive path; exercise every v0.7.0
feature.

### Observations
- CH8 arrives and swD drives the override. The six-channel figure for the iA6B
  counts its servo rails; the PPM stream carries the whole frame, so all eight
  slots populate. The TL-005 concern does not apply.
- All v0.7.0 features working: charge bar, slow mode on swA, manual override on
  swD, dot-matrix I/M under override.

### Conclusion
Supported. The prediction in TL-005 that CH7 and CH8 might be absent was wrong,
and the "six channels over PPM" claim is corrected across `config.h`, the
README, `docs/architecture.md`, and `docs/changelog.md`.

Not separately measured: `slow_rc_scale` against an instrumented 1.0 m/s. Slow
mode engages and caps the car, but the RC-path scalar is still the provisional
0.4 rather than a calibrated value.

---

## TL-005 - 2026-09-06 (bench)

### Setup
Car on the bench with the FS-i6 transmitter and iA6B receiver live, firmware at
the v0.7.0 branch head. Reported from the robot, not measured here.

### Hypothesis
The v0.7.0 switch scheme and the charge bar behave on hardware as they do on the
host, and swA/swD sit on CH5/CH8 as the FS-i6 aux order implies.

### Method
Flip each switch and watch the dot matrix, the LED strip, and the drive
response.

### Observations
- swA (CH5, index 4) selects slow mode; the speed cap engages.
- LED strip renders the charge bar.
- The dot matrix switches between I and M on flipping **swB**, not swD. So the
  override was bound to index 5, and index 5 carries swB.
- CH5 = swA and CH6 = swB, matching the stock FS-i6 aux assignment.

### Conclusion
The override mechanism works; only the channel binding was wrong.
`ch_manual_override` moves from index 5 (CH6, swB) to index 7 (CH8, swD).

---

## TL-004 - 2026-09-06

### Setup
Host only (no hardware attached). PlatformIO teensy platform, GCC 13 for the
native environment. Python 3, pytest 9.0.2.

### Hypothesis
The state-of-charge bar, the RC remap, and the switch scheme leave the firmware
building and the logic tests green; the SoC curve is monotonic across
7.0-8.4 V, and an absent switch channel resolves to its permissive position.

### Method
`pio run`, `pio test -e native`, `python3 -m pytest tests -q` from
`racecar-pit-pi/`. Firmware size compared against the TL-003 build.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| SoC curve span | 7000 to 8400 | mV |
| Curve breakpoints | 10 | - |
| Native test cases | 17 | - |
| Pytest cases | 13 | - |

### Observations
- teensy41 build SUCCESS. Flash code 65748 -> 66196 B (+448), data 8676 B
  unchanged, RAM1 variables 15104 B and RAM2 17568 B both unchanged.
- Switch decode: pulses of 0 and 2500 us both read down, so a lost receiver
  leaves fast mode and autonomy enabled.
- native: 17/17 passed in 0.51 s.
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
steers left. Reverse the affected channel on the transmitter if not. Then
confirm swA and swD sit on CH5 and CH6 and that each switch's down position
reads below 1500 us, and calibrate `slow_rc_scale` against a measured 1.0 m/s.

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
