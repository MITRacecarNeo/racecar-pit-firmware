# Test log

One entry per run. Record numbers, not adjectives.

---

## TL-017 - 2026-10-26

### Setup
Reference car on a block, all four wheels free, teleop running, gamepad and
FS-i6 transmitter with iA6B receiver. Firmware A: `c1e3091` (drive loop in a
thread, before sensor pacing). Firmware B: `c1e3091` plus the 1 kHz drive
interrupt, paced sensors, and swA decoded as slow on the low pulse.

### Hypothesis
The drive interrupt leaves the drive path and the switch behavior unchanged;
the wheels-free speed overshoot seen on 2026-10-26 predates it.

### Method
A recorder logged `/motor`, `/encoder/speed`, `/rc/channels`, `/rc/link`,
`/joy` and `/battery/current` while the operator drove a scripted sequence:
gamepad forward and reverse, steering, release of the enable button, RC
override, both switches in both positions. The switch polarity was read by
holding both levers up, then down, twice. The same full-stick gamepad and
slow-mode RC holds ran on firmware A and B. Logs (not in git): `data/raw/drive_regression_20261026.csv`,
`data/raw/drive_ab_old_20261026.csv`, `data/raw/drive_ab_new_20261026.csv`.

### Parameters
| Name | A | B | Unit |
| ---- | - | - | ---- |
| Gamepad full stick, peak speed | 1.89, 2.01, 2.07 | 1.84, 1.96, 1.94 | m/s |
| Gamepad full stick, last 1.5 s of a 3 to 4 s hold | 1.35, 1.22, 1.24 | 1.20, 1.35, 1.21 | m/s |
| Slow-mode RC full stick, peak / last 1.5 s | 1.90 / 1.26 | 1.95 / 1.19 | m/s |
| Gamepad commanded with swD high: wheel speed | - | 0.00 (134 samples) | m/s |

### Observations
- Lever up (away from the operator) sends the low pulse on both switches,
  lever down the high pulse; the FS-i6 starts only with every lever up and
  will not power off while the receiver is on, so a no-signal case was not
  reached.
- On firmware A, swA up (low) gave fast mode, the opposite of the intended
  default; swD behaved as intended (high, lever down, locks the Pi out).
  Firmware B decodes swA low as slow; swD unchanged.
- Gamepad forward and reverse, steering both ways, stop on releasing the
  enable button, RC taking over from the gamepad, swD lockout and release,
  slow-mode RC, and fast-mode RC all behaved as expected on B (operator
  check and recorded speeds).
- With the wheels free the closed loop peaks about twice the 1.0 m/s
  setpoint and is still 0.2 to 0.35 m/s high after 3 to 4 s, on both A and
  B. Fast-mode RC is open-loop PWM: a quarter stick reached 8.6 m/s.

### Conclusion
Supported. The drive interrupt keeps the drive and switch behavior; swA now
starts in slow mode with the transmitter's default lever position. The
wheels-free overshoot is the existing speed loop with no load; check on the
floor with the speed-loop work.

---

## TL-016 - 2026-10-25

### Setup
Reference car on a block, teleop running, 6 min after a reboot, fresh battery
(8.20 V), transmitter off. Profiling build of `c1e3091` plus the drive loop
in a 1 kHz IntervalTimer interrupt (priority 144, `Seqlock` for RC channels
and the drive command), LSM9DS1 read every 4 ms, INA226 every 10 ms.

### Hypothesis
With the drive loop on a timer and the sensors paced, every timing target
holds.

### Method
`scripts/profile_capture.py 60 --probe`, then
`scripts/telemetry_probe.py 10 --runs 3` and the info reply. Pi CPU from
`top` during the runs.

### Parameters
| Name | TL-015 | Timer drive, paced sensors | Unit |
| ---- | ------ | -------------------------- | ---- |
| `drive_loop` rate | 864.7 | 1000.0 | Hz |
| `drive_loop` period mean / max | 1.16 / 2.72 | 1.000 / 1.000 | ms |
| `tx_slot` rate | 200.0 | 200.0 | Hz |
| `tx_slot` p99 / max | 6.45 / 6.48 | 5.60 / 5.57 | ms |
| `imu_loop` / distinct IMU samples | 503 / 503 | 250.0 / 250 | per s |
| `power_loop` | 964 | 100.0 | per s |
| `cmd_to_pwm` mean / max | 10 / 428 | 755 / 999 | us |
| `rx_loop` max | 2.79 | 1.65 | ms |
| `irq_late` max | 0 | 0 | us |
| Telemetry at the Pi (3 probe runs) | - | 197.1, 189.9, 196.7 | Hz |
| Teensy-stamped gaps at the Pi, median / max | - | 5.43 / 84 to 85 | ms |

### Observations
- Every profiling target passed: telemetry rate, gap p99 and max, drive loop
  mean and max, command to PWM, interrupt lateness. p99 values are 50 us bin
  edges, so a p99 can read above the exact max.
- `cmd_to_pwm` rose from 10 to 755 us mean: an accepted command now waits for
  the next 1 ms tick instead of running in the thread after `UARTRx`.
- Sensor values plausible (accel about 9.8 m/s^2 on z, gyro near 0, mag and
  battery steady); info reply `imu.read_hz=250`, `power.poll_ms=10`,
  `i2c.clock_hz=400000`, both sensors `ok`.
- The firmware sent 200.0 frames/s with no gap over 5.57 ms, but the Pi
  received 189.9 to 197.1 Hz: one gap of 84 to 85 ms (about 16 frames) per
  10 s run in the Teensy-stamped stream, plus scattered drops. Load average
  4.4, `pit_node` 50% and `imu_fusion_node` 42% of a core.

### Conclusion
Supported for the firmware. The remaining loss is on the Pi, between the
serial read in `pit_node` and the best-effort subscriber; it needs counters in
the driver. Driving checks (gamepad, RC, switches, stop on release) still to
run with the drive loop in an interrupt.

---

## TL-015 - 2026-10-25

### Setup
As TL-014; profiling build of `26d0fd4` plus `UARTRx` on `CommandFramer`
(frames built from available bytes, no `readBytes`). I2C at 400 kHz.

### Hypothesis
Without the `readBytes` wait, every thread gets more turns and telemetry
holds 200 Hz.

### Method
`scripts/profile_capture.py 60 --probe`; info reply through the new receive
path.

### Parameters
| Name | TL-014, 400 kHz | Non-blocking receive | Unit |
| ---- | --------------- | -------------------- | ---- |
| `drive_loop` rate | 620.2 | 864.7 | Hz |
| `drive_loop` period mean / p99 / max | 1.63 / 6.7 / 7.58 | 1.16 / 2.3 / 2.72 | ms |
| `tx_slot` rate | 194.3 | 200.0 | Hz |
| `tx_slot` p99 / max | 11.10 / 11.37 | 6.45 / 6.48 | ms |
| `rx_frame` mean / max | 4213 / 5108 | 8.2 / 9 | us |
| `cmd_to_pwm` mean / max | 26 / 425 | 10 / 428 | us |
| Distinct IMU samples | 391 | 503 | per s |
| Pi-side loss | none measurable | 0.7 | % |

### Observations
- 60.4 command frames/s accepted, 0 CRC failures, 0 stalled frames.
- Info reply over the new path: `fw.label=v0.9.0.4-pcb`, `i2c.clock_hz=400000`,
  both sensors `ok`.
- The capture first printed 201.4 Hz telemetry and 870.7 Hz drive loop: it
  divided by the window count while the windows ran slightly over 1 s (60
  reports over 60.41 s). Rates now come from the report timestamps; the
  figures above are recomputed that way. Earlier entries are overstated by
  under 1%.

### Conclusion
Supported. Drive loop 865 Hz with a 2.7 ms worst period; telemetry at 200 Hz
with gaps under 6.5 ms. Remaining gap to the targets is round-robin
scheduling (drive max 2.7 ms against 1.5 ms).

---

## TL-014 - 2026-10-25

### Setup
Reference car on a block, teleop running, transmitter off, LED strip on the
idle charge bar. Profiling build (`teensy41_profile`, `fw.profile=1`) of
`fc35e3a` plus the measurement suite. Battery 7.61 to 7.41 V over the runs.

### Hypothesis
Every 1 ms thread runs once per scheduler rotation, so `DriveControl` runs far
below 1 kHz; blocking I2C transfers set the rotation length.

### Method
`scripts/profile_capture.py 60` per variant (CSV in `data/`), `--probe` on two
of them for the Pi-side frame count. Variants: the committed config; 400 kHz
set only in `setup()`; INA226 read every 10 ms; 400 kHz applied again after
`lsm.begin()`. Telemetry probe and info reply on the last.

### Parameters
| Name | 100 kHz, INA 1 ms | 400 kHz in setup | 100 kHz, INA 10 ms | 400 kHz applied after begin | Unit |
| ---- | ----------------- | ---------------- | ------------------ | --------------------------- | ---- |
| `drive_loop` rate | 180.6 | 180.2 | 249.5 | 620.2 | Hz |
| `drive_loop` period mean / p99 / max | 5.57 / 10.0 / 10.2 | 5.56 / 9.4 / 10.0 | 4.03 / 8.75 / 9.76 | 1.63 / 6.7 / 7.58 | ms |
| `tx_slot` rate | 153.9 | 153.4 | 123.9 | 194.3 | Hz |
| `tx_slot` p99 / max | 14.75 / 15.25 | 13.70 / 14.54 | 12.05 / 13.41 | 11.10 / 11.37 | ms |
| `imu_read` | 3937 | 3937 | 3937 | 1090 | us |
| `power_read` | 995 | 995 | 995 | 275 | us |
| `rx_frame` mean / max | 1686 / 5101 | 1669 / 5055 | 3123 / 5089 | 4213 / 5108 | us |
| `led_show` mean / max | 194 / 198 | - | - | 194 / 200 | us |
| `irq_late` max | 0 | - | - | 0 | us |
| `cmd_to_pwm` mean / max | 44 / 439 | - | - | 26 / 425 | us |
| Distinct IMU samples | 181 | 180 | 181 | 391 | per s |
| Pi-side loss | 0.5 | - | - | none measurable | % |

### Observations
- Every 1 ms thread ran at the same rate (180.6 Hz at 100 kHz): one turn per
  scheduler rotation. LSM9DS1 and INA226 reads took 4.9 of the 5.57 ms.
- `Wire.setClock()` in `setup()` had no effect: Adafruit BusIO's `begin()`
  calls `Wire.begin()`, which resets the bus to 100 kHz. TL-012 therefore ran
  at 100 kHz. Applying the clock after `lsm.begin()` cut the reads 3.6x.
- At 400 kHz the accel, gyro, mag and battery values matched the 100 kHz runs;
  telemetry probe 192.9 and 196.0 Hz at the Pi, median gap 5.37 ms; info reply
  `i2c.clock_hz=400000`, both sensors `ok`.
- `FastLED.show()` costs 194 us and no interrupt ran late by 1 us or more; the
  LED strip does not block on this FastLED version.
- `UARTRx` busy-waits in `readBytes` until a whole 474-byte command frame has
  arrived (5.1 ms on the wire); with shorter rotations it starts earlier and
  waits longer (4.2 ms mean at 400 kHz, about 25% of the CPU at 60 Hz).
- `cmd_to_pwm` counts from frame accepted, so it misses time a frame waits in
  the serial buffer before `UARTRx` runs.
- `report_lines_dropped` was cumulative in the first run (78 lines from before
  the capture opened the port); it resets each second from the second run on.

### Conclusion
Supported. `DriveControl` ran at 181 Hz, not 1 kHz. Blocking I2C at an
unintended 100 kHz set the rotation; with the clock applied after `begin()`
the drive loop reached 620 Hz and telemetry 194 Hz. Remaining blockers: the
`readBytes` wait in `UARTRx`, blocking I2C, and round-robin scheduling
(drive-loop max 7.6 ms).

---

## TL-013 - 2026-10-25

### Setup
Reference car on a block, teleop running, transmitter off. Firmware
`00f6bf2` plus the staged tree: `fw.label`, the measured 15-point charge bar
curve, and the I2C knob back at 100 kHz. Battery 7.60 to 7.61 V.

### Hypothesis
At 100 kHz telemetry returns to the 152 Hz of TL-011.

### Method
`scripts/telemetry_probe.py 10 --runs 3` (checks that `pit_node` is the only
publisher) with the INA226 read every 10 ms, then the same with every 1 ms.
Info reply against a host build of the same tree.

### Parameters
| Name | 100 kHz, INA226 10 ms | 100 kHz, INA226 1 ms | Unit |
| ---- | --------------------- | -------------------- | ---- |
| Teensy-stamped rate (runs) | 125.6, 122.4, 124.1 | 152.3, 151.8, 152.8 | Hz |
| Messages received at the Pi (runs) | 125.9, 118.0, 123.2 | 152.6, 145.4, 132.6 | Hz |
| Stamp gap median | 8.25 to 8.29 | 6.28 to 6.45 | ms |
| Stamp gap p99 / max | 10.1 / 14.8 | 10.2 to 11.8 / 13.2 to 15.2 | ms |

### Observations
- The bus clock had no effect on its own: 100 kHz with the 10 ms INA226 read
  ran at the TL-012 rate.
- The first info request failed with a pyserial "multiple access on port"
  error, since `pit_node` reads the same port; the retry matched the host
  build (74 keys, `fw.label=dev`, `i2c.clock_hz=100000`).
- In two of three 1 ms runs the Pi received fewer messages than the Teensy
  stamps show it sent (145.4 and 132.6 against about 152 Hz).
- Accel, gyro, mag, battery and RC readings normal in every run.

### Conclusion
The INA226 poll rate set the telemetry rate: reading the battery less often
leaves the bus to `ReadIMUStates`, which then busy-waits through more
scheduler slices. Kept at 1 ms (the previous behavior) until the scheduler
change. The Pi-side message loss needs a count in the measurement suite.

---

## TL-012 - 2026-10-25

### Setup
Reference car on a block, teleop service running, after a reboot. Firmware
`00f6bf2` plus the staged I2C change: `cfg::I2C::clock_hz` 100000 -> 400000,
INA226 read every 10 ms instead of every 1 ms. Battery 8.05 V.

### Hypothesis
At 100 kHz one LSM9DS1 read holds the bus about 3 ms and starves `UARTTx`;
at 400 kHz telemetry rises toward 200 Hz.

### Method
Telemetry probe (best-effort subscriber, 10 s, one publisher checked each run)
once on the 100 kHz build after the reboot and three times after the flash.
Info reply read with `scripts/pit_uart.py --info`.

### Parameters
| Name | 100 kHz | 400 kHz | Unit |
| ---- | ------- | ------- | ---- |
| Telemetry rate (runs) | 151.9; earlier 152.2, 153.6, 153.1 | 127.1, 132.5, 91.8 | Hz |
| Stamp gap median | 6.25 | 8.20 to 8.30 | ms |
| Stamp gap p95 | 10.09 | 8.98 to 9.24 | ms |
| Stamp gap max | 15.20 | 14.48 to 14.82 | ms |
| Distinct battery voltages | 20.2 | 12.6 to 16.9 | per s |

### Observations
- The upload failed on the first write and succeeded on the retry (13.4 s).
- Info reply: `i2c.clock_hz=400000`, `power.poll_ms=10`, `status.imu=ok`,
  `status.power=ok`. Accel (-0.02, -0.15, +10.00 m/s^2), gyro, mag and
  battery voltage read normally.
- The third run counted 917 frames in 10 s against a median gap of 8.3 ms
  (about 120 Hz).
- TeensyThreads `delay()` loops on `yield()`, and each thread holds a 1 ms
  slice (10 ticks of 100 us). A thread busy-waiting on I2C keeps its whole
  slice, so `UARTTx` runs once per round of all eight contexts.

### Conclusion
Not supported. Telemetry fell by about 20% and the gaps moved from a 5/10 ms
mix to a steady 8.2 ms. The rate is bounded by round-robin wake-up of
`UARTTx`, not by bus time. This run changed the bus clock and the INA226
poll together; TL-013 separates them: the poll change caused the drop.
Next action: send telemetry independent of the thread rounds.

---

## TL-011 - 2026-10-25

### Setup
Reference car on a block, wheels off the ground, teleop service running.
Firmware built from the clean tree at `00f6bf2`. Battery 7.98 V.

### Hypothesis
A build from a committed tree reports its commit, clean, and runs as TL-009.

### Method
`pio run -t upload`; on failure, the same command again. Then
`python3 scripts/pit_uart.py --info` (teleop still running, as TL-010), a diff
against a host build of `writeFirmwareInfo` with `FW_GIT_HASH=00f6bf2`, and a
10 s telemetry probe.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| Telemetry rate | 153.3 | Hz |
| Stamp gap median / p95 / max | 6.23 / 10.09 / 15.07 | ms |
| Info keys | 72 | - |

### Observations
- The first upload rebooted the Teensy into HalfKay, then failed with "error
  writing to Teensy" after 259.7 s (build included). The retry, started with
  the Teensy already in HalfKay, programmed in 17.2 s. The same pattern
  occurred in TL-009.
- Info reply: `fw.git=00f6bf2`, `fw.git_dirty=0`, byte for byte equal to the
  host build.
- Telemetry topics, accel, gyro, mag and battery as in TL-009.

### Conclusion
Supported. Uploads from running firmware fail on the first write on this car;
rerunning `pio run -t upload` once completes them.

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
