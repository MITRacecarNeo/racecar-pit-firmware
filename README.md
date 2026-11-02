# racecar-pit-firmware

Teensy 4.1 firmware for the NEO-PIT drive PCB on the RACECAR Neo V2. The board
replaces the Pololu Maestro and absorbs the servo, ESC, LSM9DS1 IMU, INA226 power sensor, hall encoder, MAX7219 dot matrix,
WS2812B strips, and FlySky iA6B RC receiver. The Pi speaks a binary UART
protocol to the Teensy and sends normalized drive commands; the Teensy maps them
to servo and ESC PWM and streams telemetry back.

Pairs with [racecar_neo_ros2_driver](https://github.com/MITRacecarNeo/racecar_neo_ros2_driver),
which owns the Pi side (`pit_node`, `pit_protocol.py`). The two share the wire
format in `src/packets.h` and the version numbers that select a matching pair; see
[Versioning](#versioning).

## Table of contents

- [Hardware](#hardware)
- [Versioning](#versioning)
- [Repository layout](#repository-layout)
- [Branches](#branches)
- [Wire protocol](#wire-protocol)
- [Firmware info](#firmware-info)
- [Drive control](#drive-control)
- [Telemetry](#telemetry)
- [RC receiver](#rc-receiver)
- [Idle displays](#idle-displays)
- [Build and flash](#build-and-flash)
- [ESC calibration](#esc-calibration)
- [Tests](#tests)
- [Architecture](#architecture)
- [Changelog](#changelog)
- [License](#license)

## Hardware

This firmware requires the NEO-PIT PCB. It does not run on a stock RACECAR Neo
with the Pololu Maestro. The PCB fabrication files are open hardware. They are
not yet public; publication is pending release review.

## Versioning

**High priority. The major and minor version here mirror
[racecar_neo_ros2_driver](https://github.com/MITRacecarNeo/racecar_neo_ros2_driver).
Flash firmware `X.Y.*` to run driver `X.Y.*`.** The patch number is independent
on each side; a firmware patch fixes something on the Teensy alone.

Rules for cutting a release:

- A change to `packets.h`, the baud rate, or the command semantics is a protocol
  break. Both repos move to the same next minor, firmware tagged first.
- A driver minor bump that never touches the link (networking, Coral, ROS nodes)
  still takes the firmware to that minor, with or without firmware changes, so
  the pairing stays readable from the tag alone.
- A firmware-only fix bumps the patch and leaves the minor pinned to the driver.
- Record the paired driver version in the changelog entry and update the table
  below.

| Driver | Firmware | Wire format |
| ------ | -------- | ----------- |
| 0.9.x | 0.9.0 | sizes unchanged; adds the info reply |
| 0.7.4 - 0.9.0 | 0.7.1 | unchanged |
| 0.6.0 | none tagged; run 0.5.0 | unchanged |
| 0.5.0 - 0.5.1 | 0.5.0 | `TXPacket` 94 B, `RxPacket` 470 B + 4 B magic |
| 0.4.0 | 0.4.0 | pre-mA current packing |
| 0.3.0 - 0.3.2 | 0.3.0 | pre-mA current packing |

The firmware skips from `0.5.0` to `0.7.0` and from `0.7.1` to `0.9.0` to
re-align with the driver. There are no `0.6.0` or `0.8.0` firmware tags.

## Repository layout

```
platformio.ini          firmware envs (teensy41, teensy41_profile, teensy41_wdt_test), host tests (native)
config/
  config.h              all tunables (UART, ESC, servo, encoder, RC, PID, IMU)
src/                    Teensy firmware
  main.cpp              boot: peripherals, interrupts, thread launch
  shared.{h,cpp}        state shared with the interrupts (sequence counters)
  link.{h,cpp}          telemetry timer, info builder, command receive
  drive.{h,cpp}         1 kHz drive tick, speed loop, hall encoder
  sensors.{h,cpp}       INA226, LSM9DS1, FlySky PPM threads
  displays.{h,cpp}      dot matrix thread, LED strip
  watchdog.{h,cpp}      RTWDOG and reset cause; heartbeat.h per-context bits
  packets.{h,cpp}       wire format, info chunk builder
  command_framer.*      command frame assembly from arriving bytes
  info.{h,cpp}          info payload; info_buffers.h double buffer
  seqlock.h             sequence counter for interrupt-shared state
  profile.*, prof_stats.*     profiling build hooks and statistics
  version.h             firmware version
  crc16, drive_map, battery   Arduino-free logic
tests/
  test_*/               host unit tests (pio test -e native)
  scripts/              pytest suite for scripts/
scripts/
  build_flags.py        git commit flags for the firmware build
  upload.py             upload command with retry
  pit_uart.py           bench tool: drive the link from the Pi without ROS
  profile_capture.py    record and check a profiling run
  telemetry_probe.py    telemetry rate and gaps as the driver publishes them
data/                   hardware captures for the test log
docs/                   protocol, architecture, changelog, test log
pyproject.toml          pytest, ruff, black, mypy settings for scripts/
```

The Arduino-free files in `src/` and `config/config.h` also build on the host,
so the tests read the shipped values. The Pi side of the link lives in
`racecar_neo_ros2_driver`; `scripts/pit_uart.py` is only for bench bring-up.

## Branches

- `main` is the closed-loop build. The Pi throttle command becomes a wheel-speed
  setpoint that a PI controller holds against the hall encoder.
- `pwm-control` targets a car variant with no encoder: actuation is open-loop
  PWM passthrough, no setpoint and no PID. It is kept on purpose for that
  hardware and should not be deleted.

## Wire protocol

UART over the Pi GPIO14/15 header to the Teensy `Serial1`, 921600 baud,
little-endian packed structs (`src/packets.h`). CRC-16/CCITT-FALSE
covers every byte before the trailing 2-byte checksum, matching the driver's
`pit_protocol.crc16_ccitt` (check value `0x29B1`). Full layout:
[docs/protocol.md](docs/protocol.md).

| Direction | Struct | Magic | Size | Rate |
|---|---|---|---|---|
| Teensy to Pi (telemetry) | `TXPacket` | `0xDEADBEEF` | 94 B | 200 Hz (`cfg::UART::tx_freq`) |
| Pi to Teensy (command) | `RxPacket` | `0xBEEFDEAD` | 470 B + 4 B magic | 60 Hz, driver-side |
| Teensy to Pi (info chunk) | `HeaderPacket` + chunk | `0xFEEDC0DE` | 18 to 146 B | on request |

Both struct sizes are pinned by `static_assert`; changing one is a protocol
break that has to land on both sides together. Telemetry carries a microsecond
timestamp, INA226 bus voltage and current, the FlySky RC channels, the LSM9DS1
IMU (9 axes), the encoder, and a reserved EKF slot. Commands carry the
normalized servo angle and motor speed, the dot-matrix and LED buffers, and a
system-state byte. `verify_rx_crc` (default true) drops corrupt command frames.

## Firmware info

The Pi can ask the Teensy what it is running: the firmware version and
commit, and the compiled configuration. That covers PWM bounds, the speed
range and slow-mode cap, PID gains, drivetrain and encoder constants, the RC
channel map, IMU ranges and units, power sensor settings, and sensor status.
The driver asks once when it starts and stores the answer with the car's
settings.

The Pi sets bit `0x20` of `SystemState`. While it is set, the Teensy sends one
info chunk after each telemetry frame, so telemetry keeps its rate. The
payload is ASCII `key=value` lines (about 1.5 kB in 12 chunks, 60 ms per
cycle). Keys and the reassembly rules are in
[docs/protocol.md](docs/protocol.md#info-reply).

On the bench, with the teleop service stopped:

```sh
python3 scripts/pit_uart.py --info                       # print the reply
python3 scripts/pit_uart.py --info --yaml firmware.yaml  # also write YAML
```

Firmware older than 0.9.0 does not answer; the script exits with an error
after 2 s.

## Drive control

Actuation follows an RC-priority, Pi-fallback model with no arm switch:

- The RC transmitter overrides the Pi whenever a stick's mapped PWM leaves its
  neutral band (`throt_override_band` / `steer_override_band`, 40 us), gated on
  a plausible receiver pulse so a dropped or absent receiver never engages RC.
  The RC path writes PWM straight through. See [RC receiver](#rc-receiver) for
  which stick drives which axis.
- With the sticks centered or no receiver, a fresh Pi command drives. Steering
  is open loop: a normalized `[-1, 1]` angle maps piecewise across
  `angle_pwm_min`/`angle_pwm_neutral`/`angle_pwm_max`.
- The Pi throttle is closed loop, as is the RC throttle in slow mode. The
  normalized magnitude maps onto `min_speed_mps`..`max_speed_mps` (0.5 to
  1.0 m/s in the indoor build, 4.0 m/s ceiling in the field build), or onto `slow_speed_mps` in slow mode, and a PI controller with
  feedforward holds that setpoint against the hall encoder, clamped to
  `speed_pwm_min`..`speed_pwm_max`. A command inside `cmd_deadband` writes
  neutral and resets the integrator.
- A command stale past `rx_timeout` (50 ms) falls to neutral. A dropped or
  absent receiver holds neutral instead of extrapolating to full reverse.

The ESC has a deadband either side of neutral. A feedforward term per
direction (`deadband_fwd_us` plus `kv_ff_fwd` per m/s, `deadband_rev_us` plus
`kv_ff_rev`) carries the output across it, and `kp_boost` adds authority once
the error exceeds `err_threshold`; in reverse the boost applies only near a
standstill. The output stays on the setpoint's side of neutral, because a
forward pulse while reversing makes this ESC brake. The integrator only
accumulates when the output is off its limit or the error pushes back toward
the usable range.

From a standstill a twitch of a few hall edges reads as real speed, so motion
counts as confirmed only after `start_confirm_intervals` (5) hall intervals
spanning `start_confirm_us` (100 ms) in the target direction
(`src/start_assist.h`). Until then a per-direction start assist holds the
output at least on a ramp: reverse from 160 to 240 us off neutral at
1000 us/s, the integrator frozen. With no confirmed motion after 0.6 s the
output rests at neutral for 0.3 s and the ramp starts over. The reverse boost
ends at confirmed motion. Forward has no assist in the shipped tuning.
`speed_pwm_min` and `speed_pwm_max` (1000 and
2000 us) equal the ESC throttle calibration endpoints (see
[ESC calibration](#esc-calibration)). The feedforward is fitted open loop on
that calibration (`docs/test-log.md`, TL-025).

Polarity is corrected on the Pi side (`pit.yaml` `speed_sign` /
`steering_sign`), not here.

## Telemetry

A 200 Hz timer interrupt assembles each frame at send time from the latest
LSM9DS1 sample (accel m/s^2, gyro rad/s, mag uT), the INA226 bus voltage (mV)
and current (mA), the eight FlySky RC channels (raw pulse widths, us), and a
microsecond timestamp. Current is packed as mA rather than truncated amps so
sub-amp resolution survives the `uint16`. The encoder field carries signed
ground speed in m/s, not RPM, as computed by the drive tick: it takes the
median of the last five hall intervals, zeroes on a hall-transition timeout
(`stop_timeout_us`, 30 ms) instead of reporting a ghost speed from a stale
interval, and applies the two gear ratios and the wheel circumference. The
sign follows the hall sequence; a reversal needs two agreeing transitions.

The `int32` timestamp is `micros()` and wraps every ~35.8 minutes; treat it as a
wrapping counter.

## RC receiver

The FS-iA6B feeds PPM into pin 5; `RxReceive` copies channels 1-8 into
slots 0-7 of the shared RC channel block, so index N-1 is PPM channel N. The transmitter emits AETR
order.

| Index | PPM ch | Source | Centering | Used by |
| ----- | ------ | ------ | --------- | ------- |
| 0 | CH1 | aileron, right stick horizontal | sprung | telemetry only |
| 1 | CH2 | elevator, right stick vertical | sprung | ESC PWM (`ch_throt`) |
| 2 | CH3 | throttle, left stick vertical | ratcheted | telemetry only |
| 3 | CH4 | rudder, left stick horizontal | sprung | steering PWM (`ch_steer`) |
| 4 | CH5 | swA | switch | speed mode (`ch_speed_mode`) |
| 5 | CH6 | swB | switch | telemetry only |
| 6 | CH7 | swC | switch | telemetry only |
| 7 | CH8 | swD | switch | manual override (`ch_manual_override`) |

Drive reads the two sprung axes: **right stick vertical for throttle, left stick
horizontal for steering**. The transmitter's own throttle axis (CH3) is
ratcheted and rests at its low endpoint, so the RC override would read it as a
held reverse command and lock the Pi out for as long as the transmitter was on.
Both drive axes now return to center on release.

Which way is forward and which way is left is set on the transmitter (Functions
> Reverse), not in firmware.

The iA6B has six servo rails, but its PPM stream carries the whole frame, so
all eight slots populate; CH8 is confirmed live on hardware. `PPMReader` reports
0 for a channel that is absent, out of band, or stale past its 500 ms failsafe,
and clamps everything else into 1000-2000 us; `rc_valid_min`/`rc_valid_max`
(900/2100) therefore only reject the 0 case.

All eight slots are always published, including the two the iA6B never fills;
narrowing the array would change `TXPacket` and break the wire. See
[Drive control](#drive-control) for the RC-over-Pi priority rules.

### Switches

The FS-i6 aux order puts swA through swD on CH5 through CH8, all four of which
arrive over PPM. A pulse at or above `sw_threshold` (1500 us) is high; a
plausible pulse below it is low; a channel that is absent or outside the
valid band is neither. On the FS-i6 as set up, a lever pointing up (away from
the operator) sends low.

| Switch | Ch | Index | Lever up: low (default) | Lever down: high | No receiver signal |
| ------ | -- | ----- | ----------------------- | ---------------- | ------------------ |
| swA | CH5 | 4 | slow: capped at `slow_speed_mps` (1.0 m/s) | fast: capped by `max_speed_mps` (1.0 m/s indoor, 4.0 m/s field) | fast |
| swD | CH8 | 7 | Pi commands accepted | manual override, Teensy drives alone | Pi commands accepted |

The FS-i6 refuses to start transmitting until every lever is up, so the car
sees slow mode and Pi control when the transmitter comes on. A receiver with
no transmitter leaves the Pi in control. Reversing a switch channel on the
transmitter swaps its two columns. The decode is `rcSwitchUp()` (high) and
`rcSwitchDown()` (low) in `drive_map.h`, host tested.

Slow mode rescales the full stick travel onto `slow_speed_mps` on both paths,
so full throttle is a 1.0 m/s setpoint exactly rather than a clipped `max_speed_mps`
command. It rescales rather than clips, which keeps stick resolution usable
instead of pinning most of the travel at the cap.

Because a cap in m/s cannot be held open loop, slow mode closes the loop on the
RC stick too, running it through the same PI controller and encoder as the Pi
path. Fast-mode RC keeps the direct PWM path, so a human override there stays
instant.

Manual override locks the Pi out of drive entirely: with the sticks centered the
car holds neutral instead of handing back to autonomy. It also takes the dot
matrix, which shows `M` while the sticks are live and `I` otherwise; the Pi's
bitmap and its auto glyph are both suppressed, since `A` has no meaning while
autonomy is locked out. The LED strip is untouched and the Pi can still drive it.

### Driver agreement

Both switches ride in the existing telemetry RC array, so this needs no protocol
change. The driver reads the same two channels from `/rc/channels` and applies
the same threshold: index 4 for speed mode, index 7 for override, up at or above
1500 us raw (0.0 normalized) for high: speed mode index 4 is slow when low,
override index 7 is on when high. The driver is not required to act on either
switch. Command frames are received, CRC-checked, and stored no matter what the
switches say; override is applied at actuation only, so the Pi can keep
publishing at its own rate and never blocks or gridlocks on the Teensy. Reading
the switches is therefore optional and only useful for telling the operator what
the car is doing. Flipping swD back up resumes drive on the next frame,
with no reconnect.

## Idle displays

With no fresh Pi command the dot matrix shows the drive-mode glyph and the LED
strip shows a red charge bar filling from both ends.

The bar reads state of charge, not volts. A 2S LiPo runs from 8.4 V (4.20
V/cell) down to 7.0 V (3.50 V/cell), but not evenly: under the car's idle load
half the charge is gone by 7.55 V, and the last 10% falls between 7.3 and
7.0 V in a few minutes. `src/battery.cpp` holds a 15-point measured discharge
curve of load voltage against charge; `batterySocPercent` interpolates it to
0-100% and the bar length follows. Under driving load the voltage sags
further, so the bar reads low while the car drives.

## Build and flash

PlatformIO with the Teensy platform. From the repository root:

```sh
pio run              # build
pio run -t upload    # flash over USB (Teensy on /dev/ttyACM0)
```

If `pio` is not on PATH it lives in the PlatformIO virtualenv, usually
`~/.platformio/penv/bin/pio`. The upload runs `scripts/upload.py`: on this
board the first write after a soft reboot fails, so the script retries and
prints the attempt that succeeded. Linux needs PJRC's `00-teensy.rules` in
`/etc/udev/rules.d/` (a copy ships in PlatformIO's `tool-teensy` package). The platform and every library are pinned in
`platformio.ini`, so one commit always builds the same firmware. The build
embeds the short commit and a dirty flag (`scripts/build_flags.py`); the info
reply reports both.

`/dev/ttyACM0` is the USB debug and flash port only. The drive link is the GPIO
UART (Teensy `Serial1` to the Pi's `/dev/neo-pit-pcb`, a udev symlink to
`ttyAMA0` on Pi 5 / Ubuntu). The Pi needs `enable_uart=1` and a reboot to
activate GPIO14/15.

The Teleplot-style speed-loop trace is a build option:

```sh
PLATFORMIO_BUILD_FLAGS="-DRCN_SERIAL_PLOT" pio run -t upload
```

It prints the latest target speed, measured speed and ESC PWM on the USB
serial port from `loop()` every 25 ms, so it samples the 1 kHz loop rather
than following every tick. Lines are skipped while nothing reads the port.
Flash the plain build afterward.

Two test builds replace the drive behavior; never ship either:

| Environment | `fw.speed_control` | Behavior |
| ----------- | ------------------ | -------- |
| `teensy41_esc_cal` | `esc_cal` | ESC held at one calibration step, chosen by `scripts/esc_cal.py`; no driving |
| `teensy41_raw_pwm` | `raw_pwm` | Pi command n drives the ESC open loop at 1500 + 1000 * n us (limit +/- 300 us); RC unchanged |

## ESC calibration

The ESC stores its own throttle range. Every car is calibrated to the
firmware's PWM bounds: 1500 us neutral, 2000 us full forward, 1000 us full
reverse. A car calibrated to other endpoints responds differently to the same
PWM, and the speed-loop feedforward no longer fits it.

1. Flash the calibration build: `pio run -e teensy41_esc_cal -t upload`.
2. Run `python3 scripts/esc_cal.py` on the Pi with the Teensy USB cable
   plugged in, and follow its prompts. The car is on a stand with the wheels
   off the ground, and the ESC enters calibration mode from its SET button.
3. The LED strip shows the held step as a purple bar from both ends: one LED
   per end at full reverse, half the strip at neutral, the whole strip at full
   forward.
4. Flash the normal firmware: `pio run -e teensy41 -t upload`.

The build accepts full forward only from neutral and full reverse only from
full forward. A held step returns to neutral when USB serial has been quiet
for 2 s (`cfg::ESC_CAL::link_timeout_ms`); the script polls every 0.5 s. The
script sends no throttle until the operator confirms the ESC LED is flashing.

## Tests

Host unit tests over the Arduino-free logic, from the repository root:

```sh
pio test -e native
```

79 cases in seven suites: `test_logic` (CRC, PWM mapping, RC stick map, hall
decode and direction filter, interval median, switches, speed setpoints,
state-of-charge curve), `test_info` (payload format and content, chunking,
the reference packet), `test_framer` (command frame assembly), `test_seqlock`,
`test_info_buffers`, `test_heartbeat`, and `test_prof` (profiling
statistics).

Bench tool tests and checks, from the repository root:

```sh
pytest
ruff check scripts tests/scripts && black --check scripts tests/scripts && mypy
```

### Profiling

```sh
pio run -e teensy41_profile -t upload                 # firmware with timing hooks
python3 scripts/profile_capture.py 60 --tag baseline  # 60 s, CSV under data/
python3 scripts/telemetry_probe.py 10 --runs 3        # telemetry as pit_node publishes it
```

The profiling build reports thread periods, section timings, command-to-PWM
latency, and interrupt lateness on the USB serial port once per second; see
`docs/architecture.md`. Flash the plain build (`pio run -t upload`) afterward.

### Watchdog test

```sh
pio run -e teensy41_wdt_test -t upload   # test firmware; never ship it
```

Sending one character on the USB serial port (`/dev/ttyACM0`) stops one
context; the watchdog resets the Teensy within about 150 ms and the info reply
then reads `status.last_reset=watchdog`. The letters are listed in
`docs/architecture.md`. Flash the plain build afterward.

Hardware runs are recorded in `docs/test-log.md`.

## Architecture

See `docs/architecture.md` for the threads and interrupts, the shared state,
the data pipeline, and the pin assignments, and `docs/protocol.md` for the wire format.

## Changelog

Latest entries; full history in `docs/changelog.md`.

### [Unreleased]
- ESC calibration build `teensy41_esc_cal` and `scripts/esc_cal.py`; purple
  LED bar for the held step.
- Raw-PWM test build `teensy41_raw_pwm` for PWM-to-speed sweeps.
- ESC PWM bounds 1000/2000 us (were 800/2200); RC fast mode 1375/1625 us.

### [0.9.0] - 2026-10-26
Pairs with driver v0.9.x; telemetry and command sizes unchanged.
- Added: firmware info reply (version, commit, compiled configuration, sensor
  status) on request through `SystemState` bit `0x20`; hardware watchdog
  (100 ms); profiling and watchdog test builds; `scripts/pit_uart.py`,
  `scripts/upload.py`; `docs/protocol.md`.
- Changed: drive loop in a 1 kHz timer interrupt and telemetry in a 200 Hz
  timer interrupt (every frame gap 5.00 ms, was 141.6 Hz); I2C at 400 kHz;
  speed loop refit on the floor (forward within 0.04 m/s, was 0.21 to 0.25
  m/s fast); swA slow on the low pulse; repository layout; platform and
  libraries pinned.
- Fixed: speed spikes from bounce intervals and sign flips in the speed
  estimate; the I2C clock never left 100 kHz; a missing sensor held the I2C
  bus.
- Removed: the `fillTxBuff` thread, unused mutexes, `BasicLinearAlgebra`.

### [0.7.1] - 2026-09-06
- Changed: slow mode is an exact 1.0 m/s setpoint on both paths. RC slow mode
  now closes the loop on the encoder instead of trimming the PWM throw by a
  guessed scalar, and full stick rescales onto the cap rather than clipping.
- Removed: `slow_rc_scale`.

### [0.7.0] - 2026-09-06
Pairs with driver v0.7.x; wire format unchanged from 0.5.0.
- Added: `battery.{h,cpp}` 2S state-of-charge curve, host unit tests, Pi-side
  pytest suite, wire-size and config-invariant `static_assert`s, `docs/`.
- Changed: the idle LED bar spans 7.0-8.4 V and fills by charge, not voltage;
  `cfg::FLYSKY_IA6B_RX` documents the real AETR channel map; long comments
  trimmed across the firmware; Arduino-free logic split into `crc16` and
  `drive_map`; `config.h` is now authoritative for the sensor, timing, and RC
  settings that `helpers.cpp` used to hardcode.
- Fixed: `initRxHeader` left `FrameBytes` unset and wrote `IDNumber` twice;
  `PiUART.py` had a 2-byte schema mismatch on the telemetry encoder field and
  sent commands with a zero checksum.
- Removed: the superseded `src/claude.txt`, the dead `InnerControlLoop` thread,
  and unused globals and mutexes.

## License

See [LICENSE](./LICENSE).
