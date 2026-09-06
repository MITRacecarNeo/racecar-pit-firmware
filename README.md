# racecar-pit-firmware

Teensy 4.1 firmware for the NEO-PIT drive PCB on the RACECAR Neo V2, plus the
Pi-side UART reference. The board replaces the Pololu Maestro and absorbs the
servo, ESC, LSM9DS1 IMU, INA226 power sensor, hall encoder, MAX7219 dot matrix,
WS2812B strips, and FlySky iA6B RC receiver. The Pi speaks a binary UART
protocol to the Teensy and sends normalized drive commands; the Teensy maps them
to servo and ESC PWM and streams telemetry back.

Pairs with [racecar_neo_ros2_driver](https://github.com/MITRacecarNeo/racecar_neo_ros2_driver),
which owns the Pi side (`pit_node`, `pit_protocol.py`). The two share the wire
format in `packets.h` and the version numbers that select a matching pair; see
[Versioning](#versioning).

## Table of contents

- [Versioning](#versioning)
- [Repository layout](#repository-layout)
- [Branches](#branches)
- [Wire protocol](#wire-protocol)
- [Drive control](#drive-control)
- [Telemetry](#telemetry)
- [RC receiver](#rc-receiver)
- [Idle displays](#idle-displays)
- [Build and flash](#build-and-flash)
- [Tests](#tests)
- [Architecture](#architecture)
- [Changelog](#changelog)
- [License](#license)

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
| 0.7.0 - 0.7.3 | 0.7.0 | unchanged |
| 0.6.0 | none tagged; run 0.5.0 | unchanged |
| 0.5.0 - 0.5.1 | 0.5.0 | `TXPacket` 94 B, `RxPacket` 470 B + 4 B magic |
| 0.4.0 | 0.4.0 | pre-mA current packing |
| 0.3.0 - 0.3.2 | 0.3.0 | pre-mA current packing |

The firmware skips from `0.5.0` to `0.7.0` to re-align with the driver. There
is no `0.6.0` firmware tag and there will not be one.

## Repository layout

```
racecar-pit-teensy/     PlatformIO project for the Teensy 4.1
  include/
    config.h            all tunables (UART, ESC, servo, encoder, RC, PID, drive model)
    packets.h           wire format (authority for both sides)
    Mutexes.h           per-resource locks for the TeensyThreads scheduler
    crc16.h             CRC-16/CCITT-FALSE, Arduino-free
    drive_map.h         normalized-to-PWM mapping and hall decode, Arduino-free
    helpers.h           thread and shared-state declarations
  src/
    main.cpp            boot: peripherals, interrupts, thread launch
    helpers.cpp         threads: telemetry, command RX, sensors, actuation, displays
    crc16.cpp
    drive_map.cpp
  test/test_logic/      host unit tests
  platformio.ini        teensy41 firmware env, native test env
racecar-pit-pi/
  PiUART.py             standalone Pi-side link reference (no ROS)
  tests/                pytest suite for the schemas and CRC
docs/                   architecture, changelog, test log
```

## Branches

- `main` is the closed-loop build. The Pi throttle command becomes a wheel-speed
  setpoint that a PI controller holds against the hall encoder.
- `pwm-control` targets a car variant with no encoder: actuation is open-loop
  PWM passthrough, no setpoint and no PID. It is kept on purpose for that
  hardware and should not be deleted.

## Wire protocol

UART over the Pi GPIO14/15 header to the Teensy `Serial1`, 921600 baud,
little-endian packed structs (`packets.h`). CRC-16/CCITT-FALSE covers every byte
before the trailing 2-byte checksum, matching the driver's
`pit_protocol.crc16_ccitt` (check value `0x29B1`).

| Direction | Struct | Magic | Size | Rate |
|---|---|---|---|---|
| Teensy to Pi (telemetry) | `TXPacket` | `0xDEADBEEF` | 94 B | 200 Hz (`cfg::UART::tx_freq`) |
| Pi to Teensy (command) | `RxPacket` | `0xBEEFDEAD` | 470 B + 4 B magic | 60 Hz, driver-side |

Both struct sizes are pinned by `static_assert`; changing one is a protocol
break that has to land on both sides together. Telemetry carries a microsecond
timestamp, INA226 bus voltage and current, the FlySky RC channels, the LSM9DS1
IMU (9 axes), the encoder, and a reserved EKF slot. Commands carry the
normalized servo angle and motor speed, the dot-matrix and LED buffers, and a
system-state byte. `verify_rx_crc` (default true) drops corrupt command frames.

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
- The Pi throttle is closed loop. The normalized magnitude maps onto
  `min_speed_mps`..`max_speed_mps` (0.75 to 5.0 m/s) and a PI controller with
  feedforward holds that setpoint against the hall encoder, clamped to
  `speed_pwm_min`..`speed_pwm_max`. A command inside `cmd_deadband` writes
  neutral and resets the integrator.
- A command stale past `rx_timeout` (50 ms) falls to neutral. A dropped or
  absent receiver holds neutral instead of extrapolating to full reverse.

The ESC has a wide deadband either side of neutral. A static feedforward term
(`deadband_fwd_us` / `deadband_rev_us`) carries the output across it, `kp_boost`
adds authority once the error exceeds `err_threshold`, and
`stiction_kick_rev_us` adds reverse PWM while the wheels are still. The
integrator only accumulates when the output is off its limit or the error pushes
back toward the usable range. `speed_pwm_min` sits deeper than a symmetric
forward throw to clear the reverse deadband.

Polarity is corrected on the Pi side (`pit.yaml` `speed_sign` /
`steering_sign`), not here.

## Telemetry

`fillTxBuff` packs the LSM9DS1 (9 axes via `getEvent(accel, mag, gyro)`), INA226
bus voltage (mV) and current (mA), the eight FlySky RC channels (raw pulse
widths, us), and a microsecond timestamp. Current is packed as mA rather than
truncated amps so sub-amp resolution survives the `uint16`. The encoder field
carries signed ground speed in m/s, not RPM: `getRPM` averages the last five
hall intervals and zeroes on a hall-transition timeout (`stop_timeout_us`,
30 ms) instead of reporting a ghost speed from a stale interval, and
`getCurrentSpeed` applies the two gear ratios and the wheel circumference.

The `int32` timestamp is `micros()` and wraps every ~35.8 minutes; treat it as a
wrapping counter.

## RC receiver

The FS-iA6B feeds PPM into pin 5; `RxReceive` copies channels 1-8 into
`channelVals[0..7]`, so index N-1 is PPM channel N. The transmitter emits AETR
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

A stock iA6B carries six channels over PPM, so CH7 and CH8 arrive only if the
transmitter and receiver are both set up to emit eight. `PPMReader` reports 0
for a channel that is absent, out of band, or stale past its 500 ms failsafe,
and clamps everything else into 1000-2000 us; `rc_valid_min`/`rc_valid_max`
(900/2100) therefore only reject the 0 case.

All eight slots are always published, including the two the iA6B never fills;
narrowing the array would change `TXPacket` and break the wire. See
[Drive control](#drive-control) for the RC-over-Pi priority rules.

### Switches

The FS-i6 aux order puts swA through swD on CH5 through CH8. A pulse at or
above `sw_threshold` (1500 us) is up; anything below, and any channel that is
absent or outside the valid band, is down.

swD lands past the six channels a stock iA6B carries over PPM, so the link has
to be emitting eight channels for the override to arrive. A channel that never
shows up reads as down, which means the override silently never engages;
confirm CH8 moves in telemetry before relying on it.

| Switch | Ch | Index | Down | Up |
| ------ | -- | ----- | ---- | -- |
| swA | CH5 | 4 | fast: speed capped by `max_speed_mps` (5.0 m/s) | slow: capped at `slow_speed_mps` (1.0 m/s) |
| swD | CH8 | 7 | Pi autonomy commands accepted | manual override, Teensy drives alone |

Both default to the left column when the receiver is absent, so losing the link
never leaves the car stuck slow or the Pi locked out. The decode is
`rcSwitchUp()` in `drive_map.h`, host tested.

Slow mode acts in two places. On the Pi path it clamps the closed-loop setpoint
to `slow_speed_mps`. The RC path is open loop with no setpoint to clamp, so it
trims the PWM throw about neutral by `slow_rc_scale`; that scalar is provisional
and wants a bench pass against a measured 1.0 m/s. Override detection runs on
the untrimmed throw, so slow mode never makes it harder to grab the car back.

Manual override locks the Pi out of drive entirely: with the sticks centered the
car holds neutral instead of handing back to autonomy. It also takes the dot
matrix, which shows `M` while the sticks are live and `I` otherwise; the Pi's
bitmap and its auto glyph are both suppressed, since `A` has no meaning while
autonomy is locked out. The LED strip is untouched and the Pi can still drive it.

### Driver agreement

Both switches ride in the existing telemetry RC array, so this needs no protocol
change. The driver reads the same two channels from `/rc/channels` and applies
the same threshold: index 4 for speed mode, index 7 for override, up at or above
1500 us raw (0.0 normalized). While override is up the driver should stop
publishing drive commands rather than have the Teensy discard them, and in slow
mode it should cap its own setpoint at 1.0 m/s so the two ends agree on the
limit instead of fighting over it.

## Idle displays

With no fresh Pi command the dot matrix shows the drive-mode glyph and the LED
strip shows a red charge bar filling from both ends.

The bar reads state of charge, not volts. A 2S LiPo runs from 8.4 V (4.20
V/cell) down to 7.0 V (3.50 V/cell), but the curve is flat between roughly 7.6
and 8.0 V and falls off below 7.4 V, so a bar drawn linearly on voltage sits
near full for most of a run and then empties in seconds. `battery.h` holds a
ten-point piecewise-linear approximation of that curve; `batterySocPercent`
returns 0-100% and the bar length follows it.

## Build and flash

PlatformIO with the Teensy platform. From `racecar-pit-teensy/`:

```sh
pio run              # build
pio run -t upload    # flash over USB (Teensy on /dev/ttyACM0)
```

If `pio` is not on PATH it lives in the PlatformIO virtualenv, usually
`~/.platformio/penv/bin/pio`.

`/dev/ttyACM0` is the USB debug and flash port only. The drive link is the GPIO
UART (Teensy `Serial1` to the Pi's `/dev/neo-pit-pcb`, a udev symlink to
`ttyAMA0` on Pi 5 / Ubuntu). The Pi needs `enable_uart=1` and a reboot to
activate GPIO14/15.

Set `cfg::DEBUG::serial_plot` to enable the Teleplot-style control-loop trace on
the USB serial port. It is off by default because it runs inside the 1 kHz
actuation loop.

## Tests

Host unit tests over the Arduino-free logic (CRC, PWM mapping, hall decode),
from `racecar-pit-teensy/`:

```sh
pio test -e native
```

Fourteen cases over the CRC, the PWM mapping, the hall decode, and the 2S
state-of-charge curve.

Pi-side schema and CRC tests, from `racecar-pit-pi/`:

```sh
pytest
```

Hardware runs are recorded in `docs/test-log.md`. The most recent on-robot
session (2026-07-06, wheels up) streamed `/imu/lsm9ds1` at 151 Hz with no CRC or
resync errors and drove the car through the ROS mux in both directions.

## Architecture

See `docs/architecture.md` for the thread and mutex map, the data pipeline, and
the pin assignments.

## Changelog

Latest entries; full history in `docs/changelog.md`.

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
