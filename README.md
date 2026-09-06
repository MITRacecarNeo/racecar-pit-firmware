# racecar-pit-firmware

Teensy 4.1 firmware for the NEO-PIT drive PCB on the RACECAR Neo V2, plus the
Pi-side UART reference. The board replaces the Pololu Maestro and absorbs the
servo, ESC, LSM9DS1 IMU, INA226 power sensor, hall encoder, MAX7219 dot matrix,
WS2812B strips, and FlySky iA6B RC receiver. The Pi speaks a binary UART
protocol to the Teensy and sends normalized drive commands; the Teensy maps them
to servo and ESC PWM and streams telemetry back.

Pairs with [racecar_neo_ros2_driver](https://github.com/MITRacecarNeo/racecar_neo_ros2_driver)
v0.5.0, which owns the Pi side (`pit_node`, `pit_protocol.py`). The two share the
wire format in `packets.h`.

## Table of contents

- [Repository layout](#repository-layout)
- [Branches](#branches)
- [Wire protocol](#wire-protocol)
- [Drive control](#drive-control)
- [Telemetry](#telemetry)
- [Build and flash](#build-and-flash)
- [Tests](#tests)
- [Architecture](#architecture)
- [Changelog](#changelog)
- [License](#license)

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
  The RC path writes PWM straight through.
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

### [Unreleased]
- Added: host unit tests, Pi-side pytest suite, wire-size and config-invariant
  `static_assert`s, `docs/`.
- Changed: Arduino-free logic split into `crc16` and `drive_map`; `config.h` is
  now authoritative for the sensor, timing, and RC settings that `helpers.cpp`
  used to hardcode.
- Fixed: `initRxHeader` left `FrameBytes` unset and wrote `IDNumber` twice;
  `PiUART.py` had a 2-byte schema mismatch on the telemetry encoder field and
  sent commands with a zero checksum.
- Removed: the superseded `src/claude.txt`, the dead `InnerControlLoop` thread,
  and unused globals and mutexes.

## License

See [LICENSE](./LICENSE).
