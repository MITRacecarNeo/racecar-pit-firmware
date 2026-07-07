# racecar-pit-firmware

Teensy 4.1 firmware for the NEO-PIT drive PCB on the RACECAR Neo V2, plus the Pi-side UART reference. The board replaces the Pololu Maestro and absorbs the servo, ESC, LSM9DS1 IMU, INA226 power sensor, hall encoder, MAX7219 dot matrix, WS2812B strips, and FlySky iA6B RC receiver. The Pi speaks a binary UART protocol to the Teensy and sends normalized drive commands; the Teensy maps them to servo/ESC PWM and streams telemetry back.

Pairs with [racecar_neo_ros2_driver](https://github.com/MITRacecarNeo/racecar_neo_ros2_driver) v0.4.0, which owns the Pi side (`pit_node`, `pit_protocol.py`). The two share the wire format in `packets.h`.

## Contents

- [Repository layout](#repository-layout)
- [Wire protocol](#wire-protocol)
- [Drive control](#drive-control)
- [Telemetry](#telemetry)
- [Build and flash](#build-and-flash)
- [Verification status](#verification-status)
- [License](#license)

## Repository layout

```
racecar-pit-teensy/     PlatformIO project for the Teensy 4.1
  include/
    config.h            all tunables (UART, ESC, servo, encoder, RC, drive model)
    packets.h           wire format (authority for both sides)
    Mutexes.h           per-resource locks for the TeensyThreads scheduler
    helpers.h
  src/
    main.cpp            thread setup, control loop, UART tx/rx, actuation
    helpers.cpp
  platformio.ini        board teensy41, library pins
racecar-pit-pi/
  PiUART.py             standalone Pi-side link reference (no ROS)
```

## Wire protocol

UART over the Pi GPIO14/15 header to the Teensy `Serial1`, `921600` baud, little-endian packed structs (`packets.h`). CRC-16/CCITT-FALSE covers every byte before the trailing 2-byte checksum, matching the driver's `pit_protocol.crc16_ccitt` (check value `0x29B1`).

| Direction | Struct | Magic | Rate |
|---|---|---|---|
| Teensy to Pi (telemetry) | `TXPacket` | `0xDEADBEEF` | `tx_freq` 200 Hz |
| Pi to Teensy (command) | `RxPacket` | `0xBEEFDEAD` | `rx_freq` 60 Hz |

Telemetry carries a microsecond timestamp, INA226 bus voltage and current, the FlySky RC channels, the LSM9DS1 IMU (9 axes), the encoder, and an EKF slot. Commands carry the normalized servo angle and motor speed, the dot-matrix and LED buffers, and a system-state byte. `verify_rx_crc` (default true) drops corrupt command frames.

## Drive control

Normalized `[-1, 1]` passthrough: the Pi sends normalized command, the Teensy maps to PWM open-loop. Actuation follows an RC-priority, Pi-fallback model with no arm switch:

- The RC transmitter overrides the Pi whenever a stick's mapped PWM leaves its neutral band (`throt_override_band` / `steer_override_band`, 40 us), gated on a valid receiver signal so a dropped or absent receiver never engages RC.
- With the sticks centered or no receiver, a fresh Pi command drives via passthrough, otherwise neutral. The Teensy owns the physical PWM limits: a normalized `[-1, 1]` command maps piecewise across `speed_pwm_min`/`speed_pwm_neutral`/`speed_pwm_max` (and `angle_pwm_*` for steering), so the Pi sends the full range and tightens the bounds here to protect the drivetrain.
- A command stale past `rx_timeout` (50 ms) falls to neutral. A dropped or absent receiver holds neutral instead of extrapolating to full reverse.

The ESC ignores an instant forward step from neutral, so the Pi throttle slews toward a forward target (`throt_slew_us`); steering and the neutral failsafe stay instant, and the RC path is human-ramped. A slow slew *into* reverse reads as brake on this forward/brake/reverse ESC and never latches, so a Pi reverse command snaps straight to the reverse endpoint (`rev_snap`); `speed_pwm_min` sits deeper than a symmetric forward throw to clear the reverse deadband.

## Telemetry

`fillTxBuff` packs the LSM9DS1 (9 axes via `getEvent(accel, mag, gyro)`), INA226 bus voltage (mV) and current (mA), the eight FlySky RC channels (raw pulse widths, us), and a microsecond timestamp. Current is packed as mA rather than truncated amps so sub-amp resolution survives the `uint16`. `getRPM` zeroes the encoder after a hall-transition timeout (`stop_timeout_us`, 100 ms) instead of reporting a ghost speed from a stale interval.

## Build and flash

PlatformIO with the Teensy platform. From `racecar-pit-teensy/`:

```sh
~/.pio-venv/bin/pio run              # build
~/.pio-venv/bin/pio run -t upload    # flash over USB (Teensy on /dev/ttyACM0)
```

`/dev/ttyACM0` is the USB debug/flash port only. The drive link is the GPIO UART (Teensy `Serial1` to the Pi's `/dev/neo-pit-pcb`, a udev symlink to `ttyAMA0` on Pi 5 / Ubuntu). The Pi needs `enable_uart=1` and a reboot to activate GPIO14/15.

## Verification status

Bench (wheels up, driving the Teensy directly over the GPIO UART): telemetry CRC 537/537 valid, accelerometer reads gravity, gyro and mag return real values, bus voltage 7.9 V, encoder and timestamp populate. Steering actuates, and a Pi throttle command ramps the motor to about 35k rpm with the encoder confirming rotation.

On-robot end-to-end (2026-07-06, wheels up): with the full ROS stack up, the LSM9DS1 streams to `/imu/lsm9ds1` at 151 Hz (accel z about -9.66) with no CRC or resync errors, and an Xbox controller drives the car through the mux to the ESC and servo at the configured limits. Forward and reverse both actuate (reverse via `rev_snap`); drive and steering polarity are corrected on the Pi side (`pit.yaml` `speed_sign`/`steering_sign` = -1), and the dot-matrix glyph and LED strip respond to controller input.

## License

See [LICENSE](./LICENSE).
