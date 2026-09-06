# Changelog

All notable changes to this project. Format: Keep a Changelog
(keepachangelog.com). Versioning: Semantic Versioning (semver.org).

## [Unreleased]

## [0.7.0] - 2026-09-06

Pairs with `racecar_neo_ros2_driver` v0.7.x. The wire format is unchanged from
0.5.0; the minor jumps from 0.5 to 0.7 to re-align with the driver, so there is
no 0.6.0 firmware release. See the versioning rules in the README.

### Added
- Two RC switches, sharing one contract with the driver and needing no wire
  change since both ride in the existing telemetry RC array. swA on CH5 selects
  fast (`max_speed_mps`, 5.0 m/s) or slow (`slow_speed_mps`, 1.0 m/s); swD on
  CH8 selects Pi autonomy or manual override, where the Teensy drives alone and
  centered sticks hold neutral instead of handing back to the Pi. Both default
  to the permissive position when the receiver is absent. Under override the dot
  matrix shows `M` with live sticks and `I` otherwise, ignoring the Pi's bitmap
  and mode bits; `A` is meaningless while autonomy is locked out.
- `rcPulseValid` / `rcSwitchUp` in `drive_map.{h,cpp}`, host tested, so the
  fail-safe switch default is covered rather than asserted.
- `battery.{h,cpp}`: 2S LiPo state of charge from pack voltage, piecewise-linear
  over a ten-point discharge curve (3.50 to 4.20 V/cell), Arduino-free and host
  tested.
- Host unit tests (`pio test -e native`, 17 cases) over the CRC, the
  normalized-to-PWM mapping, the hall direction decode, the RC switch decode,
  and the SoC curve.
- Pytest suite for the Pi-side reference (`racecar-pit-pi/tests/`, 13 cases),
  including schema sizes checked against the firmware packet sizes.
- `static_assert` on every wire struct size in `packets.h` and on the PWM,
  RC-band, speed-band, and channel-index invariants in `config.h`.
- `docs/architecture.md`, `docs/changelog.md`, `docs/test-log.md`,
  `.gitattributes`.
- `cfg::DEBUG::serial_plot` to gate the control-loop serial trace, off by
  default.
- `--port` and `--no-verify-crc` arguments for `PiUART.py`.

### Changed
- The idle LED bar spans 7.0 to 8.4 V (was 7.5 to 8.4 V) and fills by charge
  rather than by voltage. A 2S pack holds most of its usable charge across the
  flat 7.6-8.0 V plateau, so a linear voltage bar read near full for most of a
  run and then emptied in seconds. `cfg::DISP::volt_bar_min_cv` /
  `volt_bar_max_cv` are replaced by `batt_empty_mv` / `batt_full_mv` in
  `battery.h`.
- RC drive moves onto the two sprung stick axes: throttle from CH2 (right stick
  vertical, `ch_throt` 2 -> 1) and steering from CH4 (left stick horizontal,
  `ch_steer` 0 -> 3). CH3 is the transmitter's ratcheted throttle axis and rests
  at its low endpoint, which the RC override read as a held reverse command,
  locking out the Pi whenever the transmitter was on. Stick polarity is set on
  the transmitter (Functions > Reverse). Telemetry still carries all eight
  channels unchanged.
- `cfg::FLYSKY_IA6B_RX` documents the real channel map: AETR order, index N-1
  for PPM channel N, aux on CH5/CH6. The old comment claimed SWA/SWB/SWC/SWD on
  indices 4-7, carried over from an i-BUS listing that ran past the array. The
  iA6B has six servo rails, but its PPM stream carries the whole frame, so all
  eight slots populate and swA-swD land on CH5-CH8.
- Long explanatory comments trimmed across `config.h`, `helpers.cpp`,
  `packets.h`, `crc16.h`, `drive_map.h`, `helpers.h`, `main.cpp`, and the host
  tests; comments that only restate the code are gone.
- Split the Arduino-free logic out of `helpers.cpp` into `crc16.{h,cpp}` and
  `drive_map.{h,cpp}` so it can be tested on the host.
- `config.h` is now authoritative: the INA226 averaging and shunt settings, the
  LSM9DS1 ranges, the telemetry rate, the boot delay, the thread periods, the
  RC channel indices and deadzone, and the LED voltage-bar endpoints all read
  from it instead of being hardcoded in `helpers.cpp`.
- `PiUART.py` restructured into importable functions with a `main()` guard;
  opening the serial port no longer happens at import.
- README rewritten to match the firmware: the Pi throttle path is closed-loop,
  not the open-loop passthrough previously described.
- Renamed `percentThrot`/`percentRev` to `percent_throt`/`percent_rev`,
  `RecieveMutex` to `ReceiveMutex`, and `getDirection` to `hallDirection`.

### Fixed
- `initRxHeader` assigned `IDNumber` twice and never set `FrameBytes`; every
  header field is now written explicitly.
- `PiUART.py` declared the telemetry `Encoder` field as `int16` where the
  firmware sends a `float`, so the schema was 2 bytes short and every field
  after it decoded from the wrong offset.
- `PiUART.py` sent command frames with a zero checksum, which the firmware drops
  whenever `verify_rx_crc` is set; it now computes CRC-16/CCITT-FALSE over the
  frame.

### Removed
- The `speed_demon` reverse-select branch in `testPWM`, which read CH5 to pick a
  throttle direction. It mapped the whole pulse range onto neutral-to-max, so
  after the move to the sprung CH2 a centered stick sat at roughly half
  throttle. `speed_demon` now only widens the RC endpoints in `main.cpp` and the
  normal piecewise path handles both directions, which frees CH5 for swA.
- `src/claude.txt`, a superseded copy of the firmware describing a 230400-baud
  link and an RC arm switch that no longer exist.
- Dead `InnerControlLoop` thread, unused `max_pwm_left`/`max_pwm_right`,
  `numleds`, `colorIdx`, `currrentPalette`, `errType`, `drive_mode_sw`, and the
  unused `HallMutex`, `TransPongMutex`, and `SerMutex`.
- Commented-out debug blocks in the hall ISR, the receive loop, and the speed
  path.

## [0.1.0] - 2026-07-24

### Added
- Teensy 4.1 firmware for the NEO-PIT drive PCB: servo and ESC actuation,
  FlySky iA6B RC override, LSM9DS1 IMU, INA226 power sensing, hall encoder,
  MAX7219 dot matrix, WS2812B strip.
- Binary UART protocol to the Pi at 921600 baud with CRC-16/CCITT-FALSE on both
  directions.
- Closed-loop wheel-speed control on the Pi throttle command.
- Pi-side UART reference script.
