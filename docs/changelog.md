# Changelog

All notable changes to this project. Format: Keep a Changelog
(keepachangelog.com). Versioning: Semantic Versioning (semver.org).

## [Unreleased]

### Added
- Host unit tests (`pio test -e native`, 10 cases) over the CRC, the
  normalized-to-PWM mapping, and the hall direction decode.
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
