# Changelog

All notable changes to this project. Format: Keep a Changelog
(keepachangelog.com). Versioning: Semantic Versioning (semver.org).

## [Unreleased]

## [0.9.0] - Unreleased

Pairs with `racecar_neo_ros2_driver` v0.9.x. Telemetry and command sizes are
unchanged; the info reply is additive and only sent on request, so earlier
drivers run as before. The minor jumps from 0.7 to 0.9 to re-align with the
driver; there is no 0.8.0 firmware release.

### Added
- Firmware info reply. The Pi sets `SystemState` bit `0x20`; the Teensy sends
  the firmware version, commit, compiled configuration, and sensor status as
  ASCII `key=value` lines in CRC-checked chunks (magic `0xFEEDC0DE`), one chunk
  after each telemetry frame. Format and keys in `docs/protocol.md`.
- `src/version.h` and `scripts/build_flags.py`: the build embeds the
  version, the short commit, and a dirty flag.
- `scripts/pit_uart.py` (was `racecar-pit-pi/PiUART.py`), a bench tool for the
  link, with `--info` and `--yaml`, chunk reassembly and typed
  parsing (`InfoAssembler`, `parse_info`, `info_to_yaml`).
- Host tests: `tests/test_info` (14 cases) and five RC stick map
  cases in `test_logic`; info decoding and reassembly in the pytest suite
  (29 cases). Both suites check the same reference info packet byte for byte.
- Profiling build `teensy41_profile` (`src/profile.*`, `src/prof_stats.*`):
  thread periods, section timings, command-to-PWM latency, and interrupt
  lateness on USB serial; `scripts/profile_capture.py` records and checks
  them, and `scripts/telemetry_probe.py` measures telemetry at the Pi. Info
  key `fw.profile`.
- `docs/protocol.md`, `pyproject.toml` (pytest, ruff, black, mypy settings),
  `data/`.
- `cfg::I2C::clock_hz`, 400 kHz (the bus ran at 100 kHz before),
  `cfg::INA226_SENSOR::poll_ms` (1 ms, the previous rate),
  `cfg::UART::tx_buffer_bytes`, `cfg::UART::rx_read_timeout_ms`,
  `cfg::SYS_STATE::info_request`.

### Changed
- Repository layout: Teensy firmware only. `platformio.ini` at the root,
  firmware in `src/`, tunables in `config/config.h`, host tests in `tests/`,
  and the Pi bench tool in `scripts/`. The Pi side of the link lives in
  `racecar_neo_ros2_driver`.
- `helpers.cpp` split by subsystem into `shared`, `link`, `drive`, `sensors`,
  and `displays`. `testPWM` is now `DriveControl`; `ledStuffs` is now
  `updateLedStrip`.
- `config.h` is Arduino-free. IMU ranges are plain numbers mapped onto the
  Adafruit enums in `sensors.cpp`; wire constants (magic numbers, channel
  count, buffer sizes) live in `packets.h`. The host tests read the shipped
  values instead of mirrored copies.
- The RC-manual PWM endpoints are compile-time constants
  (`rc_pwm_min`/`rc_pwm_max`) instead of globals set in `setup()`.
- The RC stick-to-PWM and stick-to-command maps moved into `drive_map`
  (`rcThrottleToPwm`, `rcThrottleToNorm`, `mapRange` with Teensyduino's
  integer rounding) and are host tested.
- `UARTTx` assembles telemetry at send time and keeps a running deadline in
  place of restarting its timer after each write; `Serial1` has a 512-byte
  transmit buffer so a slot never blocks. Measured on the car: 141.6 -> 151.4
  Hz; the thread still wakes late, so the rate stays under 200 Hz.
- `UARTRx` checks the byte count of each command read and times out after
  20 ms instead of the 1 s stream default.
- The platform and every library are pinned to exact versions.
- `scripts/pit_uart.py` defaults to `/dev/neo-pit-pcb`.
- The charge bar curve is a 15-point measured discharge of the 5000 mAh 2S
  battery under the car's idle load (was a 10-point rest-voltage
  approximation that read 13 to 20 points low mid-run).
- Speed band set to the indoor build the cars run: `min_speed_mps` 0.75 ->
  0.5 m/s, `max_speed_mps` 5.0 -> 1.0 m/s. The field build raises the ceiling
  to 4.0 m/s. Slow mode stays at 1.0 m/s.

- The drive loop runs in a 1 kHz IntervalTimer interrupt instead of a
  thread, so it keeps its period whatever the threads do. The RC channels and
  the Pi drive command it reads are shared through sequence counters
  (`src/seqlock.h`) in place of `RxMutex` and the `CmdMutex` setpoints.
- The LSM9DS1 is read every 4 ms (250 Hz) and the INA226 every 10 ms; each
  read blocks about 1.1 ms and 0.28 ms on the bus. New info key
  `imu.read_hz`.
- `UARTRx` assembles command frames from the bytes that have arrived
  (`src/command_framer.*`) instead of waiting in `readBytes` for a whole
  frame.
- The `serial_plot` trace prints from `loop()`.
- swA slow mode is now the low pulse (lever up on the FS-i6, its start
  position); high is fast. A receiver with no signal reads fast. swD is
  unchanged: high (lever down) is manual override, low or no signal leaves the
  Pi in control.

### Fixed
- `cfg::I2C::clock_hz` never took effect: Adafruit BusIO's `begin()` calls
  `Wire.begin()`, which resets the bus to 100 kHz. The clock is applied again
  after the LSM9DS1 starts.
- The speed estimate was updated by two threads (telemetry and actuation),
  each pushing samples into the same filter. Only the actuation thread updates
  it now; telemetry reads its result.
- A sensor missing at boot held the I2C bus in its `begin()` retry loop, so the
  other sensor never started. Retries now release the bus between attempts.
- Two sensor threads shared one `errorFlag` and cleared each other's error.
  Each sensor now has its own presence flag, reported in the info reply.

### Removed
- The `fillTxBuff` thread, the receive ping/pong copy, and `ReceiveMutex`,
  `RecPongMutex`, `TransmitMutex`, `SPIMutex`, and `ErrMutex`, each guarding
  state touched by a single thread.
- The unused INA226 power read (`getPower`) on every 1 ms cycle.
- The unused `BasicLinearAlgebra` dependency.
- `initRxHeader`, whose fields every received frame overwrote.
- PlatformIO template READMEs in `include/`, `lib/`, and `test/`.

## [0.7.1] - 2026-09-06

Pairs with `racecar_neo_ros2_driver` v0.7.x. Wire format unchanged.

### Changed
- Slow mode (swA up) is an exact 1.0 m/s setpoint on both the Pi and the RC
  path. It rescales the full stick travel onto `slow_speed_mps` rather than
  clipping the top of the fast-mode range, so stick resolution stays usable and
  full throttle lands on the cap exactly.
- RC slow mode closes the loop on the hall encoder, running the stick through
  the same PI controller as the Pi path. A speed cap in m/s cannot be held open
  loop. Fast-mode RC keeps the direct PWM path, so a human override there is
  still instant.
- `throttleToTargetMps` moves into `drive_map.{h,cpp}` with the deadband and
  both speed endpoints as parameters, and is host tested.

### Removed
- `cfg::ESC_10BL60_G2::slow_rc_scale`, the open-loop PWM trim that stood in for
  a speed cap on the RC path. It was a provisional 0.4 that was never calibrated
  and could not hold a speed under load.

### Notes
- The Teensy accepts, CRC-checks, and stores Pi command frames regardless of
  either switch; manual override is applied at actuation only. The Pi is never
  blocked by the switch state and drive resumes on the next frame when override
  drops, so acting on the switches driver-side is optional.

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
- Host unit tests (`pio test -e native`) over the CRC, the normalized-to-PWM
  mapping, the hall direction decode, the RC switch decode, and the SoC curve.
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
