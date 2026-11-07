# Wire protocol

UART between the Pi (`/dev/neo-pit-pcb`, GPIO14/15) and the Teensy `Serial1`,
921600 baud 8N1. Structs are packed little-endian; `src/packets.h` is
the authority and pins every size with `static_assert`. Each packet ends with
a CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, check value
`0x29B1`) over every preceding byte, magic included.

## Contents

- [Packet types](#packet-types)
- [Telemetry](#telemetry)
- [Command](#command)
- [System state bits](#system-state-bits)
- [Info reply](#info-reply)
- [Info keys](#info-keys)
- [Compatibility](#compatibility)

## Packet types

| Direction | Magic | Size | When |
| --------- | ----- | ---- | ---- |
| Teensy to Pi, telemetry | `0xDEADBEEF` | 94 B | every slot, 200 Hz |
| Pi to Teensy, command | `0xBEEFDEAD` | 4 B magic + 470 B | 60 Hz, driver side |
| Teensy to Pi, info chunk | `0xFEEDC0DE` | 18 to 146 B | one per slot, only while requested |

Telemetry and info both start with `HeaderPacket` (16 B):

| Field | Type | Telemetry | Info chunk |
| ----- | ---- | --------- | ---------- |
| `MagicNum` | u32 | `0xDEADBEEF` | `0xFEEDC0DE` |
| `VersionNum` | u8 | 1 | 1 |
| `IDNumber` | u8 | 0 | payload revision |
| `PacketBytes` | u16 | 94 | bytes in this packet, magic and CRC included |
| `FrameBytes` | u32 | 0 | payload bytes across all chunks |
| `PacketIndex` | u16 | 0 | chunk index |
| `PacketsPerFrame` | u16 | 1 | chunk count |

## Telemetry

| Field | Type | Unit |
| ----- | ---- | ---- |
| `Timestamp` | i32 | `micros()` at send; wraps every ~35.8 min |
| `VoltAndCurr` | u16[2] | bus mV, current mA (negative clamps to 0) |
| `RxReceiver` | u16[8] | raw FlySky pulse widths, us; 0 when absent |
| `IMU` | f32[9] | accel xyz m/s^2, gyro xyz rad/s, mag xyz uT (LSM9DS1 chip axes) |
| `Encoder` | f32 | signed ground speed, m/s |
| `EKF` | f32[3] | stall guard: [0] state (0 normal, 1 tripped, 2 cooldown, 3 lockout), [1] seconds left in the cooldown or lockout, else 0, [2] trips since boot; all 0 in builds without `fw.stall_protect` |
| `Checksum` | u16 | CRC-16 |

## Command

The Teensy matches the magic with a sliding window, then reads the 470-byte
`RxPacket`: `HeaderPacketNoMagic` (12 B), `ServoAng` f32 and `MotorSpeed` f32
(normalized [-1, 1]), `DotMatrix` 192 B (8x24, row-major), `LED` 255 B (RGB
triplets, 84 used), `SystemState` u8, `Checksum` u16. A frame failing the CRC
is dropped (`cfg::UART::verify_rx_crc`).

## System state bits

`RxPacket.SystemState`, mirrored in `cfg::SYS_STATE`.

| Bits | Name | Meaning |
| ---- | ---- | ------- |
| `0x03` | mode | 0 idle, 1 manual, 2 auto; selects the dot-matrix glyph |
| `0x04` | `dotmatrix_active` | the Pi bitmap replaces the glyph |
| `0x08` | `led_active` | the Pi pixels replace the charge bar |
| `0x10` | `driver_starting` | play the LED loading sweep |
| `0x20` | `info_request` | send info chunks until the bit clears |
| `0xC0` | none | reserved, send 0 |

## Info reply

The info reply reports the firmware version and the compiled configuration.
The Pi requests it once per driver start.

Request and reply:

1. The Pi sets `info_request` in every command frame.
2. While the bit is set and commands are fresh (within 1 s), the Teensy sends
   one info chunk after each telemetry frame, cycling through chunk indices
   0 to `PacketsPerFrame - 1` and starting over.
3. The Pi keeps CRC-valid chunks of one revision until it holds every index,
   then joins them in index order. The joined length must equal
   `FrameBytes`.
4. The Pi clears the bit. Telemetry continues unchanged.

The Teensy rebuilds the payload at the start of each cycle. `IDNumber`
changes only when the payload content does (a sensor found after boot), so
the Pi restarts collection when the revision, chunk count, or `FrameBytes`
changes, and never joins chunks of two payloads. A lost chunk comes round
again in the next cycle; a full cycle of the current payload (about 2.0 kB,
16 chunks) takes about 80 ms. The firmware holds up to 3072 B
(`info::payload_max_bytes`).

One telemetry frame plus the largest chunk is 240 B, 2.6 ms on the wire,
inside the 5 ms slot. Firmware without the info reply never answers; the
Pi falls back to configured values after a timeout (2 s in
`scripts/pit_uart.py`).

Payload format: ASCII, one `key=value` line per setting, each ending in
`\n`. Keys are dotted and unique. Values are integers, fixed-point decimals
(trailing zeros kept), or text without `=` or line breaks. Parse
`fw.version`, `fw.label`, and `fw.git` as text even when they look numeric. Readers must
ignore unknown keys: new keys may be added without changing `info`; a change
to a key's meaning or unit increments `info`.

Reference packet, used by both test suites: payload `info=1\n`, revision 3,
chunk 0 of 1:

```
de c0 ed fe 01 03 19 00 07 00 00 00 00 00 01 00 69 6e 66 6f 3d 31 0a d4 60
```

## Info keys

Schema `info=1`. Source is `writeFirmwareInfo` in `src/info.cpp`;
values come from `config/config.h` unless noted.

| Key | Example | Meaning |
| --- | ------- | ------- |
| `info` | 1 | payload schema version |
| `fw.version` | 0.9.0 | firmware version, `src/version.h` |
| `fw.label` | v0.9.0.3-pcb | latest labeled commit in the build's history; `dev` when none |
| `fw.git` | e330426 | short commit of the build; `unknown` outside git |
| `fw.git_dirty` | 0 | 1 when built with uncommitted tracked changes |
| `fw.board` | teensy41 | target board |
| `fw.profile` | 0 | 1 in the profiling build (`teensy41_profile`) |
| `fw.protocol` | 1 | header `VersionNum` |
| `fw.speed_control` | closed_loop | Pi throttle is a speed setpoint on the encoder; `esc_cal` and `raw_pwm` in the test builds |
| `link.baud` | 921600 | UART baud |
| `link.telemetry_hz` | 200 | telemetry slot rate |
| `link.cmd_timeout_ms` | 50 | stale command falls to neutral |
| `link.cmd_crc` | 1 | command CRC enforced |
| `steer.pwm_min_us`, `steer.pwm_center_us`, `steer.pwm_max_us` | 1025, 1500, 1975 | servo PWM at -1, 0, +1 (-1 = full left) |
| `esc.pwm_min_us`, `esc.pwm_neutral_us`, `esc.pwm_max_us` | 1000, 1500, 2000 | ESC PWM bounds, equal to the ESC calibration endpoints |
| `esc.rc_pwm_min_us`, `esc.rc_pwm_max_us` | 1375, 1625 | RC fast-mode PWM endpoints |
| `speed.min_mps` | 0.500 | setpoint just past the deadband |
| `speed.max_mps` | 1.000 | setpoint at full command |
| `speed.slow_mps` | 1.000 | setpoint at full command with swA up |
| `speed.deadband` | 0.010 | normalized commands inside this are stop |
| `pid.kp`, `pid.ki`, `pid.kd` | 20.000, 60.000, 0.000 | speed loop gains, us per m/s |
| `pid.ki_rev` | 60.000 | integral gain while the target is reverse |
| `pid.max_integral` | 150.000 | integrator clamp |
| `pid.kp_boost`, `pid.boost_error_mps` | 60.000, 0.250 | gain above this error |
| `pid.ff_fwd_us`, `pid.ff_rev_us` | 69.0, 94.0 | static feedforward across the ESC deadband |
| `pid.kv_ff_fwd`, `pid.kv_ff_rev` | 12.500, 55.000 | feedforward per m/s |
| `pid.start_confirm_intervals`, `pid.start_confirm_ms` | 5, 50 | hall intervals and time since standstill that confirm motion |
| `pid.start_fwd_from_us`, `pid.start_fwd_max_us` | 0.0, 0.0 | forward start assist ramp (0 is off) |
| `pid.start_rev_from_us`, `pid.start_rev_max_us`, `pid.start_rev_ramp_us_per_s` | 160.0, 320.0, 2000.0 | reverse start assist ramp |
| `pid.start_rev_timeout_ms`, `pid.start_rev_rest_ms` | 600, 300 | assist time before a neutral rest, rest time |
| `pid.start_rev_handover_mps`, `pid.start_rev_handover_frac` | 0.500, 0.800 | the assist ends at the lower of this speed and this fraction of the target |
| `drivetrain.wheel_diameter_m` | 0.0720 | wheel diameter |
| `drivetrain.gear_ratio` | 0.119628 | wheel rev per motor rev |
| `encoder.edges_per_motor_rev` | 12 | hall transitions per motor rev |
| `encoder.m_per_edge` | 0.002255 | ground distance per hall transition |
| `encoder.rpm_window`, `encoder.filter_alpha` | 5, 0.800 | hall intervals in the speed median, filter weight |
| `encoder.stop_timeout_us` | 30000 | no edge for this long reads 0 m/s |
| `rc.ch_steer`, `rc.ch_throttle`, `rc.ch_speed_mode`, `rc.ch_manual_override` | 3, 1, 4, 7 | zero-based telemetry RC indices |
| `rc.switch_threshold_us` | 1500 | switch up at or above |
| `rc.valid_min_us`, `rc.valid_max_us` | 900, 2100 | plausible pulse band |
| `rc.override_throttle_us`, `rc.override_steer_us` | 40, 40 | stick travel that takes over from the Pi |
| `imu.model` | LSM9DS1 | IMU chip |
| `imu.accel_range_g` | 8 | accel full scale |
| `imu.gyro_odr_hz`, `imu.accel_odr_hz` | 952, 952 | data rates read back from the chip; the accelerometer follows the gyro while it runs; 0 until the IMU answers |
| `imu.mag_odr_hz`, `imu.mag_range_read_gauss` | 155.000, 8 | magnetometer rate and full scale read back from the chip |
| `imu.reg_ctrl1_g`, `imu.reg_ctrl6_xl`, `imu.reg_ctrl1_m`, `imu.reg_ctrl2_m`, `imu.reg_ctrl3_m` | 200, 120, 98, 32, 0 | the raw registers behind those rates |
| `imu.gyro_range_dps`, `imu.mag_range_gauss` | 500, 8 | gyro and mag full scale |
| `imu.read_hz` | 250 | LSM9DS1 read rate |
| `imu.accel_unit`, `imu.gyro_unit`, `imu.mag_unit` | m/s2, rad/s, uT | telemetry `IMU` units |
| `cal.source` | builtin | where the per-car `pid.*` values in use came from: `eeprom`, `builtin`, or either with `_modified` after an unsaved `cal set` |
| `cal.format`, `cal.eeprom` | 1, empty | EEPROM block format, and its state: `ok`, `empty`, `bad_magic`, `bad_format`, `bad_crc`, `out_of_range` |
| `cal.crc`, `cal.written` | 3f2a, 2026-11-06 | stored block CRC and save date; `none` without a stored block |
| `fw.stall_protect` | 1 | stall guard present; absent in builds without it |
| `stall.trip_a`, `stall.moving_mps`, `stall.trip_ms` | 10.0, 0.10, 500 | a stall is throttle commanded, speed under `moving_mps` and current over `trip_a`; it trips after `trip_ms` of it |
| `stall.cooldown_ms` | 2000 | neutral after the command returns to neutral |
| `stall.lockout_trips`, `stall.lockout_window_ms`, `stall.lockout_ms` | 3, 60000, 30000 | this many trips within the window give a lockout in place of the cooldown |
| `power.sensor` | INA226 | power sensor |
| `power.shunt_ohm`, `power.max_current_a` | 0.0020, 40.0 | shunt and current range |
| `power.avg_samples` | 16 | INA226 hardware averaging |
| `power.poll_ms` | 10 | INA226 read period |
| `power.dip_mv`, `power.dip_min_ms` | 7200, 100 | a dip is the bus voltage below `dip_mv` for `dip_min_ms`; the next counts after 7300 mV |
| `battery.cells`, `battery.empty_mv`, `battery.full_mv` | 2, 7000, 8400 | charge bar endpoints |
| `i2c.clock_hz` | 400000 | I2C bus clock |
| `display.leds`, `display.dot_rows`, `display.dot_cols` | 84, 8, 24 | display sizes |
| `watchdog.timeout_ms` | 100 | watchdog reset after this long without a refresh |
| `status.imu`, `status.power` | ok | `missing` until the sensor answers |
| `status.watchdog` | armed | `starting` until every context has run once after boot; `off` when disabled |
| `status.last_reset` | watchdog | cause of the last boot: `watchdog`, `software`, `reset`, or `unknown` |
| `status.reset_flags` | 128 | `SRC_SRSR` at boot (bit 7 watchdog 3, bit 1 software or lockup, bit 0 reset pin) |
| `power.min_mv` | 7412 | lowest bus voltage since boot, at the time of the request; -1 before the first reading |
| `power.dip_count` | 0 | dips since boot, at the time of the request |
| `power.last_dip_ms` | 812345 | Teensy uptime at the start of the last dip; -1 for none |

The per-car keys (`pid.ff_fwd_us`, `pid.kv_ff_fwd`, `pid.ff_rev_us`,
`pid.kv_ff_rev`, `pid.kp`, `pid.kp_boost`, `pid.boost_error_mps`, `pid.ki`,
`pid.ki_rev`, `pid.max_integral`, `pid.start_rev_from_us`,
`pid.start_rev_max_us`) report the values in use. The same names are the keys
of the USB `cal set` command (README "Speed constants").

The `power.min_mv`, `power.dip_count` and `power.last_dip_ms` values are taken
when the reply is built. A request made right after a reset finds no voltage
reading yet; request again for current values.

## Compatibility

Telemetry and command sizes are unchanged since 0.5.0. A driver that does not
know info frames never sets `info_request`, so the Teensy never sends one. If
stray bytes did arrive, the payload is ASCII and cannot contain the telemetry
magic bytes, so the old scanner skips them. Firmware older than 0.9.0
ignores bit `0x20` and never answers.
