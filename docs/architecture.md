# Architecture

## Overview

Two ends of one serial link. A Teensy 4.1 on the NEO-PIT drive PCB owns every
actuator and sensor on the RACECAR Neo V2; a Raspberry Pi 5 sends normalized
drive commands and consumes telemetry. The drive loop runs in a 1 kHz timer
interrupt; six threads under TeensyThreads and the Arduino `loop()` do the
I/O. There is no RTOS and no dynamic allocation after boot.

```
                    FlySky iA6B (PPM)
                            |
                            v
  Pi 5 ---- UART 921600 --> Teensy 4.1 ---- PWM ---> servo (D135F)
       <---  Serial1    ---     |        ---- PWM ---> ESC (10BL60-G2)
                                |        ---- SPI ---> MAX7219 dot matrix
                                |        ---- DI ----> WS2812B strip
                                |
                    I2C: INA226, LSM9DS1
                    GPIO IRQ: 3x hall
```

## Components

| Module (path) | Responsibility | Depends on |
| ------------- | -------------- | ---------- |
| `config/config.h` | Every tunable; compile-time invariant checks; Arduino-free | `packets.h` |
| `src/packets.{h,cpp}` | Wire format, size static_asserts, info chunk builder | `crc16` |
| `src/info.{h,cpp}` | Info payload from the compiled config; Arduino-free | `config.h`, `battery.h`, `version.h` |
| `src/version.h` | Firmware version; commit macros set by `scripts/build_flags.py` | none |
| `src/crc16.{h,cpp}` | CRC-16/CCITT-FALSE; Arduino-free | none |
| `src/drive_map.{h,cpp}` | Normalized-to-PWM map, RC stick map, hall decode, switch decode, speed setpoint; Arduino-free | none |
| `src/battery.{h,cpp}` | 2S LiPo voltage-to-charge curve; Arduino-free | none |
| `src/shared.{h,cpp}` | State shared between threads and its mutexes | TeensyThreads |
| `src/link.{h,cpp}` | `UARTTx`, `UARTRx`: telemetry, info chunks, commands | `packets`, `info`, `shared` |
| `src/drive.{h,cpp}` | Drive tick (1 kHz interrupt), speed PID, hall ISR and speed estimate | `drive_map`, `shared`, Servo, IntervalTimer |
| `src/seqlock.h` | Single-writer value readable from an interrupt; Arduino-free | none |
| `src/command_framer.{h,cpp}` | Command frame assembly from the UART byte stream; Arduino-free | `packets`, `crc16` |
| `src/sensors.{h,cpp}` | `ReadCurrAndVolt`, `ReadIMUStates`, `RxReceive` | INA226, Adafruit LSM9DS1, PPM-reader |
| `src/displays.{h,cpp}` | `DotMatrix` thread, `updateLedStrip` | MD_Parola, FastLED, `battery` |
| `src/main.cpp` | Boot: peripherals, interrupts, thread launch | all threads |
| `src/profile.{h,cpp}`, `src/prof_stats.{h,cpp}` | Profiling hooks and statistics; active only with `RCN_PROFILE` | cycle counter, IntervalTimer |
| `scripts/pit_uart.py` | Bench tool: drives the link from the Pi without ROS | numpy, pyserial |

## Threads

The drive tick is an IntervalTimer interrupt started in `setup()`. The threads
start after it; each loops forever and yields with `threads.delay()` or
`threads.yield()`. TeensyThreads runs them round robin with 1 ms slices and
no priorities, so a thread waits for every other thread's turn; the drive
loop does not.

| Context | Period | Reads | Writes |
| ------- | ------ | ----- | ------ |
| Drive tick (interrupt) | 1 ms | `rc_channels`, `drive_cmd`, hall ISR state | servo + ESC PWM, `measured_speed_mps`, `manual_override`, `rc_driving` |
| `RxReceive` | 1 ms | PPM pin | `rc_channels` |
| `ReadCurrAndVolt` | 10 ms | INA226 (I2C) | `power_amps`, `power_volts`, `power_ok` |
| `ReadIMUStates` | 4 ms | LSM9DS1 (I2C) | `imuVals`, `imu_ok` |
| `UARTTx` | 5 ms slot | `rc_channels`, `imuVals`, power, `measured_speed_mps`, `system_state` | `Serial1` |
| `UARTRx` | 1 ms | `Serial1` | `drive_cmd`, display buffers, `system_state`, `lastRxTime` |
| `DotMatrix` | 50 ms | `dotmatrix_cmd`, `system_state`, `manual_override` | MAX7219 (SPI) |
| `loop()` | 25 ms | `led_cmd`, `system_state`, power | WS2812B strip, serial trace |

The drive tick runs at priority 144, below the Servo, PPM, hall and serial
interrupts (128), so pulse edges and RC pulse timing never wait on it.

Each shared block has one writer. What the drive tick reads goes through a
`Seqlock` (`src/seqlock.h`): a sequence counter that is odd during a write.
An interrupt cannot wait on a mutex; when it lands mid-write it keeps the
previous value. Threads reading a `Seqlock` retry with `readShared()`. Other
blocks wider than one word take their own mutex: `CmdMutex` (display
buffers, `system_state`), `ImuMutex`, `PowerMutex`, and `I2CMutex` for the
bus both sensors share. Single words (`lastRxTime`, the sensor and drive
flags, `measured_speed_mps`) are `volatile`. The hall ISR state is read under
`noInterrupts()`.

A sensor that does not answer `begin()` is retried every 100 ms with the bus
released, so the other sensor keeps running; its `status.*` info key reads
`missing` until then.

## Data pipeline

Telemetry, Teensy to Pi:

```
LSM9DS1 --I2C--> ReadIMUStates  -> imuVals[9] ------+
INA226  --I2C--> ReadCurrAndVolt -> power ----------+
iA6B    --PPM--> RxReceive -> rc_channels[8] -------+--> UARTTx (200 Hz, CRC-16) --> Serial1
hall    --IRQ--> drive tick -> measured_speed_mps --+        |
                                                             +--> info chunk while requested
```

Command, Pi to Teensy:

```
Serial1 -> UARTRx (CommandFramer, CRC-16 gate) -> drive_cmd -> drive tick -> speedPID -> ESC PWM
                                            -> dotmatrix_cmd / led_cmd / system_state -> DotMatrix, updateLedStrip
                                            -> system_state info_request -> UARTTx
```

The speed estimate averages the last five hall intervals, converts electrical
to mechanical RPM, then low-pass filters at `rpm_filter_alpha`. It applies the
two gear ratios and the wheel circumference to reach m/s at the ground, which
is what the encoder field carries and what the speed loop closes on. Only
the drive tick updates it.

`UARTTx` keeps a running deadline; after a stall longer than one slot it
restarts from the current time. The rate reaches `tx_freq` only while the
thread wakes within one slot; measured 151 Hz on the car (TL-009). `Serial1` gets a
512-byte transmit buffer, enough for one telemetry frame and one info chunk,
so a slot never blocks the thread.

## RC receiver mapping

The FS-iA6B feeds PPM into pin 5. `PPMReader` decodes it and `RxReceive` copies
channels 1-8 into `rc_channels` slots 0-7, so index N-1 is PPM channel N. The
transmitter emits AETR order.

| Index | PPM ch | Transmitter source (FS-i6 default) | Centering | Used by |
| ----- | ------ | ---------------------------------- | --------- | ------- |
| 0 | CH1 | Aileron, right stick horizontal | sprung | telemetry only |
| 1 | CH2 | Elevator, right stick vertical | sprung | `ch_throt`, ESC PWM |
| 2 | CH3 | Throttle, left stick vertical | ratcheted | telemetry only |
| 3 | CH4 | Rudder, left stick horizontal | sprung | `ch_steer`, steering PWM |
| 4 | CH5 | swA | switch | `ch_speed_mode`, fast/slow |
| 5 | CH6 | swB | switch | telemetry only |
| 6 | CH7 | swC | switch | telemetry only |
| 7 | CH8 | swD | switch | `ch_manual_override`, Pi lockout |

Drive reads only sprung axes, so both controls return to center on release. CH3
is the transmitter's own throttle axis and is ratcheted; the RC override in
drive tick treats any output past `throt_override_band` as a human at the
controls, so a stick resting at its low endpoint would hold the car in reverse
and lock out the Pi. Stick polarity is set on the transmitter, not in firmware.

`latestValidChannelValue(ch, 0)` returns 0 for a channel that is absent, out of
band, or stale past the library's 500 ms failsafe; otherwise it clamps into
1000-2000 us. So each channel is either 0 or a clamped pulse width, and
`rc_valid_min`/`rc_valid_max` (900/2100) reject only the 0 case.

All eight slots are republished raw over telemetry (`TXPacket.RxReceiver`), the
two unfilled ones included, and the driver normalizes all eight in
`pit_protocol.Telemetry.rc_normalized`. The array width is fixed by the wire
format, so dropping the dead slots would be a protocol break.

### Switch contract

`rcSwitchUp()` and `rcSwitchDown()` in `drive_map.h` decide a switch: high at
or above `sw_threshold` (1500 us), low for a valid pulse below it, and neither
for a pulse outside the valid band. An absent channel reads 0, so a receiver
with no signal leaves fast mode and the Pi in control. On the FS-i6 as set
up, lever up sends low.

| Switch | Ch | Index | Low, lever up (default) | High, lever down | Resolved in |
| ------ | -- | ----- | ----------------------- | ---------------- | ----------- |
| swA | CH5 | 4 | slow, `slow_speed_mps` | fast, `max_speed_mps` | drive tick |
| swD | CH8 | 7 | Pi allowed | Teensy-only drive | drive tick, `DotMatrix` |

Confirmed on hardware: CH5 is swA, CH6 is swB, and CH8 is swD and arrives
intact. The iA6B has six servo rails, but the PPM stream carries the whole
frame, so all eight slots populate.

The drive tick publishes both states to `DotMatrix` through the `manual_override` and
`rc_driving` volatiles. Under override the display shows `M` while the sticks
are live and `I` otherwise, and the Pi's bitmap and mode bits are ignored.

Slow mode rescales full stick travel onto `slow_speed_mps` on both paths, so the
cap is an exact setpoint rather than a clipped command. A speed cap cannot be
held open loop, so slow mode routes the RC stick through `speedPID` as well;
fast-mode RC keeps the direct PWM path and its instant override.

Command frames are received, CRC-checked, and stored regardless of either
switch; override is applied at actuation only. The Pi therefore never blocks on
the Teensy, and reading the switches out of `/rc/channels` is optional.

## Profiling build

`pio run -e teensy41_profile` builds the same firmware with `-DRCN_PROFILE`.
The hooks in `src/profile.h` compile to nothing in the shipped build. The
profiling build times thread loops and code sections with the ARM cycle
counter, runs a 10 kHz top-priority timer interrupt whose lateness bounds
interrupt-masked windows, and once per second prints one line per stat on
USB serial (`/dev/ttyACM0`):

```
P,<t_ms>,period,<name>,<n>,<sum_us>,<max_us>,<p50_us>,<p99_us>
P,<t_ms>,section,<name>,<n>,<sum_us>,<max_us>
P,<t_ms>,counter,<name>,<value>
```

Periods are the time between successive events of one context (50 us
histogram bins, quantiles as upper bin edges). Names are listed in
`src/profile.h`. Lines are dropped, and counted in `report_lines_dropped`,
when the USB transmit buffer is full, so `loop()` never waits on USB.
`scripts/profile_capture.py` records them to `data/` and checks them against
the timing targets.

## Interfaces

| Interface | Pins / device                     | Defined in                     |
| --------- | --------------------------------- | ------------------------------ |
| Pi UART   | `Serial1`, 921600 8N1; see `docs/protocol.md` | `cfg::UART`, `packets.h` |
| USB debug | `Serial`, 115200, `/dev/ttyACM0`  | `cfg::DEBUG`                   |
| Servo PWM | pin 2                             | `cfg::SERVO_D135F`             |
| ESC PWM   | pin 3                             | `cfg::ESC_10BL60_G2`           |
| PPM in    | pin 5                             | `cfg::FLYSKY_IA6B_RX`          |
| WS2812B   | pin 9                             | `cfg::RGB_LED_WS2812B`         |
| MAX7219   | SPI, CS pin 10                    | `cfg::MAX_7219_DOT_MATRIX`     |
| Hall      | pins 30, 31, 32 (IRQ on CHANGE)   | `cfg::ES2845S_4900KV_MOTOR`    |
| I2C       | INA226 at 0x40, LSM9DS1, 400 kHz  | `cfg::I2C`, `cfg::INA226_SENSOR`, `cfg::LSM9DS1_IMU` |

The wire format in `src/packets.h` is shared with `racecar_neo_ros2_driver`
(`pit_protocol.py`). Struct sizes are pinned by `static_assert`; changing one is
a protocol break that has to land on both sides together.

## Constraints

- Actuation runs at 1 kHz in an interrupt; the tick must stay short (a few
  microseconds) and must never wait on a mutex, `Serial`, or the I2C bus.
- `Serial1` gets a 2048-byte RX buffer. The stock 64 bytes fills in about 0.7 ms
  at 921600 baud, shorter than the scheduler can preempt `UARTRx` for, and a
  474-byte command frame is lost mid-flight when it overflows.
- swD high (lever down) locks the Pi out of drive: centered sticks hold
  neutral rather than handing control back to autonomy. The link stays live
  and frames are still accepted, so drive resumes on the next frame when the
  lever goes back up.
- Pi commands are dropped after `rx_timeout` (50 ms) and the drive falls to
  neutral. The RC transmitter overrides the Pi whenever a stick leaves its
  neutral band and a plausible pulse is present, so a dead receiver holds
  neutral instead of reading as full reverse.
- The idle LED bar reads state of charge from a measured 2S discharge curve
  (`src/battery.cpp`, load voltage at idle current), not raw volts. It reads
  low under driving load, when the voltage sags further.
- Current is packed as mA in a `uint16`, covering 0 to 65.5 A against an 8 A
  shunt setting; negative current clamps to 0.
- Telemetry timestamps are `micros()` in an `int32`, so they wrap every ~35.8
  minutes. Consumers must treat the field as a wrapping counter.
