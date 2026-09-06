# Architecture

## Overview

Two ends of one serial link. A Teensy 4.1 on the NEO-PIT drive PCB owns every
actuator and sensor on the RACECAR Neo V2; a Raspberry Pi 5 sends normalized
drive commands and consumes telemetry. The Teensy runs eight cooperative threads
under TeensyThreads plus the Arduino `loop()`; there is no RTOS and no dynamic
allocation after boot.

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

| Module (path)                            | Responsibility                                              | Depends on                |
| ---------------------------------------- | ----------------------------------------------------------- | ------------------------- |
| `racecar-pit-teensy/include/config.h`     | Every tunable; compile-time invariant checks                 | Arduino, Adafruit_LSM9DS1 |
| `racecar-pit-teensy/include/packets.h`    | Wire format; header init; packet size static_asserts         | `config.h`                |
| `racecar-pit-teensy/include/Mutexes.h`    | Lock declarations, one per shared resource                   | TeensyThreads             |
| `racecar-pit-teensy/include/crc16.h`      | CRC-16/CCITT-FALSE, Arduino-free                             | none                      |
| `racecar-pit-teensy/include/drive_map.h`  | Normalized-to-PWM mapping, hall direction decode; Arduino-free| none                      |
| `racecar-pit-teensy/src/main.cpp`         | Boot: peripherals, interrupts, thread launch                 | `helpers.h`               |
| `racecar-pit-teensy/src/helpers.cpp`      | Threads: telemetry, command RX, sensors, actuation, displays | all of the above          |
| `racecar-pit-pi/PiUART.py`                | Standalone Pi-side link reference; no ROS                    | numpy, pyserial           |

## Threads

Started in `setup()`; each loops forever and yields with `threads.delay()`.

| Thread            | Period | Reads                              | Writes                    |
| ----------------- | ------ | ---------------------------------- | ------------------------- |
| `testPWM`         | 1 ms   | `channelVals`, `cmd_setpoint`, hall| servo + ESC PWM           |
| `RxReceive`       | 1 ms   | PPM pin                            | `channelVals`             |
| `ReadCurrAndVolt` | 1 ms   | INA226 (I2C)                       | `power`                   |
| `ReadIMUStates`   | 1 ms   | LSM9DS1 (I2C)                      | `imuVals`                 |
| `fillTxBuff`      | 1 ms   | `channelVals`, `imuVals`, `power`  | `txPacketPing`            |
| `UARTTx`          | 5 ms   | `txPacketPing`                     | `Serial1`                 |
| `UARTRx`          | 1 ms   | `Serial1`                          | `cmd_setpoint`, displays  |
| `DotMatrix`       | 50 ms  | `dotmatrix_cmd`, `system_state`    | MAX7219 (SPI)             |
| `loop()`          | 25 ms  | `led_cmd`, `system_state`, `power` | WS2812B strip             |

Shared state is guarded per resource, never by one global lock: `CmdMutex`
(command setpoints and display buffers), `RxMutex` (`channelVals`), `ImuMutex`,
`PowerMutex`, `TransmitMutex` (`txPacketPing`), `ReceiveMutex`/`RecPongMutex`
(the RX ping/pong pair), `I2CMutex` and `SPIMutex` (buses), `ErrMutex`
(`errorFlag`). The hall ISR state is read under `noInterrupts()` rather than a
mutex, since an ISR cannot block on one.

## Data pipeline

Telemetry, Teensy to Pi:

```
LSM9DS1 --I2C--> ReadIMUStates -> imuVals[9] --+
INA226  --I2C--> ReadCurrAndVolt -> power[3] --+--> fillTxBuff -> txPacketPing
iA6B    --PPM--> RxReceive -> channelVals[8] --+         |
hall    --IRQ--> onHallChange -> getRPM -------+         v
                                            UARTTx (200 Hz, CRC-16) --> Serial1
```

Command, Pi to Teensy:

```
Serial1 -> UARTRx (magic sync, CRC-16 gate) -> rxPacketPong -> fillRxData
             -> cmd_setpoint[2] -> testPWM -> speedPID -> ESC PWM
             -> dotmatrix_cmd / led_cmd / system_state -> DotMatrix, ledStuffs
```

`getRPM` averages the last five hall intervals, converts electrical to
mechanical RPM, then low-pass filters at `rpm_filter_alpha`. `getCurrentSpeed`
applies the two gear ratios and the wheel circumference to reach m/s at the
ground, which is what the encoder field carries and what the speed loop closes
on.

## Interfaces

| Interface | Pins / device                     | Defined in                     |
| --------- | --------------------------------- | ------------------------------ |
| Pi UART   | `Serial1`, 921600 8N1             | `cfg::UART`                    |
| USB debug | `Serial`, 115200, `/dev/ttyACM0`  | `cfg::DEBUG`                   |
| Servo PWM | pin 2                             | `cfg::SERVO_D135F`             |
| ESC PWM   | pin 3                             | `cfg::ESC_10BL60_G2`           |
| PPM in    | pin 5                             | `cfg::FLYSKY_IA6B_RX`          |
| WS2812B   | pin 9                             | `cfg::RGB_LED_WS2812B`         |
| MAX7219   | SPI, CS pin 10                    | `cfg::MAX_7219_DOT_MATRIX`     |
| Hall      | pins 30, 31, 32 (IRQ on CHANGE)   | `cfg::ES2845S_4900KV_MOTOR`    |
| I2C       | INA226 at 0x40, LSM9DS1           | `cfg::INA226_SENSOR`, `cfg::LSM9DS1_IMU` |

The wire format in `packets.h` is shared with `racecar_neo_ros2_driver`
(`pit_protocol.py`). Struct sizes are pinned by `static_assert`; changing one is
a protocol break that has to land on both sides together.

## Layout note

The Teensy project follows the PlatformIO layout (`include/`, `src/`, `lib/`,
`test/`) rather than the generic `src/ config/ tests/` scaffold. `config/` has no
runtime meaning on a microcontroller: all tunables are compile-time constants in
`include/config.h`.

## Constraints

- Actuation runs at 1 kHz; anything that blocks longer than a few milliseconds
  inside `testPWM` shows up as jitter at the ESC.
- `Serial1` gets a 2048-byte RX buffer. The stock 64 bytes fills in about 0.7 ms
  at 921600 baud, shorter than the scheduler can preempt `UARTRx` for, and a
  474-byte command frame is lost mid-flight when it overflows.
- Pi commands are dropped after `rx_timeout` (50 ms) and the drive falls to
  neutral. The RC transmitter overrides the Pi whenever a stick leaves its
  neutral band and a plausible pulse is present, so a dead receiver holds
  neutral instead of reading as full reverse.
- Current is packed as mA in a `uint16`, covering 0 to 65.5 A against an 8 A
  shunt setting; negative current clamps to 0.
- Telemetry timestamps are `micros()` in an `int32`, so they wrap every ~35.8
  minutes. Consumers must treat the field as a wrapping counter.
