# Test log

One entry per run. Record numbers, not adjectives.

---

## TL-030 - 2026-11-06

### Setup
Release regression on the reference car, clean build `v0.9.2.8-pcb`
(`d504479`), fleet speed constants (`cal.source=builtin`), battery 7.5 to
7.6 V. Floor runs: Pi publishes `/drive` while RB is held, one command per try
from standstill (`pwm_sweep.py setpoint`), lidar stops on.

### Hypothesis
The release build keeps the TL-026 speed loop, the 200 Hz link, and every
display state; normal starts do not approach the stall guard.

### Observations
| Check | Result |
| ----- | ------ |
| Telemetry | 200.0 Hz on every topic, stamp gaps median 5.00 ms, max 5.00 ms |
| Displays | NEO V2 at power-on and idle, OFF on a driver restart, WARN / HIGH / CURR and LOCK on stall trips; checked by the operator |
| Stall trips | TL-028 |

| Target | Try | Start (0.3 s of motion) | Hold speed, last 1 s | Peak current | Longest above 10 A |
| ------ | --- | ----------------------- | -------------------- | ------------ | ------------------ |
| 0.51 m/s reverse | 1 | 0.46 s | 0.46 m/s | 16.7 A | 64 ms |
| | 2 | 0.42 s | 0.56 m/s | 13.1 A | 24 ms |
| | 3 | 0.40 s | 0.56 m/s | 8.6 A | 0 ms |
| 0.75 m/s reverse | 1 | 0.31 s | 0.755 m/s | 18.6 A | 105 ms |
| | 2 | 0.23 s | 0.825 m/s | 16.2 A | - |
| | 3 | 0.28 s | 0.745 m/s | 18.3 A | - |

- Hold speeds are within 0.08 m/s of the target (limit 0.15).
- Starts at 0.51 m/s take 0.40 to 0.46 s, inside the 0.20 to 0.55 s of the
  same constants in TL-026, above the 0.3 s target; the battery was 7.53 V.
- Normal starts stay above 10 A for at most 105 ms against the 500 ms trip.

### Conclusion
Supported, with the slow 0.51 m/s reverse starts carried over from v0.9.1.
Logs: /mnt/nvme/data/parity-reverse/sweeps/sweep_20261106_2146*.csv,
sweep_20261106_214720.csv.

---

## TL-029 - 2026-11-06

### Setup
Reference car, stationary, teleop running. Per-car constants build over USB
serial with `scripts/car_constants.py`; soft reboots with
`teensy_loader_cli -s -b` (no flash). Reset cause and restarts from the
firmware info and the pit_node log.

### Hypothesis
Saved constants load at boot, an erased block falls back to the fleet
values, and saves do not reset the board.

### Method
1. `get` on an empty EEPROM; `set pid.ff_rev_us=95`; `set pid.kp=999`; `save`;
   soft reboot; `get`.
2. Stress: alternate `pid.ff_rev_us` 95 and 96 with a `save` after each,
   2800 saves on the first build (700, then 2100) and 2100 on the fixed build.
3. `erase`, soft reboot, `get`.

### Observations
| Step | Result |
| ---- | ------ |
| Empty EEPROM | `cal.source=builtin`, `cal.eeprom=empty`, `pid.ff_rev_us=94.000` |
| `set pid.kp=999` | refused, `bad value pid.kp` |
| Save, reboot | `cal.source=eeprom`, `cal.crc=4f7c`, `cal.written=2026-11-06`, `pid.ff_rev_us=95.000` |
| Stress, 100 ms watchdog throughout | 3 board resets in 2800 saves, cause `watchdog`; block still `ok` after each |
| Stress, watchdog 1.5 s during a save | 0 resets in 2100 saves; 3 saves over 40 ms (sector erase), longest 84.9 ms; median 25.5 ms |
| Erase, reboot | `cal.source=builtin`, `cal.eeprom=empty` |

- Save times are measured on the Pi from command to reply.
- About one save in 700 fills a flash sector; the erase and rewrite then
  hold the loop past the 100 ms watchdog.

### Conclusion
Supported after the fix: `wdt::flashWrite` sets the watchdog to 1.5 s for the
duration of a save. The car was left on the fleet values.

---

## TL-028 - 2026-11-06

### Setup
Reference car, stall guard build (`8d9abc9` plus the guard wiring), battery
7.5 V. Forced stalls on the stand with the wheels up and one rear tire
blocked; then a floor run, driven by the operator. Logged
from the Pi: `/battery/current`, `/battery/voltage`, `/encoder/speed` and the
driver's `/pit/stall` (about 50 rows per second).

### Hypothesis
With trip at 10 A, 0.1 m/s and 0.5 s, a blocked wheel trips in about 0.5 s,
in both directions; the third trip within 60 s gives the 30 s lockout; normal
floor starts do not trip.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| `trip_a` | 10.0 | A |
| `moving_mps` | 0.10 | m/s |
| `trip_us` | 500000 | us |
| Cooldown, lockout | 2, 30 | s |
| Lockout trips, window | 3, 60 | -, s |

### Observations
| Trip | Direction | Time over 10 A before trip | Peak current | Current under 5 A after trip | State after release |
| ---- | --------- | -------------------------- | ------------ | ---------------------------- | ------------------- |
| 1 | forward | 496 ms | 21.9 A | 74 ms | cooldown 2 s |
| 2 | forward | 547 ms | 25.0 A | 53 ms | cooldown 2 s |
| 3 | forward | 542 ms | 23.5 A | 58 ms | lockout 30 s |
| 4 | reverse | 497 ms | 21.5 A | 63 ms | cooldown 2 s |
| 5 | reverse | 496 ms | 21.5 A | 65 ms | cooldown 2 s |
| 6 | reverse | 483 ms | 22.3 A | 48 ms | lockout 30 s |

- Both lockouts ended 30.0 s after the release; each cooldown ended at 2.0 s.
- Speed read 0 through most of each stall window, with single readings up to
  0.9 m/s as the rotor rocked; the stall timer counts back down instead of
  resetting, so these did not stop the trips.
- Stall currents reached 21 to 25 A, above the old 16.38 A clip (TL-027 range).
- Floor run, about 47 s: 9 reverse and 18 forward starts, reverse peaks
  9.5 to 19.3 A, speeds to 1.65 m/s; 0 trips.
- Current under 5 A includes the INA226 averaging (35 ms) and topic delivery.

### Conclusion
Supported. Thresholds kept. Log:
/mnt/nvme/data/stall-tests/stall_2026-11-06_p4.csv.

---

## TL-027 - 2026-11-06

### Setup
Reference car on the stand, Pi running teleop with lidar and IMU fusion,
wheels still. Before: `v0.9.2.4-pcb` (INA226 range 8 A, 0.5 mA LSB). After:
the same tree with the 40 A range (1.22 mA LSB, unrounded) and the dip
record. `/battery/current` and `/battery/voltage` sampled from the Pi.

### Hypothesis
The 40 A range keeps the idle current and voltage readings within their
normal spread, with no offset.

### Method
Mean, standard deviation and range of each topic over 20 s before and 25 s
after the flash, at about 200 samples per second.

### Observations
| Name | Before | After | Unit |
| ---- | ------ | ----- | ---- |
| Current mean | 1.964 | 1.926 | A |
| Current sd | 0.075 | 0.037 | A |
| Current min to max | 1.872 to 2.252 | 1.867 to 2.242 | A |
| Voltage mean | 7.832 | 7.825 | V |
| Voltage sd | 0.0019 | 0.0016 | V |
| Info `power.max_current_a` | 8.0 | 40.0 | A |

- The 0.038 A difference in means is inside one standard deviation of the
  before run; idle load varies with the Pi.
- `power.min_mv` and `power.last_dip_ms` read -1 in the info latched at
  startup: the driver requests it within about 1 s of a reset, before the
  first voltage sample. A later request reports current values.

### Conclusion
Supported. No offset or scale change at idle; current above 16.38 A is
checked during the stall tests.

---

## TL-026 - 2026-11-02

### Setup
Same car and floor as TL-025, normal and trace builds (`teensy41_trace`) of
`v0.9.1.2-pcb` to `v0.9.1.10-pcb`. Pi publishes `/drive` while RB is held,
one normalized command per run from standstill. Battery 7.4 to 7.9 V, then a
charged battery at 8.3 V from the `ki_rev` 150 rerun on.

### Hypothesis
With motion confirmed only after sustained hall intervals and a reverse start
assist, the closed loop starts reverse every time and holds 0.51 and 0.75 m/s
within 0.10 m/s.

### Method
Closed-loop starts at commands 0.02 (0.51 m/s) and 0.505 (0.75 m/s), held
2 s (one run 3 s, one 4 s). Start time is the first sample of 0.3 s of
continuous motion at 0.05 m/s or more; hold speed is the mean of the last 1 s.
The trace build logs target, measured speed and ESC PWM every 25 ms.

### Parameters
| Build | Reverse feedforward | Reverse start ramp | Handover | `ki_rev` | Reverse boost |
| ----- | ------------------- | ------------------ | -------- | -------- | ------------- |
| .2 | 66 + 58/m/s | 160 to 240 us at 1000 us/s | at 5 intervals over 100 ms | 60 | off once moving |
| .3 | 88 + 38/m/s | same | over 50 ms | 60 | off |
| .4 | 88 + 38/m/s | 160 to 320 us at 2000 us/s | same | 60 | off |
| .5 | 93 + 38/m/s | same | same | 60 | off |
| .6 | same | same | same | 150 | off |
| .7 | same | same | until 0.5 m/s or 80% of target | 100 | off |
| .9 | same | same | same | 60 | on |
| .10 | 94 + 55/m/s | same | same | 60 | on |

### Observations
| Build | Target | Starts (0.3 s motion) | Hold speed, last 1 s |
| ----- | ------ | --------------------- | -------------------- |
| .2 | 0.51 | 3 of 3 after 0.07 to 0.75 s, several locked tries first | 0.37 to 0.62 |
| .3 | 0.51 | 0 of 9 tries; each locked at 12 A for 0.6 s | - |
| .4 | 0.51 | 6 of 6 in 0.23 to 1.16 s, starts at 16.38 A | 0.35 to 0.40 |
| .5 | 0.51 | 6 of 6 in 0.22 to 0.35 s | 0.35 to 0.48; 3 s hold 0.44 |
| .5 | 0.75 | 3 of 3 in 0.21 to 0.25 s | 0.62, 0.68 |
| .6 | 0.75 | 6 of 6 in 0.19 to 0.30 s | 0.73 to 0.88, overshoot to 1.0 |
| .7 | 0.75 | 3 of 3 in 0.23 to 0.25 s | 0.52 to 0.82 |
| .9 | 0.75 | 3 of 3 in 0.16 to 0.21 s | 0.69 to 0.77 |
| .9 | 0.51 | 3 of 3 in 0.24 to 0.27 s | 0.39 to 0.46 |
| .10 | 0.51 | 3 of 3 in 0.20 to 0.55 s | 0.52 to 0.54, sd 0.02 to 0.04 |
| .10 | 0.75 | 3 of 3 in 0.22 to 0.43 s | 0.81, 0.81; 4 s hold 0.86, 0.96, 0.87 by second |

- Trace, `.4`: at confirmed motion (0.12 s) the output fell from 320 to
  129 us and the car sat at 0.35 to 0.49 m/s while the integrator added about
  42 us/s; the PWM matched the feedforward and gains.
- Trace, `.10`, 4 s hold at 0.75: PWM 136 us at 1 to 2 s, 122 us at 2 to 3 s,
  115 us at 3 to 4 s; speed peaked near 1.0 m/s at 2.5 s and fell to 0.80.
  At 0.51 the PWM settled near 122 us. Reverse needs less PWM to hold a speed
  once moving fast than from a fresh start, and takes 2 to 3 s to settle.
- Reverse starts read 16.38 A (the INA226 range) for about 40 ms.
- The first speed sample of a reverse start often read forward (+0.5 m/s).

### Conclusion
Supported for starts: no reverse lock from `.4` on (about 30 tries); start
times 0.16 to 0.55 s, most 0.20 to 0.27 s. Supported at 0.51 m/s (within
0.03). At 0.75 m/s the hold overshoots by up to 0.2 m/s and settles slowly;
no single feedforward line fits reverse, whose PWM for a speed depends on the
speed history. Reverse accepted for 0.9.1 at a start time of 0.3 s or less and
a hold within 0.15 m/s after 2 s.

---

## TL-025 - 2026-11-02

### Setup
Same car, floor and lanes as TL-024, after an ESC factory reset (items
3,4,3,5,3,2,1,2,1,9,1,1 re-entered and read back) and a throttle calibration
at 1500 / 2000 / 1000 us (`teensy41_esc_cal`, `scripts/esc_cal.py`). Raw-PWM
build at `24c1dd6` (bounds 1000/2000). Before the sweep, a gamepad drive on
the normal build with the 0.9.0 feedforward (55/14, 28/8).

### Hypothesis
The fleet calibration moves the reverse PWM-to-speed line far enough that the
0.9.0 reverse feedforward no longer starts the car, and a refit from the
TL-024 method restores both directions.

### Method
TL-024 method. Running-point launches: 100 us forward; 210, 230 and 260 us
reverse, until speed holds 0.5 m/s for 0.1 s. Starts on the gamepad drive
counted at 0.3 s of continuous motion at 0.05 m/s or more.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| Breakaway step, hold, rest | 5, 1.0, 1.0 | us, s, s |
| Running hold | 3.0 | s |
| Launch handover | 0.5 for 0.1 | m/s for s |

### Observations
| Direction | Offset | Result |
| --------- | ------ | ------ |
| Forward | 50 to 75 us | no start, 2.2 to 3.0 A |
| Forward | 80, 85 us | twitch; 85 us starts 2 of 3 |
| Forward | 90 us | starts 3 of 3 in 0.07 to 0.17 s, 4.3 A |
| Forward | 80 us | 0.81 m/s, rising 0.06 m/s per s |
| Forward | 85 us | 1.12 m/s, rising 0.12 m/s per s |
| Reverse | 90 to 120 us | no start, 2.5 to 3.0 A |
| Reverse | 125 to 200 us | twitch then lock at every step, 3.2 to 8.4 A |
| Reverse | 205 us | starts 3 of 3 in 0.09 to 0.13 s, about 8 A |
| Reverse launch | 210, 230, 260 us | 210: 0 of 1 (lock 9.5 A); 230: 1 of 2 (lock 11.5 A); 260: 1 of 1 |
| Reverse | 100 us | decays from 0.8 to 0.36 m/s, then locks at about 7 A |
| Reverse | 115 us | 0.84 m/s, sd 0.04, settled |
| Reverse | 130 us | 1.10 m/s, sd 0.04, settled |

Gamepad drive on the 0.9.0 feedforward: forward 8 starts in 0.13 to 0.45 s;
reverse 6 of 7 starts after 1.4 to 2.4 s at 4 to 12.5 A, one none in 2 s;
6 to 82 speed samples of 0.05 m/s or more per reverse pull before motion.
The first reverse tick at full stick is about 96 us and the integrator adds
about 60 us per s, reaching 205 us after about 1.8 s.

### Conclusion
Supported. Breakaway moved from 75 to 90 us forward and from 75 to 205 us
reverse; at item 6 = 50% both are about 18 to 20% effective throttle. Fits:
forward 69 us + 12.5 us per m/s, reverse 66 us + 58 us per m/s (two points
each). Start margin: forward about +15 us; reverse about -80 us, so reverse
needs a start assist beyond the feedforward. Reverse starts at a fixed step
lock at random up to 230 us; reverse does not run steadily at 100 us.

---

## TL-024 - 2026-11-02

### Setup
Car on a laminate floor: a lane about 3 m long for breakaway, a longer lane
for running points, both about 0.7 m wide with lidar stops at the end
(0.3 m plus 0.6 m per m/s) and the sides (0.30 m from the lidar). ESC as
found: items 3,4,3,5,3,2,1,2,1,9,1,1, throttle calibration unrecorded.
Raw-PWM build (`teensy41_raw_pwm`, `fw.speed_control=raw_pwm`): a Pi command
n drives the ESC at 1500 + 1000 * n us, open loop. Battery 7.79 V at rest.
Pi publishes `/drive` while RB is held; steering holds the gyro heading
(at most 0.15) with the gyro bias learned at rest.

### Hypothesis
The open-loop PWM-to-speed line matches the v0.9.0 feedforward fitted in
TL-022 (forward 55 us + 14 us per m/s, reverse 28 us + 8 us per m/s).

### Method
- Breakaway: from standstill, offsets in 5 us steps, 1 s each, 1 s neutral
  between. A step counts when speed stays at 0.05 m/s or more for 0.3 s; the
  output then goes to neutral. Breakaway is the first step to count 3 times
  in a row.
- Running points: launch at 80 us until speed holds 0.5 m/s (0.9 m/s for one
  run) for 0.1 s, then hold one offset. Steady speed is the mean of the last
  1 s; a run whose speed drifts more than 0.05 m/s per s there is not settled.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| Breakaway step, hold, rest | 5, 1.0, 1.0 | us, s, s |
| Sustained motion | 0.3 at 0.05 | s at m/s |
| Running hold | 2.5 to 5.0 | s |
| Raw-PWM offset limit | 300 | us |
| Current stop | 16 A for 0.5 s | - |

### Observations
| Direction | Offset | Result |
| --------- | ------ | ------ |
| Forward | 40 to 70 us | no start; 2.4 to 3.7 A (idle 2.2 A); 65 and 70 us twitch only |
| Forward | 75 us | starts 3 of 3 in 0.07 to 0.19 s, 63 to 75% moving, 4.2 A |
| Forward | 58, 62 us | launched car coasts to a stop within 0.3 s |
| Forward | 66 us | 0.67 m/s, sd 0.03, settled for 4 s |
| Forward | 70 us | 1.03 m/s, sd 0.05, over 0.75 s (side stop) |
| Reverse | 20 to 30 us | no start, 2.3 to 2.6 A |
| Reverse | 35 to 65 us | twitch then lock: 0.12 s of motion (1 cm), current 3.0 to 9.9 A rising with the offset |
| Reverse | 65 to 75 us | starts or locks per try: 65 us 1 of 3, 70 us 2 of 3 (one lock at 11.8 A), 75 us 3 of 3 (one cut by the end stop while moving); 8.7 to 12.5 A |
| Reverse | 40 us | 1.05 m/s, sd 0.05, settled for 2.4 s |
| Reverse | 35 us | 0.62 m/s and still slowing 0.08 m/s per s |

- Each twitch read -0.36 to -0.45 m/s in the speed estimate while the car
  moved about 1 cm: the median of the first few hall intervals.
- Reverse starts lunge: 0.35 to 0.8 m per burst by the encoder (the operator
  saw 1 to 2 m), peaks 1.1 to 2.2 m/s, about 0.7 s of coasting after neutral.
  Launches at 80 us peaked at 14.3 to 14.5 A.
- Open loop below 70 us forward settles within about 0.3 s from a 0.5 m/s
  launch; at 70 us it is still rising after 2.6 s from 0.5 m/s.
- At rest the fused gyro drifted -0.77 deg/s; a 90 deg hand turn read
  +88 deg.

### Conclusion
Supported within about 4 us. Forward: 66 us gives 0.67 m/s against 64.4 us
from the v0.9.0 line; 70 us gives 1.03 m/s against 69.4 us. Reverse: 40 us
gives 1.05 m/s against 36.4 us; the slope from 35 and 40 us is about
11 us per m/s against 8. Starting needs about 75 us both ways, above the
running offsets (forward 64 to 70 us, reverse 35 to 40 us for 0.6 to
1.05 m/s), and reverse below 70 us locks at up to 12 A instead of turning.

---

## TL-023 - 2026-10-26

### Setup
Car on a block, wheels off the ground, transmitter off. Plain build of
`7505357` (`v0.9.0.11-pcb`) unless noted; profiling build for the profile
run. Fresh battery, 7.71 V.

### Hypothesis
The 0.9.0 firmware passes the regression suite: info reply, timing targets,
command loss, no receiver, a missing sensor at boot, and repeated uploads.

### Method
- Info reply from the car (`scripts/pit_uart.py --info`) against a host
  build of the same commit.
- `scripts/profile_capture.py 60` on the profiling build.
- Two uploads (profiling build, then plain).
- Wheels spinning at 0.82 m/s on the gamepad (LB), then `racecar-teleop`
  restarted.
- IMU cable unplugged at the IMU, Teensy rebooted by an upload, info read;
  cable plugged back in, info read.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| Info keys | 88 | - |
| Profile capture | 60 | s |
| Teleop restart at wheel speed | 0.82 | m/s |

### Observations
| Check | Result |
| ----- | ------ |
| Info reply | Every configuration key equals the host build; differences only in `fw.git`/`fw.label` (car on `35f2216`, the later commits change docs only), the IMU register readback, and `status.*` |
| Telemetry rate, gap max | 200.0 Hz, 5.000 ms |
| Drive loop period mean, max | 999.5 us, 1.000 ms |
| Command to PWM max | 999 us |
| Interrupt lateness max | 0 us |
| IMU, power reads | 250, 100 per s |
| Command CRC failures | 0 |
| Uploads | 2 of 2 booted; each first write failed, the retry succeeded |
| Teleop restart | Wheels stopped at once; LED loading sweep when teleop came back (operator) |
| No receiver | Gamepad drove with the transmitter off |
| IMU missing at boot | `status.imu=missing`, `status.power=ok`, watchdog armed, telemetry and encoder published |
| IMU plugged back in | `status.imu=ok`, rates 952 / 952 / 155 Hz, accel z 10.3 m/s^2 |

- Unplugging and plugging the IMU cable each reset the Teensy (three ESC
  beeps; `status.last_reset=reset`, flags 1, the power-on reset, not the
  watchdog). The connector dips the 3.3 V supply. The sensor coming back
  `ok` was therefore a fresh boot, not detection while running.
- Cold boot: the battery swap earlier the same day booted `35f2216` from
  power-on (`reset_flags=1`). A full system power cycle was not repeated.

### Conclusion
Supported. A sensor plugged in while the board runs resets the Teensy
through its supply; the board recovers on its own.

---

## TL-022 - 2026-10-26

### Setup
Car on the floor in a straight lane about 5 m long and 0.7 m wide, battery
7.58 V at the start and 7.45 V at the end. Pi publishes `/drive` while RB is
held: fixed speed, steering from a gyro heading hold (at most 0.15 rad), with
lidar stop zones (15 cm at the sides; the end of the lane at 0.3 m plus 0.6 m
per m/s). Trace build (`-DRCN_SERIAL_PLOT`) for target, measured speed and
PWM on USB serial. `/drive` speed is a normalized command: 0.02, 0.5 and 1.0
give setpoints of 0.51, 0.75 and 1.0 m/s.

### Hypothesis
The steady speed error comes from the feedforward, not the gains; a refit
brings each direction within +/- 0.05 m/s of the setpoint.

### Method
Six runs per sweep, 3 s each, forward then reverse at each setpoint. Steady
value is the mean over the last 1 s. One change set per sweep.

### Parameters
| Name | Sweep 1 | Sweep 2 | Sweep 3 | Sweep 4 | Unit |
| ---- | ------- | ------- | ------- | ------- | ---- |
| `deadband_fwd_us` / `kv_ff_fwd` | 75 / 10 | 55 / 14 | 55 / 14 | 55 / 14 | us, us per m/s |
| `deadband_rev_us` / `kv_ff_rev` | 70 / 15 | 35 / 10 | 35 / 10 | 28 / 8 | us, us per m/s |
| `stiction_kick_rev_us` | 0 | 0 | 0 | 0 | us |
| `ki` | 30 | 30 | 30 | 60 | - |
| Hall direction confirm | 1 | 2 | 2 | 2 | transitions |
| Output limited to the target's side of neutral | no | no | yes | yes | - |
| `kp_boost` in reverse once moving | yes | yes | no | no | - |
| Speed estimate | repeated interval | repeated interval | repeated interval | median of 5 | - |

A practice run before sweep 1, on `kv_ff_rev` 40 and kick 100 us, reversed at
-1.5 to -1.7 m/s for a 0.75 m/s setpoint.

### Observations
| Setpoint | Sweep 1 | Sweep 2 | Sweep 3 | Sweep 4 | Unit |
| -------- | ------- | ------- | ------- | ------- | ---- |
| +0.51 | 0.71 | 0.53 | 0.55 | 0.54 | m/s |
| +0.75 | 0.96 | 0.82 | 0.79 | 0.77 | m/s |
| +1.00 | 1.22 | - | 1.08 | 1.04 | m/s |
| -0.51 | -0.85 | -0.47 (sd 0.37) | -0.64 | -0.59 | m/s |
| -0.75 | -1.22 | side stop | -0.97 | -0.84 | m/s |
| -1.00 | -1.44 | - | -1.26 | -1.09 | m/s |
| Wrong-sign speed samples | 4 / 703 | 4 / 381 | 2 / 723 | 0 / 724 | - |
| Forward start peak at +0.51 | 1.10 | 1.60 | 1.75 | 1.17 | m/s |

- Sweep 1: forward PWM held 1565 to 1572 us at every setpoint; the car gains
  about 1 m/s per 14 us above a 55 us deadband. The integrator at `ki` 30 adds
  6 us/s at a 0.2 m/s error, too slow to remove the offset in 3 s.
- Sweep 2: reverse oscillated between -1.49 and +0.43 m/s with PWM 1399 to
  1515 us. A speed spike drove `kp_boost` past neutral to a forward pulse; the
  ESC braked, the drivetrain jerked, and the halls read forward transitions.
  The car visibly flipped back and forth.
- Sweep 3: reverse PWM held 1449 to 1468 us with no oscillation; forward
  spikes to 1.75 m/s at 0.5 m/s came from single short hall intervals
  repeated in the five-sample average.
- Sweep 4: reverse still decreasing at 3 s (-1.17 m/s mean over 1 to 3 s,
  -1.09 over 2 to 3 s at a 1.0 setpoint): the start boost overshoots and the
  integrator pulls it back over about 3 s.
- Manual driving afterward on the same firmware (operator report): starts
  and stops without the earlier jerk, and forward and reverse respond the
  same way to the stick.
- The lidar end-of-lane check ended one practice reverse run on the antennas,
  which sit 0.27 m behind the lidar and sway past the rear of the footprint;
  they are masked in the sweep tool.

### Conclusion
Supported forward: within +0.04 m/s at each setpoint. Partly supported in
reverse: 0.08 to 0.09 m/s fast at 3 s and still converging; the residual comes
from the start boost overshoot.

---

## TL-021 - 2026-10-26

### Setup
As TL-020, with telemetry sent from a 200 Hz IntervalTimer interrupt
(priority 160), the info payload built by an `InfoBuilder` thread into
double buffers, and the IMU and INA226 readings shared through sequence
counters.

### Hypothesis
With the frame sent from its own timer, the telemetry gap no longer depends
on the sensor threads; the IMU read split keeps its saving.

### Method
`scripts/profile_capture.py 60` twice on the profiling build; info reply;
watchdog test build with the telemetry tick, the info builder and the drive
tick stopped in turn; plain build, `scripts/telemetry_probe.py 10` three
times.

### Parameters
| Name | TL-020 | Timer telemetry | Unit |
| ---- | ------ | --------------- | ---- |
| Telemetry rate | 200.0 | 200.0, 200.0 | Hz |
| Telemetry gap p99 bin / max (firmware) | 6.5 / 6.47 | 5.05 / 5.000 | ms |
| Teensy-stamped gap max at the Pi (plain build) | - | 5.04, 5.02 | ms |
| Telemetry tick time | - | 12 | us |
| `imu_read` | 715 | 715 | us |
| Drive loop period max | 1.000 | 1.000 | ms |
| Interrupt lateness max | 0 | 1 | us |
| Info reply keys | - | 88 | - |

### Observations
- Every telemetry target passed in both profiling runs. The capture first
  printed 196.7 Hz for run 2: one report's lines were missing and the tool
  divided by all 61 windows; it now rates each stat over the windows it
  appears in, and run 2 recomputes to 200.0 Hz.
- Telemetry tick, info builder and drive tick stopped in turn: each reset the
  Teensy by the watchdog (`status.last_reset=watchdog`, flags 128).
- The first probe after the plain flash showed 2.78 ms median and 24.7 ms max
  gaps while `pit_node` re-mapped the Teensy clock after the reset; 5 s later
  every gap was 5.00 to 5.04 ms.

### Conclusion
Supported. Telemetry leaves every 5.00 ms regardless of the sensor threads;
the IMU read split stays.

---

## TL-020 - 2026-10-26

### Setup
Reference car on a block, teleop running. Profiling build of `3e3cfdd` plus
the LSM9DS1 register readback and the read split: `getAccel()`,
`getGyro()`, `getMag()` in place of `getEvent(a, m, g, temp)`. A/B against
the plain `3e3cfdd` build.

### Hypothesis
The split removes the temperature read and the second magnetometer read,
shortening each IMU read without changing the values or the timing targets.

### Method
`scripts/profile_capture.py 60` three times; info reply for the registers;
`scripts/telemetry_probe.py` on both builds.

### Parameters
| Name | Before (TL-016, TL-019) | Split | Unit |
| ---- | ----------------------- | ----- | ---- |
| `imu_read` | 1090 | 715 | us |
| Telemetry gap p99 bin / max | 5.6 to 5.9 / 5.57 to 5.85 | 6.5 / 6.47, 6.47, 6.50 | ms |
| Drive loop period max | 1.000 | 1.000 | ms |
| Gyro / accel / mag rate read back | - | 952 / 952 / 155 | Hz |
| Mag full scale read back | - | 8 | gauss |
| Registers CTRL_REG1_G, CTRL_REG6_XL, CTRL_REG1_M, CTRL_REG2_M, CTRL_REG3_M | - | 0xC8, 0x78, 0x62, 0x20, 0x00 | - |

### Observations
- The accelerometer runs at the gyro's 952 Hz; the 119 Hz written to ODR_XL
  has no effect while the gyro runs.
- The magnetometer reads 8 gauss full scale on the chip, matching the
  driver's conversion, so a range mismatch does not explain the 2x field.
- Mag readings were about -57, +136, -32 uT on both builds, against about
  -83, +167, -160 uT in every reading of 2026-10-25 and the morning of
  2026-10-26: a change around the car, not in the firmware.
- The probe's distinct-sample count varied 129 to 180 per second on both
  builds with the car still; the firmware counted 250 distinct reads per
  second on both.
- Telemetry max gap rose from 5.57-5.85 to 6.47-6.50 ms, the same in three
  runs.

### Conclusion
Read time down 34%, values unchanged; but the split worsens telemetry jitter
by about 0.9 ms under round-robin scheduling and fails the 6 ms p99 target.
Not committed as is.

---

## TL-019 - 2026-10-26

### Setup
Reference car on a block, teleop running, transmitter off. Firmware `dee5186`
plus the watchdog (RTWDOG, 32 kHz clock), built three ways: the test build
(profiling plus hang commands) with a 2000 ms timeout, the same with the
shipping 100 ms, and the plain build.

### Hypothesis
A stopped context resets the Teensy within the timeout plus one heartbeat
round; normal running never resets it; uploads keep working with it armed.

### Method
1. 2000 ms build: soft reboot into HalfKay and watch the USB ID for 15 s
   without writing.
2. 100 ms build: stop each of the eight contexts in turn from USB serial; time
   the USB drop and read the info reply after the reboot.
3. Reset latency: hang INA226, LSM9DS1 and dot matrix while counting telemetry
   at the Pi; latency = last frame after the command.
4. 120 s profiling soak with nothing stopped; any reset restarts the report
   clock.
5. Three uploads of the plain build through `scripts/upload.py`, each from
   firmware with the watchdog armed.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| Timeout | 100 | ms |
| Time in HalfKay with a 2000 ms watchdog armed | 15 (no reset) | s |
| Contexts stopped / reset by the watchdog | 8 / 8 | - |
| Reset latency from telemetry (INA226, LSM9DS1, dot matrix) | 144, 149, 144 | ms |
| USB drop after the command, host side | 256 to 418 | ms |
| Watchdog refreshes | 20.0 | per s |
| Refresh gap max | 51.0 | ms |
| Soak | 120.6 s, 0 resets | - |
| Uploads with the watchdog armed | 3 of 3 | - |

### Observations
- The watchdog does not run in HalfKay: the board stayed in the bootloader
  for 15 s with a 2000 ms timeout armed, and the write from it succeeded.
- Every stopped context (telemetry, command receive, INA226, LSM9DS1, RC
  receive, dot matrix, drive tick, `loop()`) reset the Teensy;
  `status.last_reset=watchdog`, `status.reset_flags=128` after each.
- About 3 s after a watchdog reset the reply still read
  `status.watchdog=starting`; later reads `armed`. No reset loop: the board
  stayed up for 8 s after the last test.
- The host sees the USB drop 100 to 270 ms after the telemetry stops: disconnect
  detection, not firmware time.
- Soak: drive loop 1000.0 Hz, telemetry 200.0 Hz (p99 5.9 ms); refresh rate
  set by `DotMatrix` (50 ms loop).
- Uploads still fail on the first write and succeed on the retry, with or
  without the watchdog.

### Conclusion
Supported. Reset 100 to 150 ms after any context stops; no false reset in
120 s; uploads unaffected. Margin is 2x the slowest context: a context added
later with a loop over 50 ms needs the timeout raised.

---

## TL-018 - 2026-10-26

### Setup
Reference car, Teensy on the Pi's USB (bus 3), PJRC udev rule installed,
firmware `dee5186`. PlatformIO `teensy-cli` upload: `teensy_reboot -s`, then
`teensy_loader_cli -mmcu=TEENSY41 -w -s -v`.

### Hypothesis
The first upload fails because the loader opens the HalfKay bootloader before
udev has applied the PJRC rule's permissions.

### Method
One upload with `udevadm monitor` (usb, hidraw, tty) and timestamped loader
output. Then three cycles of: soft reboot, wait for HalfKay, wait 1.5 s,
`teensy_loader_cli` without its own reboot. Then three uploads through
`scripts/upload.py`.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| Soft reboot to "Found HalfKay" | 0.9 | s |
| "Found HalfKay" to "error writing" | 0.57 | s |
| HalfKay HID node added to unbound by the loader | 13 | ms |
| Upload through the script | 14.5 to 14.9 | s |

### Observations
- udev applied the rule to the bootloader's USB device before the loader
  claimed it; the bootloader stayed enumerated through the failed write (no
  further USB events).
- Waiting 1.5 s after the bootloader appeared did not help: 2 of 2 writes after
  a soft reboot failed. A write to a bootloader left over from a failed
  attempt succeeded every time (8 of 8 on 2026-10-25 and 2026-10-26).
- `scripts/upload.py`: 3 of 3 uploads succeeded on attempt 2.

### Conclusion
Permission race not supported. The first write after a soft reboot from the
running firmware fails deterministically; the failed write leaves a
bootloader that accepts the next one. Cause not traced: something the firmware
leaves running across the soft reboot is a candidate. The upload script
retries, which makes uploads complete on one command.

---

## TL-017 - 2026-10-26

### Setup
Reference car on a block, all four wheels free, teleop running, gamepad and
FS-i6 transmitter with iA6B receiver. Firmware A: `c1e3091` (drive loop in a
thread, before sensor pacing). Firmware B: `c1e3091` plus the 1 kHz drive
interrupt, paced sensors, and swA decoded as slow on the low pulse.

### Hypothesis
The drive interrupt leaves the drive path and the switch behavior unchanged;
the wheels-free speed overshoot seen on 2026-10-26 predates it.

### Method
A recorder logged `/motor`, `/encoder/speed`, `/rc/channels`, `/rc/link`,
`/joy` and `/battery/current` while the operator drove a scripted sequence:
gamepad forward and reverse, steering, release of the enable button, RC
override, both switches in both positions. The switch polarity was read by
holding both levers up, then down, twice. The same full-stick gamepad and
slow-mode RC holds ran on firmware A and B. Logs (not in git): `data/raw/drive_regression_20261026.csv`,
`data/raw/drive_ab_old_20261026.csv`, `data/raw/drive_ab_new_20261026.csv`.

### Parameters
| Name | A | B | Unit |
| ---- | - | - | ---- |
| Gamepad full stick, peak speed | 1.89, 2.01, 2.07 | 1.84, 1.96, 1.94 | m/s |
| Gamepad full stick, last 1.5 s of a 3 to 4 s hold | 1.35, 1.22, 1.24 | 1.20, 1.35, 1.21 | m/s |
| Slow-mode RC full stick, peak / last 1.5 s | 1.90 / 1.26 | 1.95 / 1.19 | m/s |
| Gamepad commanded with swD high: wheel speed | - | 0.00 (134 samples) | m/s |

### Observations
- Lever up (away from the operator) sends the low pulse on both switches,
  lever down the high pulse; the FS-i6 starts only with every lever up and
  will not power off while the receiver is on, so a no-signal case was not
  reached.
- On firmware A, swA up (low) gave fast mode, the opposite of the intended
  default; swD behaved as intended (high, lever down, locks the Pi out).
  Firmware B decodes swA low as slow; swD unchanged.
- Gamepad forward and reverse, steering both ways, stop on releasing the
  enable button, RC taking over from the gamepad, swD lockout and release,
  slow-mode RC, and fast-mode RC all behaved as expected on B (operator
  check and recorded speeds).
- With the wheels free the closed loop peaks about twice the 1.0 m/s
  setpoint and is still 0.2 to 0.35 m/s high after 3 to 4 s, on both A and
  B. Fast-mode RC is open-loop PWM: a quarter stick reached 8.6 m/s.

### Conclusion
Supported. The drive interrupt keeps the drive and switch behavior; swA now
starts in slow mode with the transmitter's default lever position. The
wheels-free overshoot is the existing speed loop with no load; check on the
floor with the speed-loop work.

---

## TL-016 - 2026-10-25

### Setup
Reference car on a block, teleop running, 6 min after a reboot, fresh battery
(8.20 V), transmitter off. Profiling build of `c1e3091` plus the drive loop
in a 1 kHz IntervalTimer interrupt (priority 144, `Seqlock` for RC channels
and the drive command), LSM9DS1 read every 4 ms, INA226 every 10 ms.

### Hypothesis
With the drive loop on a timer and the sensors paced, every timing target
holds.

### Method
`scripts/profile_capture.py 60 --probe`, then
`scripts/telemetry_probe.py 10 --runs 3` and the info reply. Pi CPU from
`top` during the runs.

### Parameters
| Name | TL-015 | Timer drive, paced sensors | Unit |
| ---- | ------ | -------------------------- | ---- |
| `drive_loop` rate | 864.7 | 1000.0 | Hz |
| `drive_loop` period mean / max | 1.16 / 2.72 | 1.000 / 1.000 | ms |
| `tx_slot` rate | 200.0 | 200.0 | Hz |
| `tx_slot` p99 / max | 6.45 / 6.48 | 5.60 / 5.57 | ms |
| `imu_loop` / distinct IMU samples | 503 / 503 | 250.0 / 250 | per s |
| `power_loop` | 964 | 100.0 | per s |
| `cmd_to_pwm` mean / max | 10 / 428 | 755 / 999 | us |
| `rx_loop` max | 2.79 | 1.65 | ms |
| `irq_late` max | 0 | 0 | us |
| Telemetry at the Pi (3 probe runs) | - | 197.1, 189.9, 196.7 | Hz |
| Teensy-stamped gaps at the Pi, median / max | - | 5.43 / 84 to 85 | ms |

### Observations
- Every profiling target passed: telemetry rate, gap p99 and max, drive loop
  mean and max, command to PWM, interrupt lateness. p99 values are 50 us bin
  edges, so a p99 can read above the exact max.
- `cmd_to_pwm` rose from 10 to 755 us mean: an accepted command now waits for
  the next 1 ms tick instead of running in the thread after `UARTRx`.
- Sensor values plausible (accel about 9.8 m/s^2 on z, gyro near 0, mag and
  battery steady); info reply `imu.read_hz=250`, `power.poll_ms=10`,
  `i2c.clock_hz=400000`, both sensors `ok`.
- The firmware sent 200.0 frames/s with no gap over 5.57 ms, but the Pi
  received 189.9 to 197.1 Hz: one gap of 84 to 85 ms (about 16 frames) per
  10 s run in the Teensy-stamped stream, plus scattered drops. Load average
  4.4, `pit_node` 50% and `imu_fusion_node` 42% of a core.

### Conclusion
Supported for the firmware. The remaining loss is on the Pi, between the
serial read in `pit_node` and the best-effort subscriber; it needs counters in
the driver. Driving checks (gamepad, RC, switches, stop on release) still to
run with the drive loop in an interrupt.

---

## TL-015 - 2026-10-25

### Setup
As TL-014; profiling build of `26d0fd4` plus `UARTRx` on `CommandFramer`
(frames built from available bytes, no `readBytes`). I2C at 400 kHz.

### Hypothesis
Without the `readBytes` wait, every thread gets more turns and telemetry
holds 200 Hz.

### Method
`scripts/profile_capture.py 60 --probe`; info reply through the new receive
path.

### Parameters
| Name | TL-014, 400 kHz | Non-blocking receive | Unit |
| ---- | --------------- | -------------------- | ---- |
| `drive_loop` rate | 620.2 | 864.7 | Hz |
| `drive_loop` period mean / p99 / max | 1.63 / 6.7 / 7.58 | 1.16 / 2.3 / 2.72 | ms |
| `tx_slot` rate | 194.3 | 200.0 | Hz |
| `tx_slot` p99 / max | 11.10 / 11.37 | 6.45 / 6.48 | ms |
| `rx_frame` mean / max | 4213 / 5108 | 8.2 / 9 | us |
| `cmd_to_pwm` mean / max | 26 / 425 | 10 / 428 | us |
| Distinct IMU samples | 391 | 503 | per s |
| Pi-side loss | none measurable | 0.7 | % |

### Observations
- 60.4 command frames/s accepted, 0 CRC failures, 0 stalled frames.
- Info reply over the new path: `fw.label=v0.9.0.4-pcb`, `i2c.clock_hz=400000`,
  both sensors `ok`.
- The capture first printed 201.4 Hz telemetry and 870.7 Hz drive loop: it
  divided by the window count while the windows ran slightly over 1 s (60
  reports over 60.41 s). Rates now come from the report timestamps; the
  figures above are recomputed that way. Earlier entries are overstated by
  under 1%.

### Conclusion
Supported. Drive loop 865 Hz with a 2.7 ms worst period; telemetry at 200 Hz
with gaps under 6.5 ms. Remaining gap to the targets is round-robin
scheduling (drive max 2.7 ms against 1.5 ms).

---

## TL-014 - 2026-10-25

### Setup
Reference car on a block, teleop running, transmitter off, LED strip on the
idle charge bar. Profiling build (`teensy41_profile`, `fw.profile=1`) of
`fc35e3a` plus the measurement suite. Battery 7.61 to 7.41 V over the runs.

### Hypothesis
Every 1 ms thread runs once per scheduler rotation, so `DriveControl` runs far
below 1 kHz; blocking I2C transfers set the rotation length.

### Method
`scripts/profile_capture.py 60` per variant (CSV in `data/`), `--probe` on two
of them for the Pi-side frame count. Variants: the committed config; 400 kHz
set only in `setup()`; INA226 read every 10 ms; 400 kHz applied again after
`lsm.begin()`. Telemetry probe and info reply on the last.

### Parameters
| Name | 100 kHz, INA 1 ms | 400 kHz in setup | 100 kHz, INA 10 ms | 400 kHz applied after begin | Unit |
| ---- | ----------------- | ---------------- | ------------------ | --------------------------- | ---- |
| `drive_loop` rate | 180.6 | 180.2 | 249.5 | 620.2 | Hz |
| `drive_loop` period mean / p99 / max | 5.57 / 10.0 / 10.2 | 5.56 / 9.4 / 10.0 | 4.03 / 8.75 / 9.76 | 1.63 / 6.7 / 7.58 | ms |
| `tx_slot` rate | 153.9 | 153.4 | 123.9 | 194.3 | Hz |
| `tx_slot` p99 / max | 14.75 / 15.25 | 13.70 / 14.54 | 12.05 / 13.41 | 11.10 / 11.37 | ms |
| `imu_read` | 3937 | 3937 | 3937 | 1090 | us |
| `power_read` | 995 | 995 | 995 | 275 | us |
| `rx_frame` mean / max | 1686 / 5101 | 1669 / 5055 | 3123 / 5089 | 4213 / 5108 | us |
| `led_show` mean / max | 194 / 198 | - | - | 194 / 200 | us |
| `irq_late` max | 0 | - | - | 0 | us |
| `cmd_to_pwm` mean / max | 44 / 439 | - | - | 26 / 425 | us |
| Distinct IMU samples | 181 | 180 | 181 | 391 | per s |
| Pi-side loss | 0.5 | - | - | none measurable | % |

### Observations
- Every 1 ms thread ran at the same rate (180.6 Hz at 100 kHz): one turn per
  scheduler rotation. LSM9DS1 and INA226 reads took 4.9 of the 5.57 ms.
- `Wire.setClock()` in `setup()` had no effect: Adafruit BusIO's `begin()`
  calls `Wire.begin()`, which resets the bus to 100 kHz. TL-012 therefore ran
  at 100 kHz. Applying the clock after `lsm.begin()` cut the reads 3.6x.
- At 400 kHz the accel, gyro, mag and battery values matched the 100 kHz runs;
  telemetry probe 192.9 and 196.0 Hz at the Pi, median gap 5.37 ms; info reply
  `i2c.clock_hz=400000`, both sensors `ok`.
- `FastLED.show()` costs 194 us and no interrupt ran late by 1 us or more; the
  LED strip does not block on this FastLED version.
- `UARTRx` busy-waits in `readBytes` until a whole 474-byte command frame has
  arrived (5.1 ms on the wire); with shorter rotations it starts earlier and
  waits longer (4.2 ms mean at 400 kHz, about 25% of the CPU at 60 Hz).
- `cmd_to_pwm` counts from frame accepted, so it misses time a frame waits in
  the serial buffer before `UARTRx` runs.
- `report_lines_dropped` was cumulative in the first run (78 lines from before
  the capture opened the port); it resets each second from the second run on.

### Conclusion
Supported. `DriveControl` ran at 181 Hz, not 1 kHz. Blocking I2C at an
unintended 100 kHz set the rotation; with the clock applied after `begin()`
the drive loop reached 620 Hz and telemetry 194 Hz. Remaining blockers: the
`readBytes` wait in `UARTRx`, blocking I2C, and round-robin scheduling
(drive-loop max 7.6 ms).

---

## TL-013 - 2026-10-25

### Setup
Reference car on a block, teleop running, transmitter off. Firmware
`00f6bf2` plus the staged tree: `fw.label`, the measured 15-point charge bar
curve, and the I2C knob back at 100 kHz. Battery 7.60 to 7.61 V.

### Hypothesis
At 100 kHz telemetry returns to the 152 Hz of TL-011.

### Method
`scripts/telemetry_probe.py 10 --runs 3` (checks that `pit_node` is the only
publisher) with the INA226 read every 10 ms, then the same with every 1 ms.
Info reply against a host build of the same tree.

### Parameters
| Name | 100 kHz, INA226 10 ms | 100 kHz, INA226 1 ms | Unit |
| ---- | --------------------- | -------------------- | ---- |
| Teensy-stamped rate (runs) | 125.6, 122.4, 124.1 | 152.3, 151.8, 152.8 | Hz |
| Messages received at the Pi (runs) | 125.9, 118.0, 123.2 | 152.6, 145.4, 132.6 | Hz |
| Stamp gap median | 8.25 to 8.29 | 6.28 to 6.45 | ms |
| Stamp gap p99 / max | 10.1 / 14.8 | 10.2 to 11.8 / 13.2 to 15.2 | ms |

### Observations
- The bus clock had no effect on its own: 100 kHz with the 10 ms INA226 read
  ran at the TL-012 rate.
- The first info request failed with a pyserial "multiple access on port"
  error, since `pit_node` reads the same port; the retry matched the host
  build (74 keys, `fw.label=dev`, `i2c.clock_hz=100000`).
- In two of three 1 ms runs the Pi received fewer messages than the Teensy
  stamps show it sent (145.4 and 132.6 against about 152 Hz).
- Accel, gyro, mag, battery and RC readings normal in every run.

### Conclusion
The INA226 poll rate set the telemetry rate: reading the battery less often
leaves the bus to `ReadIMUStates`, which then busy-waits through more
scheduler slices. Kept at 1 ms (the previous behavior) until the scheduler
change. The Pi-side message loss needs a count in the measurement suite.

---

## TL-012 - 2026-10-25

### Setup
Reference car on a block, teleop service running, after a reboot. Firmware
`00f6bf2` plus the staged I2C change: `cfg::I2C::clock_hz` 100000 -> 400000,
INA226 read every 10 ms instead of every 1 ms. Battery 8.05 V.

### Hypothesis
At 100 kHz one LSM9DS1 read holds the bus about 3 ms and starves `UARTTx`;
at 400 kHz telemetry rises toward 200 Hz.

### Method
Telemetry probe (best-effort subscriber, 10 s, one publisher checked each run)
once on the 100 kHz build after the reboot and three times after the flash.
Info reply read with `scripts/pit_uart.py --info`.

### Parameters
| Name | 100 kHz | 400 kHz | Unit |
| ---- | ------- | ------- | ---- |
| Telemetry rate (runs) | 151.9; earlier 152.2, 153.6, 153.1 | 127.1, 132.5, 91.8 | Hz |
| Stamp gap median | 6.25 | 8.20 to 8.30 | ms |
| Stamp gap p95 | 10.09 | 8.98 to 9.24 | ms |
| Stamp gap max | 15.20 | 14.48 to 14.82 | ms |
| Distinct battery voltages | 20.2 | 12.6 to 16.9 | per s |

### Observations
- The upload failed on the first write and succeeded on the retry (13.4 s).
- Info reply: `i2c.clock_hz=400000`, `power.poll_ms=10`, `status.imu=ok`,
  `status.power=ok`. Accel (-0.02, -0.15, +10.00 m/s^2), gyro, mag and
  battery voltage read normally.
- The third run counted 917 frames in 10 s against a median gap of 8.3 ms
  (about 120 Hz).
- TeensyThreads `delay()` loops on `yield()`, and each thread holds a 1 ms
  slice (10 ticks of 100 us). A thread busy-waiting on I2C keeps its whole
  slice, so `UARTTx` runs once per round of all eight contexts.

### Conclusion
Not supported. Telemetry fell by about 20% and the gaps moved from a 5/10 ms
mix to a steady 8.2 ms. The rate is bounded by round-robin wake-up of
`UARTTx`, not by bus time. This run changed the bus clock and the INA226
poll together; TL-013 separates them: the poll change caused the drop.
Next action: send telemetry independent of the thread rounds.

---

## TL-011 - 2026-10-25

### Setup
Reference car on a block, wheels off the ground, teleop service running.
Firmware built from the clean tree at `00f6bf2`. Battery 7.98 V.

### Hypothesis
A build from a committed tree reports its commit, clean, and runs as TL-009.

### Method
`pio run -t upload`; on failure, the same command again. Then
`python3 scripts/pit_uart.py --info` (teleop still running, as TL-010), a diff
against a host build of `writeFirmwareInfo` with `FW_GIT_HASH=00f6bf2`, and a
10 s telemetry probe.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| Telemetry rate | 153.3 | Hz |
| Stamp gap median / p95 / max | 6.23 / 10.09 / 15.07 | ms |
| Info keys | 72 | - |

### Observations
- The first upload rebooted the Teensy into HalfKay, then failed with "error
  writing to Teensy" after 259.7 s (build included). The retry, started with
  the Teensy already in HalfKay, programmed in 17.2 s. The same pattern
  occurred in TL-009.
- Info reply: `fw.git=00f6bf2`, `fw.git_dirty=0`, byte for byte equal to the
  host build.
- Telemetry topics, accel, gyro, mag and battery as in TL-009.

### Conclusion
Supported. Uploads from running firmware fail on the first write on this car;
rerunning `pio run -t upload` once completes them.

---

## TL-010 - 2026-10-25

### Setup
As TL-009, new firmware flashed (`e330426` source plus the uncommitted tree,
`fw.git_dirty=1`). The teleop service could not be stopped without a password
(polkit), so `pit_node` kept the UART open and reading.

### Hypothesis
The Teensy answers the info request with the compiled configuration, and the
reassembly survives lost chunks.

### Method
`python3 scripts/pit_uart.py --info --yaml firmware.yaml` on `/dev/neo-pit-pcb`
while `pit_node` ran. Diff the reply against a host build of `writeFirmwareInfo`
with the same config and commit flags. Probe telemetry for 5 s afterward.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| Reply | 1514 | B |
| Keys | 72 | - |
| Telemetry after | 151.8 | Hz |

### Observations
- The reply came back complete and matched the host build byte for byte:
  `speed.min_mps=0.500`, `speed.max_mps=1.000`, `status.imu=ok`,
  `status.power=ok`. The YAML wrote cleanly.
- With two readers on the port, each process saw only part of the byte
  stream, so chunks were lost; the assembler completed from later cycles.
- `pit_node` telemetry continued at 151.8 Hz.

### Conclusion
Supported. In normal use run the tool with teleop stopped; sharing the port
drops bytes for both readers.

---

## TL-009 - 2026-10-25

### Setup
Reference car, wheels on the ground and parked, teleop service running
(driver v0.9.0, `pit_node`). Teensy on the Pi's USB for flashing; dot matrix
disconnected by that wiring. Firmware: `main` (`e330426` source, indoor speed
band) before, `release/v0.9.0` staged tree after. FlySky transmitter off.
Battery 7.42 to 7.47 V.

### Hypothesis
The new firmware keeps every telemetry topic and value as before and raises
the telemetry rate toward 200 Hz.

### Method
Flash with `pio run -t upload` (`teensy-cli`) while teleop runs. Before and
after, a best-effort rclpy probe subscribes for 15 s to `/imu/lsm9ds1/raw`,
`/mag/raw`, `/encoder/speed`, `/battery/voltage`, `/rc/channels`, `/rc/link`,
`/battery/state` and records message rates and the gaps between Teensy-clock
stamps on `/imu/lsm9ds1/raw`. `journalctl -u racecar-teleop` for link errors.

### Parameters
| Name | Before | After | Unit |
| ---- | ------ | ----- | ---- |
| Telemetry topics rate | 141.6 | 151.4 | Hz |
| Teensy-stamped rate | 143.1 | 151.3 | Hz |
| Stamp gap median | 5.81 | 6.27 | ms |
| Stamp gap p95 | 10.83 | 10.09 | ms |
| Stamp gap p99 | 10.93 | 11.54 | ms |
| Stamp gap max | 11.04 | 15.18 | ms |
| `/battery/state` | 1.0 | 1.0 | Hz |

### Observations
- Every topic publishes after the flash. Accel (-0.69, -0.03, +9.92 m/s^2),
  gyro, mag (about -87, +169, -156 uT raw) and battery voltage match the
  before values within noise. RC link false and all channels -1.0 in both
  runs (transmitter off).
- `pit_node` logged one Teensy clock restart and one odometry gap at the
  flash; no CRC, serial, or reconnect messages.
- The first upload stalled: the HalfKay bootloader's hidraw device was
  root-only until `00-teensy.rules` was installed, and the write that started
  during `udevadm trigger` failed. The retry flashed in 12.9 s.

### Conclusion
Existing telemetry unchanged. Rate +7%, not 200 Hz: the median gap of
6.27 ms shows `UARTTx` waking about every 6.3 ms, so the running deadline
falls behind and resyncs. Not yet checked: the info reply, Pi and RC drive
paths, slow mode, manual override, LED strip.

---

## TL-008 - 2026-10-25

### Setup
Host only, on the car's Pi 5 (no flash). PlatformIO 6.2.0, platform teensy
6.0.0, GCC 13 native. Python 3.12.3, pytest 7.4.4.

### Hypothesis
The subsystem split and the info reply leave the firmware building and the
existing logic tests green, and the info payload fits its buffer and one
telemetry slot.

### Method
`pio run`, `pio test -e native`, `pytest`, `ruff`, `black --check`, `mypy`
from the repository root. Baseline: `pio run` and the two test suites on
`main` (`e330426`) with the libraries the caret ranges resolved on
2026-10-25.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| Info payload | 1514 | B |
| Info chunks | 12 | - |
| Info cycle | 60 | ms |
| Telemetry + largest chunk | 240 | B |
| Native test cases | 40 | - |
| Pytest cases | 29 | - |

### Observations
- teensy41 build SUCCESS. Flash code 86060 -> 90224 B, data 13108 -> 14132 B,
  RAM1 variables 18912 -> 21952 B (2048 B payload, 512 B transmit buffer).
  RAM2 unchanged at 12448 B.
- The baseline build of `main` is 86060 B of code, against 66388 B in TL-007
  for the same source: the unpinned libraries resolved to newer versions.
- Baseline tests: native 21/21, pytest 13/13.
- native: 40/40 passed. pytest: 29/29 passed. ruff, black, mypy clean.
- The commit string is in the binary (`strings firmware.elf`).

### Conclusion
Supported on the host. Not run on hardware. Next action: flash the car, run
`python3 scripts/pit_uart.py --info` with teleop stopped and compare every key
against `config/config.h`, then repeat TL-002 (drive both directions through
the ROS mux, wheels up) and measure the telemetry rate on
`/imu/lsm9ds1/raw`.

---

## TL-007 - 2026-09-06

### Setup
Host only. PlatformIO teensy platform, GCC 13 native.

### Hypothesis
Rescaling the stick onto the slow-mode cap makes full throttle an exact
`slow_speed_mps` setpoint, and closing the loop on RC in slow mode leaves the
fast-mode RC path unchanged.

### Method
`pio run`, `pio test -e native`, `python3 -m pytest tests -q`.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| slow_speed_mps | 1.0 | m/s |
| max_speed_mps | 5.0 | m/s |
| min_speed_mps | 0.75 | m/s |
| Native test cases | 21 | - |

### Observations
- teensy41 build SUCCESS. Flash code 66196 -> 66388 B (+192); RAM1 15104 B and
  RAM2 17568 B unchanged.
- native: 21/21 passed in 0.57 s. Four new cases on `throttleToTargetMps`.
- Full stick returns the cap exactly in both modes: 5.0 and 1.0 m/s, and their
  negatives.
- Half stick in slow mode lands strictly between 0.75 and 1.0 m/s, so the map
  rescales rather than clipping; the clamp it replaced pinned everything above
  roughly 0.06 stick at the cap.

### Conclusion
Supported on the host. Not measured on hardware: whether the closed loop
actually settles at 1.0 m/s under RC, which is the point of the change. Next
action: drive in slow mode with `cfg::DEBUG::serial_plot` on and compare
Target_MPS against Measured_MPS.

---

## TL-006 - 2026-09-06 (bench)

### Setup
Same car and transmitter as TL-005, with `ch_manual_override` moved to index 7.

### Hypothesis
CH8 reaches the Teensy over PPM, so swD drives the manual override.

### Method
Flip swD and watch the dot matrix and the drive path; exercise every v0.7.0
feature.

### Observations
- CH8 arrives and swD drives the override. The six-channel figure for the iA6B
  counts its servo rails; the PPM stream carries the whole frame, so all eight
  slots populate. The TL-005 concern does not apply.
- All v0.7.0 features working: charge bar, slow mode on swA, manual override on
  swD, dot-matrix I/M under override.

### Conclusion
Supported. The prediction in TL-005 that CH7 and CH8 might be absent was wrong,
and the "six channels over PPM" claim is corrected across `config.h`, the
README, `docs/architecture.md`, and `docs/changelog.md`.

Not separately measured: `slow_rc_scale` against an instrumented 1.0 m/s. Slow
mode engages and caps the car, but the RC-path scalar is still the provisional
0.4 rather than a calibrated value.

---

## TL-005 - 2026-09-06 (bench)

### Setup
Car on the bench with the FS-i6 transmitter and iA6B receiver live, firmware at
the v0.7.0 branch head. Reported from the robot, not measured here.

### Hypothesis
The v0.7.0 switch scheme and the charge bar behave on hardware as they do on the
host, and swA/swD sit on CH5/CH8 as the FS-i6 aux order implies.

### Method
Flip each switch and watch the dot matrix, the LED strip, and the drive
response.

### Observations
- swA (CH5, index 4) selects slow mode; the speed cap engages.
- LED strip renders the charge bar.
- The dot matrix switches between I and M on flipping **swB**, not swD. So the
  override was bound to index 5, and index 5 carries swB.
- CH5 = swA and CH6 = swB, matching the stock FS-i6 aux assignment.

### Conclusion
The override mechanism works; only the channel binding was wrong.
`ch_manual_override` moves from index 5 (CH6, swB) to index 7 (CH8, swD).

---

## TL-004 - 2026-09-06

### Setup
Host only (no hardware attached). PlatformIO teensy platform, GCC 13 for the
native environment. Python 3, pytest 9.0.2.

### Hypothesis
The state-of-charge bar, the RC remap, and the switch scheme leave the firmware
building and the logic tests green; the SoC curve is monotonic across
7.0-8.4 V, and an absent switch channel resolves to its permissive position.

### Method
`pio run`, `pio test -e native`, `python3 -m pytest tests -q` from
`racecar-pit-pi/`. Firmware size compared against the TL-003 build.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| SoC curve span | 7000 to 8400 | mV |
| Curve breakpoints | 10 | - |
| Native test cases | 17 | - |
| Pytest cases | 13 | - |

### Observations
- teensy41 build SUCCESS. Flash code 65748 -> 66196 B (+448), data 8676 B
  unchanged, RAM1 variables 15104 B and RAM2 17568 B both unchanged.
- Switch decode: pulses of 0 and 2500 us both read down, so a lost receiver
  leaves fast mode and autonomy enabled.
- native: 17/17 passed in 0.51 s.
- pytest: 13/13 passed in 0.05 s.
- SoC is monotonic over the whole band at 10 mV steps and clamps to 0 and 100
  outside it.
- Curve vs a linear voltage map: 7600 mV reads 40% against 43%, 7400 mV reads
  13% against 29%. The gap is the point of the change.

### Conclusion
Supported on the host. Not yet run on hardware: the bar has not been watched
against a discharging pack, and the FS-iA6B channel map in
`docs/architecture.md` is derived from the code and the receiver spec, not from
a bench capture. Next action, before any wheels-down run: log CH1-CH8 over
telemetry with the transmitter on, confirm CH2 and CH4 rest near 1500 us, and
check that pushing the right stick forward drives forward and left stick left
steers left. Reverse the affected channel on the transmitter if not. Then
confirm swA and swD sit on CH5 and CH6 and that each switch's down position
reads below 1500 us, and calibrate `slow_rc_scale` against a measured 1.0 m/s.

---

## TL-003 - 2026-09-06

### Setup
Host only (no hardware attached). PlatformIO with the teensy platform, GCC 13 on
the host for the native environment. Python 3, numpy 2.4.0, pyserial 3.5,
pytest 9.0.2.

### Hypothesis
The cleanup refactor is behavior-preserving: the firmware still builds for
teensy41 and the extracted logic matches the original implementations.

### Method
`pio run`, `pio test -e native`, `python3 -m pytest tests -q` from
`racecar-pit-pi/`. Firmware flash and RAM compared against the pre-refactor
build.

### Parameters
| Name | Value | Unit |
| ---- | ----- | ---- |
| Build environments | teensy41, native | - |
| Native test cases | 10 | - |
| Pytest cases | 13 | - |

### Observations
- teensy41 build SUCCESS. Flash code 67028 -> 65748 B, data 9224 -> 8676 B,
  RAM1 variables 15136 -> 15104 B. RAM2 unchanged at 17568 B.
- native: 10/10 passed in 0.57 s.
- pytest: 13/13 passed in 0.06 s.
- `static_assert` confirms `sizeof(TXPacket) == 94` and
  `sizeof(RxPacket) == 470`, so the wire format did not move.

### Conclusion
Supported. No on-hardware run yet; TL-001 and TL-002 have not been repeated
against this build. Next action: re-run the bench check (TL-001) before the next
on-robot session.

---

## TL-002 - 2026-07-06

### Setup
Full ROS 2 stack on the Pi, car on stands (wheels up), Xbox controller through
the teleop mux. Pairs with `racecar_neo_ros2_driver` v0.5.0.

### Hypothesis
End-to-end control and telemetry work through the ROS driver, not only over a
direct serial link.

### Method
Bring the stack up, drive with the controller, watch `/imu/lsm9ds1` and the
driver's CRC and resync counters.

### Observations
- `/imu/lsm9ds1` at 151 Hz, accel z about -9.66 m/s^2.
- No CRC failures and no resync events over the session.
- Forward and reverse both actuate; steering follows the stick.
- Drive and steering polarity corrected Pi-side: `pit.yaml` `speed_sign` and
  `steering_sign` both -1.
- Dot-matrix glyph and LED strip respond to controller input.

### Conclusion
Supported.

---

## TL-001 - 2026-07 (bench)

### Setup
Wheels up, Teensy driven directly over the GPIO UART with no ROS stack.

### Hypothesis
Telemetry is framed correctly and every sensor reports real values.

### Observations
| Name | Value | Unit |
| ---- | ----- | ---- |
| Telemetry frames CRC-valid | 537/537 | frames |
| Bus voltage | 7.9 | V |
| Motor speed under Pi throttle | ~35000 | rpm |

- Accelerometer reads gravity; gyro and magnetometer return real values.
- Encoder and timestamp populate.
- Steering actuates.

### Conclusion
Supported.
