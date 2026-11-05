# Hardware notes

Everything below was measured on a real SO-101 follower (six STS3215 servos, CH343 USB-CDC
adapter, 1 Mbaud, 12 V supply).

## Capability matrix

| | Franka (libfranka) | **SO-101 (STS3215)** |
|---|---|---|
| Loop rate | 1 kHz, hard real-time | 200–500 Hz, soft real-time (USB CDC) |
| Command | torque τ, q/dq, Cartesian impedance | **Goal_Position** (internal PID); velocity / pwm / step modes available |
| Feedback | q, dq, τ, external wrench, pose | q (4096 ticks/turn), speed, current/load estimate, voltage, temperature |
| Force control | yes | **no** — coarse stall/collision detection from current only |
| Kinematics | bundled model | bring your own URDF |
| Safety | hard limits, watchdog | you implement it (`safety.py`) |

## Measured timings

Measured on the reference arm, both backends:

| Operation | `scs` (pure Python) | `rustypot` (optional extra) |
|---|---|---|
| single register read | 0.36 ms | — |
| `sync_read`, 6 joints | 2.18 ms | **0.98 ms** |
| `sync_write`, 6 goals | 0.26 ms | 0.26 ms |
| **read + write (one control step)** | **2.79 ms** | **1.44 ms** |
| 200 Hz loop, 1 s | 200 ticks, 3–6 overruns | **200 ticks, 0 overruns** |

`scs` is the default: one small dependency (`pyserial`), no compiled extension. Install
`pysoarm[rustypot]` and pass `backend="rustypot"` (or `--backend rustypot`) to halve the read
cost — the only real performance lever on this hardware.

## Wire protocol

```
request   FF FF  ID  LEN  INSTR  [PARAMS...]  CHECKSUM
response  FF FF  ID  LEN  ERROR  [DATA...]    CHECKSUM
```

* `LEN = len(PARAMS) + 2` — a two-byte READ therefore uses `LEN = 4`;
* `CHECKSUM = ~(ID + LEN + INSTR + PARAMS...) & 0xFF`;
* broadcast id `0xFE`; `SYNC_WRITE = 0x83`;
* positions are **sign-magnitude** (bit 15), not two's complement;
* the bus is **half-duplex**: pipelining six READ requests before reading any reply corrupts
  the stream (observed on hardware), so reads are sequential. `SYNC_WRITE` is safe to
  broadcast because the host is the only transmitter.

## Registers used

| Register | Addr | Size | Encoding |
|---|---|---|---|
| `Torque_Enable` | 40 | 1 | unsigned |
| `Goal_Position` | 42 | 2 | sign-magnitude (15) |
| `Present_Position` | 56 | 2 | sign-magnitude (15) |
| `Present_Speed` | 58 | 2 | sign-magnitude (15) |
| `Present_Load` | 60 | 2 | sign-magnitude (10) |
| `Present_Voltage` | 62 | 1 | 0.1 V/LSB |
| `Present_Temperature` | 63 | 1 | °C |
| `Present_Current` | 69 | 2 | unsigned |
| `Operating_Mode` | 33 | 1 | 0 position / 1 velocity / 2 pwm / 3 step |
| `Homing_Offset` | 31 | 2 | sign-magnitude (11) |
| `Min/Max_Position_Limit` | 9 / 11 | 2 | unsigned |
| `P/I/D_Coefficient` | 21 / 23 / 22 | 1 | unsigned |
| `Acceleration` / `Maximum_Acceleration` | 41 / 85 | 1 | unsigned |
| `Goal_Speed` | 46 | 2 | sign-magnitude (15); `0` = unlimited |
| `Return_Delay_Time` | 7 | 1 | unsigned |
| `Phase` | 18 | 1 | unsigned (bit 4 must be clear) |
| `Max_Torque_Limit` / `Protection_Current` / `Overload_Torque` | 16 / 28 / 36 | 2/2/1 | gripper caps |

## Homing and calibration offsets

The STS3215 is an **absolute** single-turn encoder: it knows its tick position (0–4095) as soon
as it is powered, so there is no “spin until the index is found” homing move. What is recorded
is the relation between a tick and the joint's physical zero:

```text
homing_offset = present_position - 2047
```

After that offset is written, the servo reports `position - homing_offset`, i.e. Feetech
subtracts the offset. The value therefore decides **which physical pose is 0°**. The 11-bit
sign-magnitude `Homing_Offset` register cannot represent `2048`, so `2047` is the canonical
half-turn value; a servo sitting at 4095 is clamped to `+2047`.

The conversion only makes sense together with the recorded travel (`Min/Max_Position_Limit`):
`homing_offset` fixes the zero, while `range_min`/`range_max` keep commands inside the
mechanical travel. Both are written by `calibrate()`.

### Why `calibrate()` asks for mid-travel

A joint only turns through part of the encoder's 360° range, so the operator must choose which
pose should correspond to tick 2047:

| Convention | Pose used for `homing_offset` | Result |
|---|---|---|
| LeRobot / pysoarm ≥ 0.1.3 (`middle=True`) | every joint held at **mid-travel** | 2047 is the mechanical middle; travel is symmetric and `q_deg` matches LeRobot |
| pre-0.1.3 (`middle=False`, `--no-middle`) | whatever pose the arm happened to be in | arbitrary `q_deg` zero, possibly asymmetric or touching 0/4095 |

The two conventions must not be mixed between arms that are teleoperated together: the same
physical pose would map to two different `q_deg` values. `wait_middle` is the injection point
for a console, a fixture or a headless test; it only blocks and never reads stdin.

`Gripper.homing()` is a different operation: it redefines the closed end of the gripper travel
but does not change `homing_offset`, so the gripper's zero stays where it was.

## Precision, deadband and drift

The STS3215 is a 12-bit single-turn servo with an internal position PID, so the software sees
three limits that are easy to mistake for bugs:

| Quantity | Value on the reference arm | Consequence |
|---|---|---|
| Encoder resolution | `360 / 4095 = 0.0879°` per tick | sub-degree goals are quantised |
| Positioning deadband | residual **0.22–0.84°** after a 5°/tick convergence | do not chase sub-degree errors |
| Minimum useful step | about **1°** | tolerances should be ≥ 1° |
| Servo following lag | 3–4.6° at ±15°/0.5 Hz | use `set_motor_config()` (P gain, acceleration) if it matters |

A very small `max_step_deg` (e.g. `0.5`) is counter-productive: each step lands inside the
deadband and the joint moves only ~0.18° in 0.3 s. By default pysoarm applies **no software
speed/step cap** (matching LeRobot's `max_relative_target=None`), so the servo's own
~270°/s no-load ceiling governs normal following. If you want an explicit safety ceiling,
use `SafetyLimits(max_speed_deg_s=720)` (above the hardware ceiling, so it only clips jump
commands) or a smaller value to deliberately slow the arm; `SafetyLimits.min_step_deg` can
suppress commands below a chosen deadband.

A **leader is limp by design**, so it drifts under gravity between sessions (observed:
`wrist_roll` moving from −104° to −87°) and the follower faithfully follows. Hold or re-zero
the leader when absolute repeatability matters; this is not a calibration bug.

## Supply voltage

Follower and leader rails are different and the default threshold follows the role:

| Role | Typical bus voltage | `SafetyLimits.voltage_min_v` default |
|---|---|---|
| follower | 11.8–12.1 V | `10.0 V` |
| leader | 4.7–4.9 V (5 V rail) | `4.0 V` |

`SOArm(role="leader")` (with `limits=None`) and `pysoarm doctor --role leader` use the leader
threshold, so a healthy 4.8 V leader is no longer reported as undervoltage. At 4.7 V the
servos can still drop the occasional reply; the bus retry and `connect()` backoff exist for
that case, not to mask a weak supply.

## Known traps

* **Exclusive port.** Two processes on one tty → `Device or resource busy`. `stop()` a
  LeRobot run properly with Ctrl+C; a suspended (`Ctrl+Z`) process keeps the port.
* **`ModemManager`** probes `/dev/ttyACM*` on hotplug and can win the race.
  `sudo systemctl disable --now ModemManager`.
* **tty numbering is not stable** — use the packaged udev rule to get `/dev/so101_follower`.
* **Single-turn encoders**: `wrist_roll` wraps at 0/4095; there is no multi-turn count.
* **Torque off means limp**: read positions before enabling torque, and always command the
  measured position first.
* **Thermals**: sustained motion pushes the servos past 60 °C and the firmware derates.
