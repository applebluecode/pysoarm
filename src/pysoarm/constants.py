"""Package-wide constants.

Each value is tagged with its provenance: the STS3215 datasheet/memory table, LeRobot's
SO-101 defaults (the de-facto reference configuration for this hardware), or a measurement
on the reference arm.
"""

from __future__ import annotations

# --------------------------------------------------------------------------- #
# Bus / protocol
# --------------------------------------------------------------------------- #
#: STS3215 factory default baudrate (datasheet). LeRobot relies on the same value.
DEFAULT_BAUDRATE = 1_000_000
#: Per-transaction timeout. Measured round trip is ~0.3 ms, so 50 ms is generous.
DEFAULT_TIMEOUT_S = 0.05
#: Extra attempts after the first one for a failed read/write.
DEFAULT_RETRIES = 2
#: Bus-wide id used by the SYNC WRITE instruction.
BROADCAST_ID = 0xFE

# --------------------------------------------------------------------------- #
# Encoder and units
# --------------------------------------------------------------------------- #
TICKS_PER_TURN = 4096
RESOLUTION_MAX = TICKS_PER_TURN - 1
#: 0.088 degrees per tick, i.e. the smallest representable joint resolution.
DEG_PER_TICK = 360.0 / RESOLUTION_MAX
#: ``Present_Speed`` unit. Datasheet: 0.732 rpm per LSB -> 4.392 deg/s.
#: Not yet validated on hardware; treat as indicative only.
SPEED_LSB_DEG_PER_S = 0.732 * 360.0 / 60.0
#: ``Present_Voltage`` unit: 0.1 V per LSB.
VOLTAGE_LSB_V = 0.1

# --------------------------------------------------------------------------- #
# Motor configuration written at connect time (LeRobot's SO-101 defaults)
# --------------------------------------------------------------------------- #
DEFAULT_P_COEFFICIENT = 16
DEFAULT_I_COEFFICIENT = 0
DEFAULT_D_COEFFICIENT = 32
DEFAULT_ACCELERATION = 254
DEFAULT_MAXIMUM_ACCELERATION = 254
DEFAULT_RETURN_DELAY_TIME = 0

#: Gripper protection: half torque and half current to avoid burnout (LeRobot defaults).
GRIPPER_MAX_TORQUE_LIMIT = 500
GRIPPER_PROTECTION_CURRENT = 250
GRIPPER_OVERLOAD_TORQUE = 25

#: ``Operating_Mode`` register value for position control.
OPERATING_MODE_POSITION = 0
#: ``Phase`` bit 4 must be clear so positions stay within [0, RESOLUTION_MAX].
PHASE_KEEP_SINGLE_TURN = 0x10

# --------------------------------------------------------------------------- #
# Control loop
# --------------------------------------------------------------------------- #
#: Default control frequency: 1.2 ms per read+write step leaves ample headroom.
DEFAULT_LOOP_HZ = 200.0
#: Health registers (temperature/voltage/current) are refreshed at this interval instead of
#: every tick, so the hot path stays at one sync read per step.
DEFAULT_HEALTH_PERIOD_S = 0.5
