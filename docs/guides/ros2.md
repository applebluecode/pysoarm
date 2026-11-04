# ROS 2

The ROS 2 integration is a **separate package**, `ros2/pysoarm_ros2`, so the device layer keeps
zero ROS dependencies (see [ADR 0001](../adr/0001-scope-device-layer.md)). It is a thin rclpy
node over [`pysoarm.bridge.ArmBridge`](../reference/api.md#pysoarm.bridge), which owns all the
logic that can be tested without ROS.

```bash
pip install "pysoarm[rustypot]"     # or plain pysoarm
colcon build --packages-select pysoarm_ros2
source install/setup.bash
ros2 run pysoarm_ros2 arm_node --ros-args \
  -p port:=/dev/so101_follower -p backend:=rustypot -p rate_hz:=50.0
```

## Interface

| Direction | Name | Type | Units |
|---|---|---|---|
| publish | `joint_states` | `sensor_msgs/JointState` | radians (REP 103) |
| subscribe | `joint_commands` | `std_msgs/Float64MultiArray` | radians, one per joint |
| service | `enable` | `std_srvs/Trigger` | holds the measured pose, enables torque |
| service | `disable` | `std_srvs/Trigger` | torque off |

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `port` | `/dev/so101_follower` | serial device |
| `model` | `so101` | arm model |
| `backend` | `scs` | `scs`, `rustypot` or `auto` |
| `calibration` | `""` | LeRobot-compatible JSON; empty reads it from the motors |
| `rate_hz` | `50.0` | state publish / read rate |
| `max_step_deg` | `5.0` | per-tick joint clipping (safety) |
| `command_timeout_s` | `0.25` | reject commands based on a state older than this |
| `frame_id` | `""` | `header.frame_id` for `joint_states` |

## Safety behaviour

* every command passes the same guards as the Python API (soft limits, per-step clipping,
  thermal/supply, watchdog);
* a **rejected command is logged and dropped** — it never takes the node down, and the arm
  keeps its previous goal;
* a command based on a state older than `command_timeout_s` is refused, so a stalled loop
  cannot act on an old reading;
* `disable` (or node shutdown) releases torque, and shutdown also closes the serial port.

## Not covered by CI

CI has no ROS installation, so the node itself is only syntax-checked and its module-level
tests skip without `rclpy`. Everything that can be tested without ROS lives in
`pysoarm.bridge` and *is* covered.
