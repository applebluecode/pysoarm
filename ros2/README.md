# pysoarm_ros2

A thin ROS 2 (rclpy) node over [`pysoarm`](../README.md). It is a **separate package** on
purpose (see [ADR 0001](../docs/adr/0001-scope-device-layer.md)): the device layer stays
dependency-free, and ROS users opt in.

Requires a sourced ROS 2 installation (tested against the current LTS) and `pysoarm` on the
same Python environment.

## Build

```bash
# from the workspace containing this directory
pip install "pysoarm[rustypot]"        # or plain pysoarm
colcon build --packages-select pysoarm_ros2
source install/setup.bash
```

## Run

```bash
ros2 run pysoarm_ros2 arm_node --ros-args \
  -p port:=/dev/so101_follower \
  -p model:=so101 \
  -p backend:=rustypot \
  -p calibration:=/path/to/my_follower_arm.json \
  -p rate_hz:=50.0 \
  -p max_step_deg:=5.0 \
  -p command_timeout_s:=0.25
```

### Interface

| Direction | Name | Type | Units / notes |
|---|---|---|---|
| publish | `joint_states` | `sensor_msgs/JointState` | radians, model joint order |
| subscribe | `joint_commands` | `std_msgs/Float64MultiArray` | radians, one value per joint |
| service | `enable` | `std_srvs/Trigger` | reads the pose, holds it, enables torque |
| service | `disable` | `std_srvs/Trigger` | torque off (arm goes limp) |

Every command goes through the same guards as the Python API: per-step clipping
(`max_step_deg`), soft limits, thermal/supply guards and the communication watchdog. A command
is rejected (logged, node keeps running) when the latest state is older than
`command_timeout_s` — a stalled loop must not act on an old reading.

```bash
# move the joints (radians)
ros2 topic pub --once /joint_commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, -0.5, 1.0, 0.0, 1.5, 0.7]}"
ros2 service call /disable std_srvs/srv/Trigger
```

## Tests

```bash
colcon test --packages-select pysoarm_ros2
```

The node needs hardware to construct, so the tests only assert that the module imports and the
documented parameters exist; the logic that can be tested without ROS lives in
`pysoarm.bridge.ArmBridge` and is covered by the main package's test suite.
