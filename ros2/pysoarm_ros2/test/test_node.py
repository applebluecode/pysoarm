"""``pysoarm_ros2`` node: publish joint states, accept joint commands, enable/disable.

Units at the ROS boundary: **radians** for ``sensor_msgs/JointState`` (REP 103) and **radians**
for the ``joint_commands`` topic. pysoarm's own API works in degrees; the conversion happens
here so the device layer stays in the units its calibration is expressed in.

All safety logic lives in ``pysoarm`` (per-step clipping, soft limits, thermal/supply guards,
communication watchdog) plus one bridge rule: a command is refused when the state it would be
applied to is older than ``command_timeout_s``. This node never bypasses those checks.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from pysoarm import ArmBridge, PysoarmError, SafetyLimits, SOArm


class SOArmNode(Node):
    """One arm per node instance."""

    def __init__(self) -> None:
        super().__init__("pysoarm")

        port = self.declare_parameter("port", "/dev/so101_follower").value
        model = self.declare_parameter("model", "so101").value
        backend = self.declare_parameter("backend", "scs").value
        calibration = self.declare_parameter("calibration", "").value
        rate_hz = float(self.declare_parameter("rate_hz", 50.0).value)
        max_step_deg = float(self.declare_parameter("max_step_deg", 5.0).value)
        command_timeout_s = float(self.declare_parameter("command_timeout_s", 0.25).value)
        self._frame_id = self.declare_parameter("frame_id", "").value

        if rate_hz <= 0.0:
            raise ValueError(f"rate_hz must be positive, got {rate_hz}")

        self._arm = SOArm(
            str(port),
            model=str(model),
            backend=str(backend),
            calibration=str(calibration) or None,
            limits=SafetyLimits(max_step_deg=max_step_deg),
        )
        self._arm.connect()
        self._bridge = ArmBridge(self._arm, max_command_age_s=command_timeout_s)

        self._state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(Float64MultiArray, "joint_commands", self._on_command, 10)
        self.create_service(Trigger, "enable", self._on_enable)
        self.create_service(Trigger, "disable", self._on_disable)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(
            f"pysoarm ready: port={port} model={model} backend={self._arm.backend} "
            f"joints={list(self._arm.model.joints)} rate={rate_hz:.1f} Hz"
        )

    # -- callbacks --------------------------------------------------------- #
    def _on_timer(self) -> None:
        try:
            frame = self._bridge.read(health=None)
        except PysoarmError as exc:
            self.get_logger().error(f"read failed: {exc}")
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        if self._frame_id:
            message.header.frame_id = str(self._frame_id)
        message.name = list(frame.names)
        message.position = list(frame.positions_rad)
        message.velocity = [math.radians(value) for value in frame.velocities_deg_s]
        self._state_pub.publish(message)

    def _on_command(self, message: Float64MultiArray) -> None:
        """Apply a joint command (radians); rejected commands are logged, not fatal."""
        try:
            sent = self._bridge.command_from_radians(message.data)
        except PysoarmError as exc:
            # A rejected command must not take the node down; the arm keeps its previous goal.
            self.get_logger().warn(f"command rejected: {exc}")
            return
        degrees = [round(math.degrees(value), 2) for value in sent]
        self.get_logger().debug(f"command accepted: {degrees} deg")

    def _on_enable(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            self._bridge.enable()
        except PysoarmError as exc:
            response.success = False
            response.message = f"enable failed: {exc}"
        else:
            response.success = True
            response.message = "torque enabled, holding the measured pose"
        return response

    def _on_disable(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        try:
            self._bridge.disable()
        except PysoarmError as exc:
            response.success = False
            response.message = f"disable failed: {exc}"
        else:
            response.success = True
            response.message = "torque disabled"
        return response

    # -- lifecycle --------------------------------------------------------- #
    def destroy_node(self) -> bool:
        """Release torque and the serial port before the node goes away."""
        try:
            self._arm.stop()
        finally:
            destroyed = super().destroy_node()
        return destroyed


def main(argv: list[str] | None = None) -> None:
    """Run the node until shutdown."""
    rclpy.init(args=argv)
    node: SOArmNode | None = None
    try:
        node = SOArmNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
