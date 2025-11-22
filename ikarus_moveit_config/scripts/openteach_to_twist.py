#!/usr/bin/env python3

"""Bridge /ikarus/pose Float64MultiArray pose into MoveIt Servo Twist commands.

Subscriptions:
- std_msgs/Float64MultiArray on parameter 'input_topic' (default /ikarus/pose),
  expected order: x, z, y, qx, qy, qz, qw.

Publications:
- geometry_msgs/TwistStamped on parameter 'output_topic'
  (default /servo_node/delta_twist_cmds) in 'frame_id' (default base_link).
"""

import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class OpenTeachToTwist(Node):
    """Convert OpenTeach pose stream into TwistStamped velocity commands."""

    def __init__(self) -> None:
        super().__init__("openteach_to_twist")

        self._input_topic = (
            self.declare_parameter("input_topic", "/ikarus/pose").get_parameter_value()
        ).string_value
        self._output_topic = (
            self.declare_parameter(
                "output_topic", "/servo_node/delta_twist_cmds"
            ).get_parameter_value()
        ).string_value
        self._frame_id = (
            self.declare_parameter("frame_id", "base_link").get_parameter_value()
        ).string_value
        self._linear_gain = (
            self.declare_parameter("linear_gain", 1.0).get_parameter_value()
        ).double_value
        self._angular_gain = (
            self.declare_parameter("angular_gain", 1.0).get_parameter_value()
        ).double_value

        self._publisher = self.create_publisher(TwistStamped, self._output_topic, 10)
        self._subscription = self.create_subscription(
            Float64MultiArray, self._input_topic, self._callback, 10
        )

        self._prev_pos: Optional[Tuple[float, float, float]] = None
        self._prev_quat: Optional[Tuple[float, float, float, float]] = None
        self._prev_time = None
        self._warned_short_msg = False

        self.get_logger().info(
            f"Subscribing to '{self._input_topic}', publishing TwistStamped to "
            f"'{self._output_topic}' in frame '{self._frame_id}'"
        )

    def _callback(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 7:
            if not self._warned_short_msg:
                self.get_logger().warn(
                    f"Expected at least 7 elements (x, z, y, qx, qy, qz, qw) but received "
                    f"{len(data)}; ignoring message"
                )
                self._warned_short_msg = True
            return

        x, z, y = data[0], data[1], data[2]
        qx, qy, qz, qw = data[3:7]

        pos = (float(x), float(y), float(z))
        quat = self._normalize_quat((float(qx), float(qy), float(qz), float(qw)))

        now = self.get_clock().now()
        if self._prev_time is None:
            self._prev_pos = pos
            self._prev_quat = quat
            self._prev_time = now
            self._publish_twist(now, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            return

        dt = (now - self._prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        linear = tuple(
            (cur - prev) / dt * self._linear_gain for cur, prev in zip(pos, self._prev_pos)
        )
        angular = tuple(
            val * self._angular_gain
            for val in self._angular_velocity(self._prev_quat, quat, dt)
        )

        self._prev_pos = pos
        self._prev_quat = quat
        self._prev_time = now

        self._publish_twist(now, linear, angular)

    def _angular_velocity(
        self, prev: Tuple[float, float, float, float], cur: Tuple[float, float, float, float], dt: float
    ) -> Tuple[float, float, float]:
        inv_prev = self._quat_inverse(prev)
        delta = self._quat_multiply(cur, inv_prev)
        delta = self._normalize_quat(delta)

        clamped_w = max(min(delta[3], 1.0), -1.0)
        angle = 2.0 * math.acos(clamped_w)
        if angle < 1e-6 or dt <= 0.0:
            return (0.0, 0.0, 0.0)

        sin_half = math.sin(angle / 2.0)
        if abs(sin_half) < 1e-6:
            axis = (0.0, 0.0, 0.0)
        else:
            axis = (delta[0] / sin_half, delta[1] / sin_half, delta[2] / sin_half)

        return tuple(axis_i * angle / dt for axis_i in axis)

    @staticmethod
    def _quat_multiply(
        q1: Tuple[float, float, float, float], q2: Tuple[float, float, float, float]
    ) -> Tuple[float, float, float, float]:
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        )

    @staticmethod
    def _quat_inverse(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        x, y, z, w = q
        norm = x * x + y * y + z * z + w * w
        if norm == 0.0:
            return (0.0, 0.0, 0.0, 1.0)
        return (-x / norm, -y / norm, -z / norm, w / norm)

    @staticmethod
    def _normalize_quat(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        x, y, z, w = q
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return (0.0, 0.0, 0.0, 1.0)
        return (x / norm, y / norm, z / norm, w / norm)

    def _publish_twist(
        self,
        stamp,
        linear: Tuple[float, float, float],
        angular: Tuple[float, float, float],
    ) -> None:
        msg = TwistStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self._frame_id
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = linear
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = angular
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenTeachToTwist()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
