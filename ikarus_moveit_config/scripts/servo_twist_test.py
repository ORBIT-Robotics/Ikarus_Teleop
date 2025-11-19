#!/usr/bin/env python3

"""Publish continuous TwistStamped commands for MoveIt Servo testing."""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class ServoTwistTest(Node):
    """Drive MoveIt Servo with a small oscillating x-axis twist."""

    def __init__(self) -> None:
        super().__init__("servo_twist_test")

        self._frame_id = (
            self.declare_parameter("frame_id", "base_link").get_parameter_value()
        ).string_value
        self._amplitude = (
            self.declare_parameter("amplitude", 0.05).get_parameter_value().double_value
        )
        self._frequency = (
            self.declare_parameter("frequency", 0.2).get_parameter_value().double_value
        )
        self._publish_rate = (
            self.declare_parameter("publish_rate", 30.0)
            .get_parameter_value()
            .double_value
        )

        self._publisher = self.create_publisher(
            TwistStamped, "/moveit_servo/delta_twist_cmds", 10
        )
        self._timer = self.create_timer(1.0 / self._publish_rate, self._on_timer)
        self._start_time: Optional[float] = None

        self.get_logger().info(
            "Publishing Twist commands on /moveit_servo/delta_twist_cmds "
            "(amp=%.3f m/s, freq=%.3f Hz, frame=%s)",
            self._amplitude,
            self._frequency,
            self._frame_id,
        )

    def _on_timer(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._start_time is None:
            self._start_time = now
        phase = 2.0 * math.pi * self._frequency * (now - self._start_time)
        velocity = self._amplitude * math.sin(phase)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.twist.linear.x = velocity
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ServoTwistTest()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
