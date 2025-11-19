#!/usr/bin/env python3

"""Bridge node that republishes JointState positions as Float64MultiArray.

Subscriptions:
- sensor_msgs/JointState on parameter 'input_topic' (default /joint_states).

Publications:
- std_msgs/Float64MultiArray on parameter 'output_topic' (default /ikarus_ik),
  ordered according to 'joint_order' (default [Shoulder1R, Shoulder2R, Shoulder3R,
  ElbowR, uaR]) so downstream clients (Isaac Sim) receive a compact joint vector.
"""

from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointStateToArray(Node):
    """Republish joint state positions in a fixed order for IK consumers."""

    def __init__(self) -> None:
        super().__init__("joint_state_to_array")

        self._input_topic = (
            self.declare_parameter("input_topic", "/joint_states").get_parameter_value()
        ).string_value
        self._output_topic = (
            self.declare_parameter("output_topic", "/ikarus_ik").get_parameter_value()
        ).string_value
        self._joint_order = (
            self.declare_parameter(
                "joint_order",
                ["Shoulder1R", "Shoulder2R", "Shoulder3R", "ElbowR", "uaR"],
            )
            .get_parameter_value()
            .string_array_value
        )
        self._warned_missing: Dict[str, bool] = {}

        self._publisher = self.create_publisher(Float64MultiArray, self._output_topic, 10)
        self._subscription = self.create_subscription(
            JointState, self._input_topic, self._callback, 10
        )

        self.get_logger().info(
            f"Publishing Float64MultiArray joint positions on '{self._output_topic}' "
            f"from '{self._input_topic}'"
        )

    def _callback(self, msg: JointState) -> None:
        if self._joint_order:
            name_to_index = {name: idx for idx, name in enumerate(msg.name)}
            values: List[float] = []
            for joint in self._joint_order:
                if joint in name_to_index and len(msg.position) > name_to_index[joint]:
                    values.append(msg.position[name_to_index[joint]])
                else:
                    if not self._warned_missing.get(joint):
                        self.get_logger().warn(
                            "Joint '%s' missing in JointState; publishing 0.0 for it",
                            joint,
                        )
                        self._warned_missing[joint] = True
                    values.append(0.0)
        else:
            values = list(msg.position)

        outgoing = Float64MultiArray()
        outgoing.data = values
        self._publisher.publish(outgoing)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateToArray()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
