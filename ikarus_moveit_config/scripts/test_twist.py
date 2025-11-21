#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TwistTest(Node):
    def __init__(self):
        super().__init__('twist_test')
        # Must match servo_node subscriber
        self.pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        # 50 Hz = 0.02 seconds
        self.timer = self.create_timer(0.02, self.timer_cb)

    def timer_cb(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'   # your planning_frame / robot_link_command_frame

        # rotate around z at 0.5 rad/s, no linear motion
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 1.0

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TwistTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
