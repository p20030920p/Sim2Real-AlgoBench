#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped')
        self.declare_parameter('in_topic', '/cmd_vel')
        self.declare_parameter('out_topic', '/omni_drive_controller/cmd_vel')
        self.declare_parameter('frame_id', 'base_footprint')
        self.frame_id = self.get_parameter('frame_id').value
        self.pub = self.create_publisher(TwistStamped, self.get_parameter('out_topic').value, 10)
        self.sub = self.create_subscription(Twist, self.get_parameter('in_topic').value, self.on_twist, 10)
        self.get_logger().info(f"Converting Twist {self.get_parameter('in_topic').value} -> TwistStamped {self.get_parameter('out_topic').value}")

    def on_twist(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub.publish(out)


def main():
    rclpy.init()
    node = TwistToTwistStamped()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
