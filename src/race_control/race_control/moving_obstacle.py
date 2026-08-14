#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64


class MovingObstacle(Node):
    """Command a physical prismatic joint on a smooth periodic path."""

    def __init__(self):
        super().__init__('moving_obstacle')
        self.declare_parameter('command_topic', '/dynamic_obstacle/cmd_pos')
        self.declare_parameter('amplitude', 1.6)
        self.declare_parameter('period_seconds', 8.0)
        self.start_time = self.get_clock().now()
        self.command_pub = self.create_publisher(
            Float64, str(self.get_parameter('command_topic').value), 10)
        self.active_pub = self.create_publisher(Bool, '/stress/moving_obstacle_active', 10)
        self.timer = self.create_timer(0.10, self.step)
        self.get_logger().info(
            f"Physical moving-obstacle controller started on "
            f"{self.get_parameter('command_topic').value}")

    def step(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        period = max(0.5, float(self.get_parameter('period_seconds').value))
        phase = 2.0 * math.pi * elapsed / period
        position = float(self.get_parameter('amplitude').value) * math.sin(phase)
        self.command_pub.publish(Float64(data=position))
        self.active_pub.publish(Bool(data=True))


def main():
    rclpy.init()
    node = MovingObstacle()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
