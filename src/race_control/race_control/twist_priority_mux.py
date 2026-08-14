#!/usr/bin/env python3
"""Small deterministic command mux for Nav2 and final visual servoing."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


class TwistPriorityMux(Node):
    def __init__(self):
        super().__init__('twist_priority_mux')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('timeout', 0.35)
        self.direct = True
        self.nav_cmd = Twist()
        self.final_cmd = Twist()
        self.nav_stamp = -1.0
        self.final_stamp = -1.0
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_nav, 10)
        self.create_subscription(Twist, '/cmd_vel_final', self.on_final, 10)
        self.create_subscription(Bool, '/race/direct_control', self.on_select, 10)
        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self.tick)
        self.get_logger().info('Twist mux ready: Nav2=/cmd_vel_nav, final=/cmd_vel_final.')

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_nav(self, msg):
        self.nav_cmd = msg
        self.nav_stamp = self.seconds()

    def on_final(self, msg):
        self.final_cmd = msg
        self.final_stamp = self.seconds()

    def on_select(self, msg):
        self.direct = msg.data

    def tick(self):
        stamp = self.final_stamp if self.direct else self.nav_stamp
        cmd = self.final_cmd if self.direct else self.nav_cmd
        if self.seconds() - stamp > self.get_parameter('timeout').value:
            cmd = Twist()
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TwistPriorityMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
