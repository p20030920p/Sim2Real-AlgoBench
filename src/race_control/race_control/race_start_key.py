#!/usr/bin/env python3
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class RaceStartKey(Node):
    def __init__(self):
        super().__init__('race_start_key')
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(Bool, '/race/start', qos)

    def wait_and_publish(self):
        if not sys.stdin.isatty():
            raise RuntimeError('race_start_key must run in an interactive terminal')
        print('车辆已就绪：按空格或回车一键启动，Q 退出。', flush=True)
        old = termios.tcgetattr(sys.stdin.fileno())
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                key = sys.stdin.read(1)
                if key in (' ', '\r', '\n'):
                    self.pub.publish(Bool(data=True))
                    print('\n已发送唯一启动信号，车辆进入全自主模式。', flush=True)
                    return
                if key.lower() == 'q':
                    return
        finally:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = RaceStartKey()
    try:
        node.wait_and_publish()
        rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
