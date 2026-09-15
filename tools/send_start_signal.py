#!/usr/bin/env python3
# Copyright (c) 2026 p20030920p and zfyyyyy
# SPDX-License-Identifier: MIT
"""Publish the one-button start signal for the race state machine.

This exists as a node rather than a `ros2 topic pub` one-liner because the
signal is easy to publish into nothing: get the QoS wrong and the message is
simply dropped, which looks exactly like the state machine ignoring the button.

``map_search_autonomy`` subscribes to ``/race/start`` with the rclpy default
profile (RELIABLE, VOLATILE, depth 10). A publisher offering TRANSIENT_LOCAL
does not match that, so this node stays on the default profile and only waits
for a subscriber to be present before sending.

Usage:
    python3 tools/send_start_signal.py
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

TOPIC = '/race/start'


def main() -> int:
    rclpy.init()
    node = Node('race_start_signal')
    # Default profile on purpose: see the module docstring.
    publisher = node.create_publisher(Bool, TOPIC, 10)

    # Wait for the autonomy node to actually be subscribed before publishing.
    deadline = time.time() + 30.0
    while publisher.get_subscription_count() < 1 and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    matched = publisher.get_subscription_count()
    print(f'{TOPIC}: matched {matched} subscriber(s)', flush=True)
    if matched < 1:
        print('no subscriber for the start signal; is the race stack running?',
              file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    for _ in range(5):
        publisher.publish(Bool(data=True))
        rclpy.spin_once(node, timeout_sec=0.2)
        time.sleep(0.3)
    print('start signal published', flush=True)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
