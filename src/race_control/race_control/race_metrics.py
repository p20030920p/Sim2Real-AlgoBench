#!/usr/bin/env python3
import csv
import json
import math
import os
import time
from collections import defaultdict
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String


class RaceMetrics(Node):
    """Record precise per-run timing and autonomous-driving quality metrics."""

    def __init__(self):
        super().__init__('race_metrics')
        self.declare_parameter('report_dir', '')
        self.declare_parameter('timeout_seconds', 300.0)
        self.declare_parameter('target_wall_range', 0.34)
        self.declare_parameter('contact_debounce_seconds', 0.35)

        report_dir = str(self.get_parameter('report_dir').value).strip()
        self.report_dir = os.path.expanduser(
            report_dir or '~/桌面/project/race_ros2_ws/reports')
        os.makedirs(self.report_dir, exist_ok=True)

        latch = QoSProfile(depth=1)
        latch.reliability = ReliabilityPolicy.RELIABLE
        latch.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool, '/race/start', self.on_start, 10)
        self.create_subscription(Bool, '/race/reset', self.on_reset, 10)
        self.create_subscription(Bool, '/race/complete', self.on_complete, latch)
        self.create_subscription(String, '/race/state', self.on_state, latch)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(
            Odometry, '/omni_drive_controller/odom', self.on_odom,
            qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Float32MultiArray, '/target/green_board_observation',
            self.on_board, 10)
        self.create_subscription(
            Contacts, '/robot/contacts', self.on_contacts,
            qos_profile_sensor_data)
        for wheel in ('front_wheel', 'left_wheel', 'right_wheel'):
            self.create_subscription(
                Contacts, f'/robot/contacts/{wheel}', self.on_contacts,
                qos_profile_sensor_data)
        self.summary_pub = self.create_publisher(String, '/race/metrics_summary', latch)
        self.timer = self.create_timer(0.5, self.check_timeout)

        self.run_sequence = 0
        self.active = False
        self.current_state = 'UNKNOWN'
        self.reset_run_data()
        self.get_logger().info(f'Race metrics reports: {self.report_dir}')

    def sim_ns(self):
        return self.get_clock().now().nanoseconds

    def reset_run_data(self):
        self.start_sim_ns = None
        self.start_wall_ns = None
        self.finish_entry_sim_ns = None
        self.finish_entry_wall_ns = None
        self.complete_sim_ns = None
        self.complete_wall_ns = None
        self.state_enter_ns = None
        self.state_durations_ns = defaultdict(int)
        self.state_counts = defaultdict(int)
        self.path_length = 0.0
        self.last_odom_xy = None
        self.last_odom_process_ns = -10**18
        self.last_contact_process_ns = -10**18
        self.current_cmd = Twist()
        self.speed_error_sq_sum = 0.0
        self.speed_error_abs_sum = 0.0
        self.speed_error_max = 0.0
        self.slip_ratio_sum = 0.0
        self.slip_ratio_max = 0.0
        self.tracking_samples = 0
        self.command_speed_max = 0.0
        self.measured_speed_max = 0.0
        self.min_clearance = math.inf
        self.collision_count = 0
        self.collision_pairs = defaultdict(int)
        self.max_contact_depth = 0.0
        self.last_contact_seen_ns = -10**18
        self.first_board_sim_ns = None
        self.board_x = math.nan
        self.board_area = math.nan
        self.front_range = math.nan
        self.finalized = False

    def begin_run(self):
        self.run_sequence += 1
        self.reset_run_data()
        self.active = True
        self.start_sim_ns = self.sim_ns()
        self.start_wall_ns = time.monotonic_ns()
        self.current_state = 'WAITING_FOR_STATE'
        self.state_enter_ns = self.start_sim_ns
        self.get_logger().info(f'Metrics run {self.run_sequence} started')

    def on_start(self, msg):
        if msg.data and not self.active:
            self.begin_run()

    def on_reset(self, msg):
        if msg.data and self.active:
            self.finalize('RESET_OR_ABORTED')

    def on_complete(self, msg):
        if msg.data and self.active:
            self.finalize('COMPLETE')

    def on_state(self, msg):
        now = self.sim_ns()
        if self.active and self.state_enter_ns is not None:
            self.state_durations_ns[self.current_state] += max(0, now - self.state_enter_ns)
        self.current_state = msg.data
        self.state_enter_ns = now
        if not self.active:
            return
        self.state_counts[msg.data] += 1
        if msg.data == 'HOLD_STILL_3_SECONDS' and self.finish_entry_sim_ns is None:
            self.finish_entry_sim_ns = now
            self.finish_entry_wall_ns = time.monotonic_ns()
            self.get_logger().info(
                f'Official finish entry at {self.elapsed_sim(now):.6f} s')
        elif msg.data == 'COMPLETE':
            self.finalize('COMPLETE')

    def on_cmd(self, msg):
        self.current_cmd = msg
        speed = math.hypot(msg.linear.x, msg.linear.y)
        self.command_speed_max = max(self.command_speed_max, speed)

    def on_odom(self, msg):
        if not self.active:
            return
        now = self.sim_ns()
        # The controller publishes odometry at 100 Hz. Twenty samples per
        # simulated second preserve timing/path statistics without stealing
        # CPU from the real-time planner in a small VM.
        if now - self.last_odom_process_ns < 50_000_000:
            return
        self.last_odom_process_ns = now
        pose = msg.pose.pose.position
        xy = (pose.x, pose.y)
        if self.last_odom_xy is not None:
            step = math.hypot(xy[0] - self.last_odom_xy[0], xy[1] - self.last_odom_xy[1])
            if step < 0.50:
                self.path_length += step
        self.last_odom_xy = xy

        measured = msg.twist.twist
        command_speed = math.hypot(self.current_cmd.linear.x, self.current_cmd.linear.y)
        measured_speed = math.hypot(measured.linear.x, measured.linear.y)
        error = math.hypot(
            self.current_cmd.linear.x - measured.linear.x,
            self.current_cmd.linear.y - measured.linear.y)
        self.measured_speed_max = max(self.measured_speed_max, measured_speed)
        self.speed_error_sq_sum += error * error
        self.speed_error_abs_sum += error
        self.speed_error_max = max(self.speed_error_max, error)
        slip = error / max(command_speed, 0.05)
        self.slip_ratio_sum += slip
        self.slip_ratio_max = max(self.slip_ratio_max, slip)
        self.tracking_samples += 1

    def on_scan(self, msg):
        valid = [
            value for value in msg.ranges
            if math.isfinite(value) and msg.range_min < value < msg.range_max
        ]
        if valid and self.active:
            self.min_clearance = min(self.min_clearance, min(valid))

        front = []
        for index, value in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            if abs(angle) <= math.radians(8) and math.isfinite(value):
                if msg.range_min < value < msg.range_max:
                    front.append(value)
        if front:
            self.front_range = sorted(front)[max(0, int(0.10 * len(front)) - 1)]

    def on_board(self, msg):
        if len(msg.data) < 4:
            return
        valid = msg.data[0] > 0.5
        self.board_x = float(msg.data[1])
        self.board_area = float(msg.data[3])
        if self.active and valid and self.first_board_sim_ns is None:
            self.first_board_sim_ns = self.sim_ns()

    def on_contacts(self, msg):
        if not self.active or not msg.contacts:
            return
        now = self.sim_ns()
        if now - self.last_contact_process_ns < 50_000_000:
            return
        self.last_contact_process_ns = now
        # Wheel contact with the floor is continuous and is not a competition
        # collision. Rough / low-grip patches are floor disturbances too.
        floor_tokens = ('ground_plane', 'rough_patch', 'low_grip_patch')
        relevant = []
        for contact in msg.contacts:
            names = (contact.collision1.name, contact.collision2.name)
            if any(token in name for token in floor_tokens for name in names):
                continue
            # The imported DAE uses one collision mesh for both its floor and
            # walls. Vertical contact normals are floor support; horizontal
            # normals are genuine wall / obstacle impacts.
            if any('competition_map_dae' in name for name in names):
                if contact.normals and all(abs(normal.z) >= 0.70 for normal in contact.normals):
                    continue
            if all('omni_car' in name for name in names):
                continue
            relevant.append(contact)
        if not relevant:
            return
        debounce_ns = int(float(
            self.get_parameter('contact_debounce_seconds').value) * 1e9)
        if now - self.last_contact_seen_ns > debounce_ns:
            self.collision_count += 1
        self.last_contact_seen_ns = now
        for contact in relevant:
            names = sorted((contact.collision1.name, contact.collision2.name))
            self.collision_pairs[' <> '.join(names)] += 1
            if contact.depths:
                self.max_contact_depth = max(self.max_contact_depth, max(contact.depths))

    def elapsed_sim(self, end_ns):
        if self.start_sim_ns is None or end_ns is None:
            return math.nan
        return (end_ns - self.start_sim_ns) * 1e-9

    def elapsed_wall(self, end_ns):
        if self.start_wall_ns is None or end_ns is None:
            return math.nan
        return (end_ns - self.start_wall_ns) * 1e-9

    @staticmethod
    def finite_or_none(value):
        return value if math.isfinite(value) else None

    def check_timeout(self):
        if not self.active:
            return
        timeout = float(self.get_parameter('timeout_seconds').value)
        if self.elapsed_sim(self.sim_ns()) >= timeout:
            self.finalize('TIMEOUT')

    def finalize(self, status):
        if not self.active or self.finalized:
            return
        self.finalized = True
        now_sim = self.sim_ns()
        now_wall = time.monotonic_ns()
        self.complete_sim_ns = now_sim
        self.complete_wall_ns = now_wall
        if self.state_enter_ns is not None:
            self.state_durations_ns[self.current_state] += max(0, now_sim - self.state_enter_ns)

        official_end_sim = self.finish_entry_sim_ns or now_sim
        official_end_wall = self.finish_entry_wall_ns or now_wall
        official_sim = self.elapsed_sim(official_end_sim)
        official_wall = self.elapsed_wall(official_end_wall)
        verified_sim = self.elapsed_sim(now_sim)
        verified_wall = self.elapsed_wall(now_wall)
        samples = max(1, self.tracking_samples)
        wall_error = (
            self.front_range - float(self.get_parameter('target_wall_range').value)
            if math.isfinite(self.front_range) else math.nan)

        timestamp = datetime.now().astimezone()
        run_id = f"{timestamp.strftime('%Y%m%d_%H%M%S')}_{self.run_sequence:03d}"
        report = {
            'run_id': run_id,
            'status': status,
            'timestamp': timestamp.isoformat(),
            'timing': {
                'start_sim_ns': self.start_sim_ns,
                'finish_entry_sim_ns': self.finish_entry_sim_ns,
                'complete_sim_ns': self.complete_sim_ns,
                'official_navigation_seconds': official_sim,
                'verified_completion_seconds': verified_sim,
                'official_wall_seconds': official_wall,
                'verified_wall_seconds': verified_wall,
                'simulation_realtime_factor_observed': (
                    verified_sim / verified_wall if verified_wall > 0 else None),
                'first_green_board_seconds': (
                    self.elapsed_sim(self.first_board_sim_ns)
                    if self.first_board_sim_ns is not None else None),
            },
            'motion': {
                'path_length_m': self.path_length,
                'command_speed_max_mps': self.command_speed_max,
                'measured_speed_max_mps': self.measured_speed_max,
                'speed_tracking_mae_mps': self.speed_error_abs_sum / samples,
                'speed_tracking_rmse_mps': math.sqrt(self.speed_error_sq_sum / samples),
                'speed_tracking_max_error_mps': self.speed_error_max,
                'mean_relative_slip': self.slip_ratio_sum / samples,
                'max_relative_slip': self.slip_ratio_max,
                'tracking_samples': self.tracking_samples,
            },
            'safety': {
                'collision_events': self.collision_count,
                'collision_pairs': dict(self.collision_pairs),
                'maximum_contact_depth_m': self.max_contact_depth,
                'minimum_lidar_clearance_m': self.finite_or_none(self.min_clearance),
            },
            'finish_accuracy': {
                'final_board_horizontal_error_normalized': self.finite_or_none(self.board_x),
                'final_board_area_ratio': self.finite_or_none(self.board_area),
                'final_front_range_m': self.finite_or_none(self.front_range),
                'final_wall_range_error_m': self.finite_or_none(wall_error),
            },
            'states': {
                'entry_counts': dict(self.state_counts),
                'duration_seconds': {
                    key: value * 1e-9
                    for key, value in self.state_durations_ns.items()
                },
            },
        }

        json_path = os.path.join(self.report_dir, f'run_{run_id}.json')
        with open(json_path, 'w', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
        self.write_csv(report)
        self.write_markdown(report, os.path.join(self.report_dir, f'run_{run_id}.md'))

        summary = (
            f"{status}: official={official_sim:.6f}s, verified={verified_sim:.6f}s, "
            f"collisions={self.collision_count}, path={self.path_length:.3f}m")
        self.summary_pub.publish(String(data=summary))
        self.get_logger().info(f'{summary}; report={json_path}')
        self.active = False

    def write_csv(self, report):
        timing = report['timing']
        motion = report['motion']
        safety = report['safety']
        finish = report['finish_accuracy']
        row = {
            'run_id': report['run_id'],
            'timestamp': report['timestamp'],
            'status': report['status'],
            'official_navigation_seconds': timing['official_navigation_seconds'],
            'verified_completion_seconds': timing['verified_completion_seconds'],
            'official_wall_seconds': timing['official_wall_seconds'],
            'observed_realtime_factor': timing['simulation_realtime_factor_observed'],
            'first_green_board_seconds': timing['first_green_board_seconds'],
            'path_length_m': motion['path_length_m'],
            'command_speed_max_mps': motion['command_speed_max_mps'],
            'measured_speed_max_mps': motion['measured_speed_max_mps'],
            'speed_tracking_rmse_mps': motion['speed_tracking_rmse_mps'],
            'mean_relative_slip': motion['mean_relative_slip'],
            'collision_events': safety['collision_events'],
            'maximum_contact_depth_m': safety['maximum_contact_depth_m'],
            'minimum_lidar_clearance_m': safety['minimum_lidar_clearance_m'],
            'final_board_error': finish['final_board_horizontal_error_normalized'],
            'final_wall_range_error_m': finish['final_wall_range_error_m'],
            'avoid_entries': report['states']['entry_counts'].get('YIELD_OR_BYPASS_OBSTACLE', 0),
            'dead_end_entries': report['states']['entry_counts'].get('DEAD_END_TURN_180', 0),
            'recovery_entries': report['states']['entry_counts'].get('TRACTION_RECOVERY', 0),
        }
        path = os.path.join(self.report_dir, 'race_summary.csv')
        new_file = not os.path.exists(path) or os.path.getsize(path) == 0
        with open(path, 'a', newline='', encoding='utf-8-sig') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(row.keys()))
            if new_file:
                writer.writeheader()
            writer.writerow(row)

    @staticmethod
    def write_markdown(report, path):
        timing = report['timing']
        motion = report['motion']
        safety = report['safety']
        finish = report['finish_accuracy']
        lines = [
            f"# 比赛运行报告 {report['run_id']}", '',
            f"- 结果：`{report['status']}`",
            f"- 正式导航时间（不含静止3秒）：`{timing['official_navigation_seconds']:.6f} s`",
            f"- 完整验证时间：`{timing['verified_completion_seconds']:.6f} s`",
            f"- 现实墙钟时间：`{timing['verified_wall_seconds']:.6f} s`",
            f"- 路径长度：`{motion['path_length_m']:.3f} m`",
            f"- 碰撞次数：`{safety['collision_events']}`",
            f"- 最小雷达净空：`{safety['minimum_lidar_clearance_m']}`",
            f"- 速度跟踪RMSE：`{motion['speed_tracking_rmse_mps']:.4f} m/s`",
            f"- 平均相对滑移：`{motion['mean_relative_slip']:.4f}`",
            f"- 最终绿板水平偏差：`{finish['final_board_horizontal_error_normalized']}`",
            f"- 最终墙距偏差：`{finish['final_wall_range_error_m']} m`",
            '', '## 状态耗时', '', '| 状态 | 秒 |', '|---|---:|',
        ]
        for state, seconds in report['states']['duration_seconds'].items():
            lines.append(f'| {state} | {seconds:.6f} |')
        with open(path, 'w', encoding='utf-8') as stream:
            stream.write('\n'.join(lines) + '\n')


def main():
    rclpy.init()
    node = RaceMetrics()
    try:
        rclpy.spin(node)
    finally:
        if node.active:
            node.finalize('INTERRUPTED')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
