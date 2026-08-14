#!/usr/bin/env python3
import math
from enum import Enum

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String


class State(Enum):
    WAITING = 'WAITING_FOR_START'
    EXPLORE = 'EXPLORE_AND_SEARCH'
    ALIGN = 'ALIGN_WITH_GREEN_BOARD'
    APPROACH = 'APPROACH_FINISH_PAD'
    AVOID = 'YIELD_OR_BYPASS_OBSTACLE'
    DEAD_END_TURN = 'DEAD_END_TURN_180'
    DEAD_END_EXIT = 'DEAD_END_EXIT_STRAIGHT'
    RECOVERY = 'TRACTION_RECOVERY'
    HOLD = 'HOLD_STILL_3_SECONDS'
    COMPLETE = 'COMPLETE'


class FinishAutonomy(Node):
    """One-button, coordinate-free finish task using a camera and 2D lidar."""

    def __init__(self):
        super().__init__('finish_autonomy')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('explore_speed', 0.50)
        self.declare_parameter('open_space_speed_max', 0.80)
        self.declare_parameter('adaptive_braking_accel', 1.20)
        self.declare_parameter('adaptive_clearance_margin', 0.42)
        self.declare_parameter('turn_speed_reduction', 0.55)
        self.declare_parameter('search_turn_speed', 0.80)
        self.declare_parameter('obstacle_distance', 0.75)
        self.declare_parameter('side_target_distance', 0.65)
        self.declare_parameter('board_lost_timeout', 0.8)
        self.declare_parameter('center_tolerance', 0.08)
        self.declare_parameter('center_kp', 1.4)
        self.declare_parameter('max_turn_speed', 1.00)
        self.declare_parameter('approach_speed_max', 0.50)
        self.declare_parameter('approach_speed_min', 0.06)
        self.declare_parameter('approach_speed_gain', 0.90)
        self.declare_parameter('approach_realign_threshold', 0.30)
        self.declare_parameter('align_forward_speed', 0.25)
        self.declare_parameter('align_drive_error_limit', 0.45)
        self.declare_parameter('approach_sidestep_speed', 0.30)
        self.declare_parameter('wall_follow_turn_limit', 0.55)
        self.declare_parameter('wall_search_turn_bias', 0.20)
        self.declare_parameter('coverage_cell_size', 0.60)
        self.declare_parameter('coverage_revisit_threshold', 4)
        self.declare_parameter('coverage_sample_period', 0.80)
        self.declare_parameter('coverage_scan_speed', 1.10)
        self.declare_parameter('coverage_scan_cooldown', 12.0)
        self.declare_parameter('avoid_sidestep_speed', 0.28)
        self.declare_parameter('avoid_turn_speed', 0.55)
        self.declare_parameter('recovery_reverse_speed', 0.32)
        self.declare_parameter('recovery_sidestep_speed', 0.32)
        self.declare_parameter('recovery_turn_speed', 0.70)
        self.declare_parameter('dead_end_scan_distance', 1.20)
        self.declare_parameter('dead_end_blocked_fraction', 0.30)
        self.declare_parameter('dead_end_turn_speed', 0.90)
        self.declare_parameter('dead_end_turn_tolerance', 0.12)
        self.declare_parameter('dead_end_exit_speed', 0.50)
        self.declare_parameter('dead_end_exit_distance', 1.00)
        self.declare_parameter('dead_end_cooldown_seconds', 3.0)
        # Lidar is 0.06 m ahead of base center. A 0.34 m scan means the
        # robot center is about 0.40 m from the wall: center of the 0.8 m pad.
        self.declare_parameter('target_wall_range', 0.34)
        self.declare_parameter('wall_range_tolerance', 0.035)
        self.declare_parameter('final_min_area_ratio', 0.025)
        self.declare_parameter('hold_seconds', 3.0)
        self.declare_parameter('emergency_stop_distance', 0.42)
        self.declare_parameter('ttc_stop_seconds', 1.4)
        self.declare_parameter('obstacle_wait_seconds', 0.35)
        self.declare_parameter('obstacle_clear_seconds', 0.35)
        self.declare_parameter('max_linear_accel', 1.00)
        self.declare_parameter('max_angular_accel', 2.50)
        self.declare_parameter('stuck_command_speed', 0.10)
        self.declare_parameter('stuck_measured_speed', 0.025)
        self.declare_parameter('stuck_timeout', 1.4)
        self.declare_parameter('scan_motion_window', 0.8)
        self.declare_parameter('scan_motion_min_change', 0.025)

        latch = QoSProfile(depth=1)
        latch.reliability = ReliabilityPolicy.RELIABLE
        latch.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, '/race/state', latch)
        self.complete_pub = self.create_publisher(Bool, '/race/complete', latch)
        # Volatile subscription accepts a physical-button node, ros2 topic pub,
        # or the supplied keyboard node without requiring special QoS flags.
        self.create_subscription(Bool, '/race/start', self.on_start, 10)
        self.create_subscription(Bool, '/race/reset', self.on_reset, 10)
        self.create_subscription(
            Float32MultiArray,
            '/target/green_board_observation',
            self.on_board,
            10,
        )
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/omni_drive_controller/odom', self.on_odom, qos_profile_sensor_data)

        self.state = State.WAITING
        self.scan = None
        self.board_valid = False
        self.board_x = 0.0
        self.board_area = 0.0
        self.last_board_time = None
        self.hold_start = None
        self.last_turn_direction = 1.0
        self.measured_speed = 0.0
        self.have_odom = False
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.last_cmd = Twist()
        self.previous_front = math.inf
        self.closing_speed = 0.0
        self.previous_scan_time = None
        self.avoid_start = None
        self.clear_start = None
        self.resume_state = State.EXPLORE
        self.stuck_start = None
        self.recovery_start = None
        self.scan_motion_reference = None
        self.scan_motion_reference_time = None
        self.scan_change_score = math.inf
        self.scan_change_time = None
        self.dead_end_turn_direction = 1.0
        self.dead_end_last_yaw = None
        self.dead_end_accumulated_yaw = 0.0
        self.dead_end_turn_start = None
        self.dead_end_exit_x = 0.0
        self.dead_end_exit_y = 0.0
        self.dead_end_exit_start = None
        self.dead_end_cooldown_until = -math.inf
        self.visited_cells = {}
        self.last_coverage_sample = -math.inf
        self.coverage_spin_until = -math.inf
        self.last_coverage_spin = -math.inf
        self.coverage_spin_direction = 1.0
        self.follow_right_wall = True
        self.publish_state()
        self.complete_pub.publish(Bool(data=False))
        period = 1.0 / float(self.get_parameter('control_rate_hz').value)
        self.timer = self.create_timer(period, self.control_step)
        self.get_logger().info('Autonomy ready. Publish /race/start=true or run race_start_key.')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def transition(self, state):
        if state == self.state:
            return
        self.get_logger().info(f'State: {self.state.value} -> {state.value}')
        self.state = state
        self.publish_state()

    def publish_state(self):
        self.state_pub.publish(String(data=self.state.value))

    def on_start(self, msg):
        if msg.data and self.state in (State.WAITING, State.COMPLETE):
            self.hold_start = None
            self.visited_cells.clear()
            self.last_coverage_sample = self.now_seconds()
            self.coverage_spin_until = -math.inf
            self.last_coverage_spin = -math.inf
            self.follow_right_wall = True
            self.complete_pub.publish(Bool(data=False))
            self.transition(State.EXPLORE)

    def on_reset(self, msg):
        if msg.data:
            self.stop()
            self.hold_start = None
            self.transition(State.WAITING)
            self.complete_pub.publish(Bool(data=False))

    def on_board(self, msg):
        if len(msg.data) < 4:
            return
        self.board_valid = msg.data[0] > 0.5
        self.board_x = float(msg.data[1])
        self.board_area = float(msg.data[3])
        if self.board_valid:
            self.last_board_time = self.now_seconds()

    def on_scan(self, msg):
        self.scan = msg
        now = self.now_seconds()
        front = self.front_range()
        if self.previous_scan_time is not None and math.isfinite(front) and math.isfinite(self.previous_front):
            dt = now - self.previous_scan_time
            if dt > 0.001:
                raw_closing = (self.previous_front - front) / dt
                # Low-pass filtering rejects one-frame lidar jumps.
                self.closing_speed = 0.7 * self.closing_speed + 0.3 * max(0.0, raw_closing)
        self.previous_front = front
        self.previous_scan_time = now

        signature = np.asarray(msg.ranges[::10], dtype=np.float64)
        signature[~np.isfinite(signature)] = msg.range_max
        signature = np.clip(signature, msg.range_min, msg.range_max)
        if self.scan_motion_reference is None:
            self.scan_motion_reference = signature
            self.scan_motion_reference_time = now
        elif now - self.scan_motion_reference_time >= float(
                self.get_parameter('scan_motion_window').value):
            count = min(len(signature), len(self.scan_motion_reference))
            difference = np.abs(signature[:count] - self.scan_motion_reference[:count])
            self.scan_change_score = float(np.percentile(difference, 75))
            self.scan_change_time = now
            self.scan_motion_reference = signature
            self.scan_motion_reference_time = now

    def on_odom(self, msg):
        twist = msg.twist.twist
        self.measured_speed = math.hypot(twist.linear.x, twist.linear.y)
        pose = msg.pose.pose
        self.odom_x = pose.position.x
        self.odom_y = pose.position.y
        q = pose.orientation
        self.odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.have_odom = True

    def sector_min(self, min_angle, max_angle):
        if self.scan is None or not self.scan.ranges:
            return math.inf
        values = []
        for i, value in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if min_angle <= angle <= max_angle and math.isfinite(value):
                if self.scan.range_min < value < self.scan.range_max:
                    values.append(value)
        if not values:
            return math.inf
        # A small percentile is less sensitive to a single noisy ray than min().
        return float(np.percentile(values, 10))

    def sector_blocked_fraction(self, min_angle, max_angle, distance):
        """Return the fraction of valid rays occupied inside ``distance``."""
        if self.scan is None or not self.scan.ranges:
            return 0.0
        total = 0
        blocked = 0
        for i, value in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if min_angle <= angle <= max_angle and math.isfinite(value):
                if self.scan.range_min < value < self.scan.range_max:
                    total += 1
                    if value < distance:
                        blocked += 1
        return float(blocked) / float(total) if total else 0.0

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def dead_end_detected(self):
        distance = float(self.get_parameter('dead_end_scan_distance').value)
        threshold = float(self.get_parameter('dead_end_blocked_fraction').value)
        front = self.sector_blocked_fraction(
            -math.radians(50), math.radians(50), distance)
        left = self.sector_blocked_fraction(
            math.radians(50), math.radians(135), distance)
        right = self.sector_blocked_fraction(
            -math.radians(135), -math.radians(50), distance)
        return front >= threshold and left >= threshold and right >= threshold

    def front_range(self):
        return self.sector_min(-math.radians(8), math.radians(8))

    def board_is_recent(self):
        if self.last_board_time is None:
            return False
        timeout = float(self.get_parameter('board_lost_timeout').value)
        return self.now_seconds() - self.last_board_time <= timeout

    def publish_cmd(self, desired, immediate=False):
        if immediate:
            self.last_cmd = Twist()
            self.cmd_pub.publish(self.last_cmd)
            return
        dt = 1.0 / float(self.get_parameter('control_rate_hz').value)
        linear_step = float(self.get_parameter('max_linear_accel').value) * dt
        angular_step = float(self.get_parameter('max_angular_accel').value) * dt

        def approach(current, target, step):
            return current + max(-step, min(step, target - current))

        output = Twist()
        output.linear.x = approach(self.last_cmd.linear.x, desired.linear.x, linear_step)
        output.linear.y = approach(self.last_cmd.linear.y, desired.linear.y, linear_step)
        output.angular.z = approach(self.last_cmd.angular.z, desired.angular.z, angular_step)
        self.last_cmd = output
        self.cmd_pub.publish(output)

    def stop(self):
        self.publish_cmd(Twist(), immediate=True)

    def control_step(self):
        if self.state in (State.WAITING, State.COMPLETE):
            self.stop()
            return
        if self.scan is None:
            self.stop()
            return

        if (
            self.state == State.EXPLORE
            and self.now_seconds() >= self.dead_end_cooldown_until
            and self.dead_end_detected()
        ):
            self.start_dead_end_escape()

        if self.state not in (
            State.WAITING, State.HOLD, State.COMPLETE, State.AVOID,
            State.DEAD_END_TURN, State.DEAD_END_EXIT, State.RECOVERY,
        ):
            front = self.front_range()
            emergency = float(self.get_parameter('emergency_stop_distance').value)
            ttc_limit = float(self.get_parameter('ttc_stop_seconds').value)
            ttc = front / self.closing_speed if self.closing_speed > 0.05 else math.inf
            final_wall = (
                self.state == State.APPROACH
                and self.board_is_recent()
                and self.board_area >= float(self.get_parameter('final_min_area_ratio').value)
            )
            if front < emergency or (ttc < ttc_limit and not final_wall):
                self.resume_state = self.state
                self.avoid_start = self.now_seconds()
                self.clear_start = None
                self.transition(State.AVOID)

        moving_command = math.hypot(self.last_cmd.linear.x, self.last_cmd.linear.y)
        odom_indicates_stuck = (
            self.have_odom
            and self.measured_speed <= float(self.get_parameter('stuck_measured_speed').value)
        )
        scan_indicates_stuck = (
            self.scan_change_time is not None
            and self.now_seconds() - self.scan_change_time < 1.5
            and self.scan_change_score < float(
                self.get_parameter('scan_motion_min_change').value)
        )
        if (
            self.state in (State.EXPLORE, State.APPROACH)
            and moving_command >= float(self.get_parameter('stuck_command_speed').value)
            and (odom_indicates_stuck if self.have_odom else scan_indicates_stuck)
        ):
            if self.stuck_start is None:
                self.stuck_start = self.now_seconds()
            elif self.now_seconds() - self.stuck_start >= float(self.get_parameter('stuck_timeout').value):
                self.recovery_start = self.now_seconds()
                self.stuck_start = None
                self.transition(State.RECOVERY)
        else:
            self.stuck_start = None

        if self.state == State.EXPLORE:
            if self.board_valid:
                if abs(self.board_x) > float(self.get_parameter('center_tolerance').value):
                    self.transition(State.ALIGN)
                else:
                    self.transition(State.APPROACH)
        elif self.state == State.ALIGN:
            if self.board_valid and abs(self.board_x) <= float(
                    self.get_parameter('center_tolerance').value):
                self.transition(State.APPROACH)
            elif not self.board_is_recent():
                self.transition(State.EXPLORE)
        elif self.state == State.APPROACH:
            if self.board_valid and abs(self.board_x) > float(
                    self.get_parameter('approach_realign_threshold').value):
                self.transition(State.ALIGN)
            elif not self.board_is_recent():
                self.transition(State.EXPLORE)

        if self.state == State.EXPLORE:
            self.explore()
        elif self.state == State.ALIGN:
            self.align()
        elif self.state == State.APPROACH:
            self.approach()
        elif self.state == State.AVOID:
            self.avoid_obstacle()
        elif self.state == State.DEAD_END_TURN:
            self.dead_end_turn()
        elif self.state == State.DEAD_END_EXIT:
            self.dead_end_exit()
        elif self.state == State.RECOVERY:
            self.traction_recovery()
        elif self.state == State.HOLD:
            self.hold()

    def start_dead_end_escape(self):
        left = self.sector_min(math.radians(20), math.radians(150))
        right = self.sector_min(-math.radians(150), -math.radians(20))
        self.dead_end_turn_direction = 1.0 if left >= right else -1.0
        self.dead_end_last_yaw = self.odom_yaw
        self.dead_end_accumulated_yaw = 0.0
        self.dead_end_turn_start = self.now_seconds()
        self.get_logger().info(
            'Three-sided enclosure detected; turning 180 degrees before straight exit.')
        self.transition(State.DEAD_END_TURN)

    def dead_end_turn(self):
        if self.dead_end_last_yaw is not None:
            delta = self.normalize_angle(self.odom_yaw - self.dead_end_last_yaw)
            self.dead_end_accumulated_yaw += self.dead_end_turn_direction * delta
        self.dead_end_last_yaw = self.odom_yaw

        tolerance = float(self.get_parameter('dead_end_turn_tolerance').value)
        elapsed = self.now_seconds() - (
            self.dead_end_turn_start
            if self.dead_end_turn_start is not None else self.now_seconds())
        timed_turn_complete = elapsed >= (
            math.pi / max(float(self.get_parameter('dead_end_turn_speed').value), 0.1) + 0.4)
        if self.dead_end_accumulated_yaw >= math.pi - tolerance or timed_turn_complete:
            self.stop()
            self.dead_end_exit_x = self.odom_x
            self.dead_end_exit_y = self.odom_y
            self.dead_end_exit_start = self.now_seconds()
            self.transition(State.DEAD_END_EXIT)
            return

        cmd = Twist()
        cmd.angular.z = self.dead_end_turn_direction * float(
            self.get_parameter('dead_end_turn_speed').value)
        self.publish_cmd(cmd)

    def dead_end_exit(self):
        distance = math.hypot(
            self.odom_x - self.dead_end_exit_x,
            self.odom_y - self.dead_end_exit_y,
        )
        target_distance = float(self.get_parameter('dead_end_exit_distance').value)
        speed = float(self.get_parameter('dead_end_exit_speed').value)
        elapsed = self.now_seconds() - (
            self.dead_end_exit_start
            if self.dead_end_exit_start is not None else self.now_seconds())
        reached = distance >= target_distance or elapsed >= target_distance / max(speed, 0.05) + 1.0
        if reached:
            self.dead_end_cooldown_until = self.now_seconds() + float(
                self.get_parameter('dead_end_cooldown_seconds').value)
            self.transition(State.EXPLORE)
            return

        if self.front_range() < float(self.get_parameter('emergency_stop_distance').value):
            self.resume_state = State.EXPLORE
            self.avoid_start = self.now_seconds()
            self.clear_start = None
            self.transition(State.AVOID)
            return

        cmd = Twist()
        cmd.linear.x = speed
        self.publish_cmd(cmd)

    def explore(self):
        cmd = Twist()
        now = self.now_seconds()
        if now < self.coverage_spin_until:
            cmd.angular.z = self.coverage_spin_direction * float(
                self.get_parameter('coverage_scan_speed').value)
            self.publish_cmd(cmd)
            return

        if self.have_odom and now - self.last_coverage_sample >= float(
                self.get_parameter('coverage_sample_period').value):
            self.last_coverage_sample = now
            cell_size = max(0.20, float(
                self.get_parameter('coverage_cell_size').value))
            cell = (round(self.odom_x / cell_size), round(self.odom_y / cell_size))
            visits = self.visited_cells.get(cell, 0) + 1
            self.visited_cells[cell] = visits
            threshold = int(self.get_parameter('coverage_revisit_threshold').value)
            cooldown = float(self.get_parameter('coverage_scan_cooldown').value)
            if visits == threshold and now - self.last_coverage_spin >= cooldown:
                scan_speed = float(self.get_parameter('coverage_scan_speed').value)
                self.coverage_spin_direction = -1.0 if self.follow_right_wall else 1.0
                self.coverage_spin_until = now + 2.0 * math.pi / max(scan_speed, 0.2)
                self.last_coverage_spin = now
                self.follow_right_wall = not self.follow_right_wall
                self.get_logger().info(
                    'Coverage loop detected: scanning 360 degrees and switching wall side.')
                cmd.angular.z = self.coverage_spin_direction * scan_speed
                self.publish_cmd(cmd)
                return

        front = self.sector_min(-math.radians(25), math.radians(25))
        left = self.sector_min(math.radians(25), math.radians(90))
        right = self.sector_min(-math.radians(90), -math.radians(25))
        obstacle = float(self.get_parameter('obstacle_distance').value)

        if front < obstacle:
            self.last_turn_direction = 1.0 if left >= right else -1.0
            cmd.angular.z = self.last_turn_direction * float(
                self.get_parameter('search_turn_speed').value)
        else:
            open_speed = float(self.get_parameter('open_space_speed_max').value)
            if math.isfinite(front):
                margin = float(self.get_parameter('adaptive_clearance_margin').value)
                braking = float(self.get_parameter('adaptive_braking_accel').value)
                available = max(0.0, front - margin)
                desired_speed = min(open_speed, math.sqrt(2.0 * braking * available))
            else:
                desired_speed = open_speed
            # Lightweight wall following keeps the robot moving through the
            # unknown fixed layout instead of repeatedly spinning in place.
            side_target = float(self.get_parameter('side_target_distance').value)
            followed_range = right if self.follow_right_wall else left
            if math.isfinite(followed_range):
                error = (
                    side_target - followed_range
                    if self.follow_right_wall
                    else followed_range - side_target)
                turn_limit = float(self.get_parameter('wall_follow_turn_limit').value)
                cmd.angular.z = max(-turn_limit, min(turn_limit, 0.9 * error))
            else:
                bias = float(self.get_parameter('wall_search_turn_bias').value)
                cmd.angular.z = -bias if self.follow_right_wall else bias
            turn_limit = max(
                0.01, float(self.get_parameter('wall_follow_turn_limit').value))
            turn_ratio = min(1.0, abs(cmd.angular.z) / turn_limit)
            reduction = float(self.get_parameter('turn_speed_reduction').value)
            desired_speed *= max(0.25, 1.0 - reduction * turn_ratio)
            cmd.linear.x = max(0.08, desired_speed)
        self.publish_cmd(cmd)

    def board_turn(self):
        kp = float(self.get_parameter('center_kp').value)
        limit = float(self.get_parameter('max_turn_speed').value)
        # Positive image error means board is right; ROS positive yaw turns left.
        return max(-limit, min(limit, -kp * self.board_x))

    def align(self):
        if not self.board_is_recent():
            self.transition(State.EXPLORE)
            return
        cmd = Twist()
        cmd.angular.z = self.board_turn()
        error = abs(self.board_x)
        drive_limit = float(self.get_parameter('align_drive_error_limit').value)
        if error < drive_limit and self.front_range() > float(
                self.get_parameter('obstacle_distance').value):
            cmd.linear.x = float(
                self.get_parameter('align_forward_speed').value) * (
                    1.0 - error / max(drive_limit, 0.01))
        self.publish_cmd(cmd)

    def approach(self):
        if not self.board_is_recent():
            self.transition(State.EXPLORE)
            return
        cmd = Twist()
        cmd.angular.z = self.board_turn()
        front = self.front_range()
        target = float(self.get_parameter('target_wall_range').value)
        tolerance = float(self.get_parameter('wall_range_tolerance').value)
        final_area = float(self.get_parameter('final_min_area_ratio').value)

        if abs(self.board_x) > float(
                self.get_parameter('approach_realign_threshold').value):
            self.transition(State.ALIGN)
            self.publish_cmd(cmd)
            return

        if front <= target + tolerance and self.board_area >= final_area:
            self.stop()
            self.hold_start = self.now_seconds()
            self.transition(State.HOLD)
            return

        # If another obstacle blocks the line to a still-small/far marker,
        # use the omni chassis to sidestep toward the clearer side.
        obstacle = float(self.get_parameter('obstacle_distance').value)
        if front < obstacle and self.board_area < final_area:
            left = self.sector_min(math.radians(20), math.radians(85))
            right = self.sector_min(-math.radians(85), -math.radians(20))
            sidestep = float(self.get_parameter('approach_sidestep_speed').value)
            cmd.linear.y = sidestep if left >= right else -sidestep
        else:
            error = max(0.0, front - target)
            cmd.linear.x = min(
                float(self.get_parameter('approach_speed_max').value),
                max(
                    float(self.get_parameter('approach_speed_min').value),
                    float(self.get_parameter('approach_speed_gain').value) * error,
                ),
            )
        self.publish_cmd(cmd)

    def avoid_obstacle(self):
        front = self.front_range()
        clear_distance = float(self.get_parameter('obstacle_distance').value)
        now = self.now_seconds()
        if front > clear_distance:
            if self.clear_start is None:
                self.clear_start = now
            if now - self.clear_start >= float(self.get_parameter('obstacle_clear_seconds').value):
                target = self.resume_state if self.resume_state not in (State.AVOID, State.RECOVERY) else State.EXPLORE
                self.transition(target)
                return
        else:
            self.clear_start = None

        waited = now - (self.avoid_start if self.avoid_start is not None else now)
        if waited < float(self.get_parameter('obstacle_wait_seconds').value):
            self.stop()
            return

        left = self.sector_min(math.radians(20), math.radians(95))
        right = self.sector_min(-math.radians(95), -math.radians(20))
        direction = 1.0 if left >= right else -1.0
        cmd = Twist()
        broad_wall = self.sector_blocked_fraction(
            -math.radians(40), math.radians(40), clear_distance) >= 0.30
        if broad_wall:
            # Back away while rotating. A pure sidestep can pin an omni wheel
            # against the adjacent wall at a corridor end.
            cmd.linear.x = -0.18
            cmd.angular.z = max(
                0.75, float(self.get_parameter('avoid_turn_speed').value)) * direction
        else:
            # A narrow occupied angular span is normally a pillar / moving
            # obstacle, where the omni chassis can pass fastest by sidestepping.
            cmd.linear.y = float(
                self.get_parameter('avoid_sidestep_speed').value) * direction
            cmd.angular.z = 0.35 * direction
        self.publish_cmd(cmd)

    def traction_recovery(self):
        now = self.now_seconds()
        elapsed = now - (self.recovery_start if self.recovery_start is not None else now)
        cmd = Twist()
        if elapsed < 1.0:
            cmd.linear.x = -float(
                self.get_parameter('recovery_reverse_speed').value)
        elif elapsed < 4.6:
            cmd.angular.z = max(
                0.90, float(self.get_parameter('recovery_turn_speed').value)
            ) * self.last_turn_direction
        elif elapsed < 5.4:
            cmd.linear.x = float(self.get_parameter('dead_end_exit_speed').value)
        else:
            self.transition(State.EXPLORE)
            return
        self.publish_cmd(cmd)

    def hold(self):
        self.stop()
        if self.hold_start is None:
            self.hold_start = self.now_seconds()
        if self.now_seconds() - self.hold_start >= float(self.get_parameter('hold_seconds').value):
            self.transition(State.COMPLETE)
            self.complete_pub.publish(Bool(data=True))
            self.get_logger().info('Finish criterion reached: stopped for 3 seconds.')


def main():
    rclpy.init()
    node = FinishAutonomy()
    try:
        rclpy.spin(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
