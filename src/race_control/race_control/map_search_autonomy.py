#!/usr/bin/env python3
"""One-button race autonomy using a saved map, AMCL, Nav2 and green-board vision."""

import math
from collections import deque
from enum import Enum

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String


class State(Enum):
    WAITING = 'WAITING_FOR_ONE_BUTTON_START'
    PREPARING = 'PREPARING_MAP_SEARCH'
    NAVIGATING = 'NAVIGATING_TO_SEARCH_VIEWPOINT'
    SCANNING = 'SCANNING_360_FOR_GREEN_BOARD'
    ALIGNING = 'ALIGNING_WITH_GREEN_BOARD'
    APPROACHING = 'APPROACHING_YELLOW_FINISH_PAD'
    HOLDING = 'HOLDING_STILL_FOR_3_SECONDS'
    COMPLETE = 'COMPLETE'
    FAILED = 'SEARCH_EXHAUSTED'


class MapSearchAutonomy(Node):
    def __init__(self):
        super().__init__('map_search_autonomy')
        defaults = {
            'control_rate_hz': 15.0, 'viewpoint_spacing': 1.00,
            'viewpoint_clearance': 0.65, 'coverage_radius': 3.8,
            'max_viewpoints': 8, 'scan_angular_speed': 1.6,
            'scan_angle': 6.40, 'center_tolerance': 0.075,
            'center_kp': 1.6, 'max_turn_speed': 1.1,
            'approach_speed_max': 0.46, 'approach_speed_min': 0.07,
            'approach_gain': 0.85, 'target_wall_range': 0.34,
            'wall_range_tolerance': 0.04, 'final_min_area_ratio': 0.025,
            'board_lost_timeout': 1.0, 'hold_seconds': 3.0,
            'viewpoint_retry_rounds': 2, 'max_search_seconds': 285.0,
            'initial_pose_x': 8.0727,
            'initial_pose_y': 7.5312, 'initial_pose_yaw': -1.5708,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        latch = QoSProfile(depth=1)
        latch.reliability = ReliabilityPolicy.RELIABLE
        latch.durability = DurabilityPolicy.TRANSIENT_LOCAL
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_final', 10)
        self.select_pub = self.create_publisher(Bool, '/race/direct_control', 10)
        self.state_pub = self.create_publisher(String, '/race/state', latch)
        self.complete_pub = self.create_publisher(Bool, '/race/complete', latch)
        self.create_subscription(Bool, '/race/start', self.on_start, 10)
        self.create_subscription(Bool, '/race/reset', self.on_reset, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, 10)
        self.create_subscription(Float32MultiArray, '/target/green_board_observation', self.on_board, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.state = State.WAITING
        self.map = None
        # The rules state that the start pose is announced and identical for
        # all runs. AMCL refines it as soon as fresh localization arrives.
        self.pose = (
            float(self.get_parameter('initial_pose_x').value),
            float(self.get_parameter('initial_pose_y').value),
            float(self.get_parameter('initial_pose_yaw').value),
        )
        self.scan = None
        self.board_valid = False
        self.board_x = 0.0
        self.board_area = 0.0
        self.last_board = -math.inf
        self.viewpoints = []
        self.view_index = 0
        self.round = 0
        self.goal_handle = None
        self.goal_pending = False
        self.scan_accum = 0.0
        self.scan_last = None
        self.last_scan_pose = None
        self.hold_start = None
        self.search_start = None
        self.started = False
        self.select_direct(True)
        self.complete_pub.publish(Bool(data=False))
        self.publish_state()
        self.create_timer(1.0 / defaults['control_rate_hz'], self.tick)
        self.get_logger().info('Saved-map search autonomy ready; waiting for one-button start.')

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def transition(self, state):
        if state != self.state:
            self.get_logger().info(f'State: {self.state.value} -> {state.value}')
            self.state = state
            self.publish_state()

    def publish_state(self):
        self.state_pub.publish(String(data=self.state.value))

    def select_direct(self, enabled):
        self.select_pub.publish(Bool(data=enabled))

    def stop(self):
        self.cmd_pub.publish(Twist())

    def on_map(self, msg):
        self.map = msg

    def on_pose(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (p.position.x, p.position.y, yaw)

    def on_scan(self, msg):
        self.scan = msg

    def on_board(self, msg):
        if len(msg.data) < 4:
            return
        self.board_valid = msg.data[0] > 0.5
        self.board_x = float(msg.data[1])
        self.board_area = float(msg.data[3])
        if self.board_valid:
            self.last_board = self.seconds()
            if self.started and self.state in (State.NAVIGATING, State.SCANNING):
                self.begin_visual_finish()

    def on_start(self, msg):
        if not msg.data or self.state not in (State.WAITING, State.COMPLETE, State.FAILED):
            return
        self.started = True
        self.complete_pub.publish(Bool(data=False))
        self.viewpoints = []
        self.view_index = 0
        self.round = 0
        self.hold_start = None
        self.search_start = self.seconds()
        self.last_scan_pose = None
        self.select_direct(True)
        self.transition(State.PREPARING)

    def on_reset(self, msg):
        if not msg.data:
            return
        self.started = False
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.goal_pending = False
        self.search_start = None
        self.stop()
        self.select_direct(True)
        self.complete_pub.publish(Bool(data=False))
        self.transition(State.WAITING)

    def board_recent(self):
        return self.seconds() - self.last_board <= self.get_parameter('board_lost_timeout').value

    def build_viewpoints(self):
        """Line-of-sight coverage followed by map-distance route ordering."""
        msg = self.map
        width, height = msg.info.width, msg.info.height
        grid = np.asarray(msg.data, dtype=np.int16).reshape((height, width))
        resolution = msg.info.resolution
        stride = max(2, int(round(self.get_parameter('viewpoint_spacing').value / resolution)))
        radius = max(1, int(math.ceil(self.get_parameter('viewpoint_clearance').value / resolution)))

        # Discard disconnected islands (including free-looking space outside the
        # course).  Otherwise Nav2 can repeatedly receive geometrically safe but
        # unreachable viewpoints and eventually abort the whole search.
        free = grid == 0
        start_col = int((self.pose[0] - msg.info.origin.position.x) / resolution)
        start_row = int((self.pose[1] - msg.info.origin.position.y) / resolution)
        start_col = int(np.clip(start_col, 0, width - 1))
        start_row = int(np.clip(start_row, 0, height - 1))
        if not free[start_row, start_col]:
            free_cells = np.argwhere(free)
            if free_cells.size == 0:
                return []
            nearest = int(np.argmin(
                (free_cells[:, 0] - start_row) ** 2 +
                (free_cells[:, 1] - start_col) ** 2))
            start_row, start_col = map(int, free_cells[nearest])
        reachable = np.zeros_like(free, dtype=bool)
        reachable[start_row, start_col] = True
        queue = deque([(start_row, start_col)])
        while queue:
            row, col = queue.popleft()
            for next_row, next_col in (
                    (row - 1, col), (row + 1, col),
                    (row, col - 1), (row, col + 1)):
                if (0 <= next_row < height and 0 <= next_col < width and
                        free[next_row, next_col] and
                        not reachable[next_row, next_col]):
                    reachable[next_row, next_col] = True
                    queue.append((next_row, next_col))
        candidates = []
        cells = []
        for row in range(radius, height - radius, stride):
            for col in range(radius, width - radius, stride):
                patch = grid[row-radius:row+radius+1, col-radius:col+radius+1]
                if (reachable[row, col] and np.all(patch >= 0) and
                        np.max(patch) < 50):
                    x = msg.info.origin.position.x + (col + 0.5) * resolution
                    y = msg.info.origin.position.y + (row + 0.5) * resolution
                    candidates.append((x, y))
                    cells.append((row, col))
        if not candidates:
            return []
        points = np.asarray(candidates, dtype=np.float64)
        start_world = np.asarray(self.pose[:2], dtype=np.float64)
        coverage = float(self.get_parameter('coverage_radius').value)

        # Do not build an N x N line-of-sight matrix here.  On a 5 cm map that
        # made the single-threaded ROS callback spend minutes doing Python
        # Bresenham tests, leaving the vehicle stopped in PREPARING.  Instead,
        # use vectorised farthest-point coverage.  Nav2 still performs the real
        # collision-safe path planning to every selected viewpoint.
        min_distance = np.linalg.norm(points - start_world, axis=1)
        selected = []
        max_views = max(0, int(self.get_parameter('max_viewpoints').value) - 1)
        while len(selected) < max_views:
            best = int(np.argmax(min_distance))
            if min_distance[best] <= coverage:
                break
            selected.append(best)
            distance_from_best = np.linalg.norm(points - points[best], axis=1)
            min_distance = np.minimum(min_distance, distance_from_best)

        # Keep one-button latency negligible. Theta* computes the true collision-safe
        # path later; here a nearest-neighbour order is enough to avoid long jumps.
        ordered_indices = []
        remaining = list(selected)
        current_world = start_world
        while remaining:
            distances = [np.linalg.norm(points[index] - current_world) for index in remaining]
            nearest = int(np.argmin(distances))
            chosen = remaining.pop(nearest)
            ordered_indices.append(chosen)
            current_world = points[chosen]
        ordered = [tuple(start_world)] + [tuple(points[index]) for index in ordered_indices]
        self.get_logger().info(
            f'Generated {len(ordered)} map-derived search viewpoints from {len(candidates)} safe samples.')
        return ordered

    def send_next_goal(self):
        if self.goal_pending or self.state == State.NAVIGATING:
            return
        if self.view_index >= len(self.viewpoints):
            self.round += 1
            elapsed = self.seconds() - self.search_start if self.search_start else 0.0
            if elapsed >= float(self.get_parameter('max_search_seconds').value):
                self.select_direct(True)
                self.stop()
                self.transition(State.FAILED)
                return
            if self.round >= int(self.get_parameter('viewpoint_retry_rounds').value):
                self.get_logger().warn(
                    'Search viewpoints exhausted while obstacles may be blocking; '
                    'starting another pass instead of stopping.')
                self.round = 0
            self.viewpoints.reverse()
            self.view_index = 0
        if not self.nav.server_is_ready():
            return
        x, y = self.viewpoints[self.view_index]
        if self.pose is not None and math.hypot(x - self.pose[0], y - self.pose[1]) <= 0.30:
            self.select_direct(True)
            self.scan_accum = 0.0
            self.scan_last = self.seconds()
            self.transition(State.SCANNING)
            return
        yaw = math.atan2(y - self.pose[1], x - self.pose[0]) if self.pose else 0.0
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(0.5 * yaw)
        goal.pose.pose.orientation.w = math.cos(0.5 * yaw)
        self.goal_pending = True
        self.select_direct(False)
        future = self.nav.send_goal_async(goal)
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        self.goal_pending = False
        try:
            self.goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Nav2 goal request failed: {exc}')
            self.view_index += 1
            return
        if not self.goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected search viewpoint; skipping it.')
            self.goal_handle = None
            self.view_index += 1
            return
        self.transition(State.NAVIGATING)
        result = self.goal_handle.get_result_async()
        result.add_done_callback(self.goal_result)

    def goal_result(self, future):
        if self.state != State.NAVIGATING:
            return
        status = future.result().status
        self.goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.select_direct(True)
            self.scan_accum = 0.0
            self.scan_last = self.seconds()
            self.transition(State.SCANNING)
        else:
            self.get_logger().warn(f'Viewpoint navigation ended with status {status}; skipping.')
            moved_since_scan = (
                self.pose is not None and
                (self.last_scan_pose is None or math.hypot(
                    self.pose[0] - self.last_scan_pose[0],
                    self.pose[1] - self.last_scan_pose[1]) >= 0.75))
            if moved_since_scan:
                self.select_direct(True)
                self.scan_accum = 0.0
                self.scan_last = self.seconds()
                self.transition(State.SCANNING)
            else:
                self.view_index += 1
                self.transition(State.PREPARING)

    def begin_visual_finish(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        self.goal_pending = False
        self.select_direct(True)
        self.stop()
        self.transition(State.ALIGNING)

    def front_range(self):
        if self.scan is None:
            return math.inf
        values = []
        for i, value in enumerate(self.scan.ranges):
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if abs(angle) <= math.radians(8) and math.isfinite(value):
                if self.scan.range_min < value < self.scan.range_max:
                    values.append(value)
        return float(np.percentile(values, 10)) if values else math.inf

    def tick(self):
        if self.state in (State.WAITING, State.COMPLETE, State.FAILED):
            self.stop()
            return
        if self.state == State.PREPARING:
            self.stop()
            if self.map is None or self.pose is None:
                return
            if not self.viewpoints:
                self.viewpoints = self.build_viewpoints()
                if not self.viewpoints:
                    self.transition(State.FAILED)
                    return
            self.send_next_goal()
        elif self.state == State.NAVIGATING:
            if self.board_valid:
                self.begin_visual_finish()
        elif self.state == State.SCANNING:
            now = self.seconds()
            dt = max(0.0, now - self.scan_last) if self.scan_last is not None else 0.0
            self.scan_last = now
            speed = float(self.get_parameter('scan_angular_speed').value)
            self.scan_accum += abs(speed) * dt
            cmd = Twist()
            cmd.angular.z = speed
            self.cmd_pub.publish(cmd)
            if self.board_valid:
                self.begin_visual_finish()
            elif self.scan_accum >= float(self.get_parameter('scan_angle').value):
                self.stop()
                if self.pose is not None:
                    self.last_scan_pose = self.pose[:2]
                self.view_index += 1
                self.transition(State.PREPARING)
        elif self.state == State.ALIGNING:
            if not self.board_recent():
                self.scan_accum = 0.0
                self.scan_last = self.seconds()
                self.transition(State.SCANNING)
                return
            if self.board_valid and abs(self.board_x) <= self.get_parameter('center_tolerance').value:
                self.transition(State.APPROACHING)
                return
            cmd = Twist()
            cmd.angular.z = float(np.clip(
                -self.get_parameter('center_kp').value * self.board_x,
                -self.get_parameter('max_turn_speed').value,
                self.get_parameter('max_turn_speed').value))
            self.cmd_pub.publish(cmd)
        elif self.state == State.APPROACHING:
            if not self.board_recent():
                self.stop()
                self.transition(State.ALIGNING)
                return
            if abs(self.board_x) > 0.22:
                self.transition(State.ALIGNING)
                return
            front = self.front_range()
            target = self.get_parameter('target_wall_range').value
            tolerance = self.get_parameter('wall_range_tolerance').value
            if (front <= target + tolerance and
                    self.board_area >= self.get_parameter('final_min_area_ratio').value):
                self.stop()
                self.hold_start = self.seconds()
                self.transition(State.HOLDING)
                return
            error = front - target if math.isfinite(front) else 0.25
            speed = float(np.clip(
                self.get_parameter('approach_gain').value * error,
                self.get_parameter('approach_speed_min').value,
                self.get_parameter('approach_speed_max').value))
            cmd = Twist()
            cmd.linear.x = speed
            cmd.angular.z = float(np.clip(-1.0 * self.board_x, -0.45, 0.45))
            self.cmd_pub.publish(cmd)
        elif self.state == State.HOLDING:
            self.stop()
            if self.seconds() - self.hold_start >= self.get_parameter('hold_seconds').value:
                self.complete_pub.publish(Bool(data=True))
                self.started = False
                self.transition(State.COMPLETE)


def main(args=None):
    rclpy.init(args=args)
    node = MapSearchAutonomy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()
