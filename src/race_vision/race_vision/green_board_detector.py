#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray


class GreenBoardDetector(Node):
    """Detect the green A4 finish marker and publish a stable observation."""

    def __init__(self):
        super().__init__('green_board_detector')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('min_area_ratio', 0.00045)
        self.declare_parameter('min_fill_ratio', 0.50)
        self.declare_parameter('min_aspect_ratio', 0.30)
        self.declare_parameter('max_aspect_ratio', 1.20)
        self.declare_parameter('stable_frames', 2)
        self.declare_parameter('lost_frames', 2)
        self.declare_parameter('hsv_low', [40, 45, 25])
        self.declare_parameter('hsv_high', [85, 255, 255])
        self.declare_parameter('use_clahe', True)
        self.declare_parameter('clahe_clip_limit', 2.0)
        self.declare_parameter('green_excess_min', 15)
        self.declare_parameter('max_center_y_ratio', 0.90)
        self.declare_parameter('clipped_marker_min_area_ratio', 0.03)
        self.declare_parameter('publish_debug_image', False)

        self.bridge = CvBridge()
        self.hsv_low = np.array(self.get_parameter('hsv_low').value, dtype=np.uint8)
        self.hsv_high = np.array(self.get_parameter('hsv_high').value, dtype=np.uint8)
        self.hit_count = 0
        self.miss_count = 0
        self.stable = False

        self.pub_detected = self.create_publisher(Bool, '/target/green_board_detected', 10)
        self.pub_center = self.create_publisher(PointStamped, '/target/green_board_center_px', 10)
        self.pub_observation = self.create_publisher(
            Float32MultiArray, '/target/green_board_observation', 10)
        self.pub_debug = self.create_publisher(Image, '/target/green_board_debug_image', 10)
        self.sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info('Green A4 detector is ready')

    def on_image(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge failed: {exc}')
            return

        height, width = bgr.shape[:2]
        image_area = float(width * height)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        if bool(self.get_parameter('use_clahe').value):
            clahe = cv2.createCLAHE(
                clipLimit=float(self.get_parameter('clahe_clip_limit').value),
                tileGridSize=(8, 8),
            )
            hsv[:, :, 2] = clahe.apply(hsv[:, :, 2])

        # Hue is relatively invariant to illumination. A second chromatic
        # green-excess test keeps dark green visible without accepting gray
        # shadows or bright white glare as the marker.
        hsv_mask = cv2.inRange(hsv, self.hsv_low, self.hsv_high)
        blue, green, red = cv2.split(bgr.astype(np.int16))
        green_excess = green - np.maximum(red, blue)
        chromatic_mask = np.where(
            green_excess >= int(self.get_parameter('green_excess_min').value), 255, 0
        ).astype(np.uint8)
        hue_only = cv2.inRange(
            hsv[:, :, 0], int(self.hsv_low[0]), int(self.hsv_high[0]))
        mask = cv2.bitwise_or(hsv_mask, cv2.bitwise_and(chromatic_mask, hue_only))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        if contours:
            for contour in contours:
                area = float(cv2.contourArea(contour))
                x, y, w, h = cv2.boundingRect(contour)
                if w == 0 or h == 0:
                    continue
                area_ratio = area / image_area
                fill_ratio = area / float(w * h)
                aspect_ratio = float(w) / float(h)
                center_y_ratio = (y + 0.5 * h) / height
                touches_border = (
                    x <= 2 or y <= 2 or x + w >= width - 2 or y + h >= height - 2)
                normal_shape = (
                    float(self.get_parameter('min_aspect_ratio').value)
                    <= aspect_ratio
                    <= float(self.get_parameter('max_aspect_ratio').value)
                )
                clipped_shape = (
                    touches_border
                    and area_ratio >= float(
                        self.get_parameter('clipped_marker_min_area_ratio').value)
                    and 0.10 <= aspect_ratio <= 5.0
                )
                if (
                    area_ratio >= float(self.get_parameter('min_area_ratio').value)
                    and fill_ratio >= float(self.get_parameter('min_fill_ratio').value)
                    and (normal_shape or clipped_shape)
                    and center_y_ratio <= float(self.get_parameter('max_center_y_ratio').value)
                ):
                    # A4 portrait width/height is about 0.707. Area remains the
                    # main term, while shape reduces preference for green floor strips.
                    shape_score = 1.0 if clipped_shape else max(
                        0.25, 1.0 - 0.35 * abs(np.log(aspect_ratio / 0.707)))
                    candidates.append((area * shape_score, contour, area, x, y, w, h, area_ratio, fill_ratio))

        candidate = max(candidates, key=lambda item: item[0])[1:] if candidates else None

        if candidate is not None:
            self.hit_count += 1
            self.miss_count = 0
            if self.hit_count >= int(self.get_parameter('stable_frames').value):
                self.stable = True
        else:
            self.hit_count = 0
            self.miss_count += 1
            if self.miss_count > int(self.get_parameter('lost_frames').value):
                self.stable = False

        observation = Float32MultiArray()
        if candidate is None:
            observation.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            contour, area, x, y, w, h, area_ratio, fill_ratio = candidate
            moments = cv2.moments(contour)
            cx = float(moments['m10'] / moments['m00']) if moments['m00'] else x + 0.5 * w
            cy = float(moments['m01'] / moments['m00']) if moments['m00'] else y + 0.5 * h
            cx_norm = (cx - 0.5 * width) / (0.5 * width)
            cy_norm = (cy - 0.5 * height) / (0.5 * height)
            observation.data = [
                1.0 if self.stable else 0.0,
                float(cx_norm),
                float(cy_norm),
                float(area_ratio),
                float(w) / width,
                float(h) / height,
                float(fill_ratio),
            ]

            point = PointStamped()
            point.header = msg.header
            point.point.x = cx
            point.point.y = cy
            point.point.z = area
            self.pub_center.publish(point)

            color = (0, 255, 0) if self.stable else (0, 180, 255)
            cv2.rectangle(bgr, (x, y), (x + w, y + h), color, 2)
            cv2.circle(bgr, (int(cx), int(cy)), 4, (0, 0, 255), -1)
            cv2.putText(
                bgr,
                f'green A4 stable={self.stable} area={area_ratio:.3f}',
                (x, max(20, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
            )

        self.pub_detected.publish(Bool(data=self.stable))
        self.pub_observation.publish(observation)
        if bool(self.get_parameter('publish_debug_image').value):
            debug_msg = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            debug_msg.header = msg.header
            self.pub_debug.publish(debug_msg)


def main():
    rclpy.init()
    node = GreenBoardDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
