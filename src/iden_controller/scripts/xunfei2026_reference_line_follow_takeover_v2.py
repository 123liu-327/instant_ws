#!/usr/bin/env python3
"""Reference-based visual line follower that takes over after the old flow.

This is intentionally a new, isolated node.  It waits for the existing total
flow to finish stop-line parking and its 25 cm advance, then stops the legacy
``/follow_test`` node, performs the required hard turn, and follows the chosen
white-line branch.  The image processing is based on the algorithms in
~/src/ucar_2026_line_follow, with command smoothing and fail-safe line loss.
"""

import json
import math
import subprocess
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def angle_error(target, actual):
    return math.atan2(math.sin(target - actual), math.cos(target - actual))


class ReferenceLineFollowTakeover(object):
    def __init__(self):
        rospy.init_node("reference_line_follow_takeover_v2")
        self.bridge = CvBridge()
        self.lock = threading.RLock()

        self.mode = str(rospy.get_param("~mode", "middle")).strip().lower()
        if self.mode not in ("left", "middle", "right"):
            raise ValueError("mode must be left, middle, or right")

        self.image_topic = rospy.get_param("~image_topic", "/ucar_camera/image_raw")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.handoff_topic = rospy.get_param(
            "~handoff_topic", "/factory/virtual_collaboration_status")
        self.status_topic = rospy.get_param(
            "~status_topic", "/reference_line_follow/status")
        self.debug_topic = rospy.get_param(
            "~debug_image_topic", "/reference_line_follow/debug_image")

        self.hard_turn_enabled = bool(rospy.get_param("~hard_turn_enabled", True))
        self.hard_turn_angle = math.radians(abs(float(
            rospy.get_param("~hard_turn_angle_deg", 60.0))))
        self.hard_turn_speed = abs(float(rospy.get_param("~hard_turn_speed", 0.42)))
        self.hard_turn_min_speed = abs(float(
            rospy.get_param("~hard_turn_min_speed", 0.16)))
        self.hard_turn_tolerance = math.radians(abs(float(
            rospy.get_param("~hard_turn_tolerance_deg", 2.5))))
        self.hard_turn_timeout = float(rospy.get_param("~hard_turn_timeout_s", 6.0))
        self.hard_turn_settle = float(rospy.get_param("~hard_turn_settle_s", 0.12))

        self.roi_start = float(rospy.get_param("~roi_y_start_ratio", 0.45))
        self.roi_end = float(rospy.get_param("~roi_y_end_ratio", 1.0))
        self.white_s_max = int(rospy.get_param("~white_s_max", 85))
        self.white_v_min = int(rospy.get_param("~white_v_min", 150))
        self.gray_threshold = int(rospy.get_param("~gray_white_threshold", 185))
        self.kernel_size = int(rospy.get_param("~morph_kernel_size", 5))
        self.min_area = float(rospy.get_param("~min_contour_area", 60.0))
        self.min_line_width = int(rospy.get_param("~min_line_width_px", 6))
        self.max_line_width_ratio = float(
            rospy.get_param("~max_line_width_ratio", 0.32))
        self.merge_gap = int(rospy.get_param("~min_segment_gap_px", 12))
        self.scan_rows = [float(v) for v in rospy.get_param(
            "~scan_row_ratios", [0.20, 0.35, 0.50, 0.65, 0.80, 0.92])]
        self.bottom_weight = float(rospy.get_param("~target_row_weight_bottom", 1.5))
        self.lane_width = float(rospy.get_param("~lane_width_px", 230.0))
        self.lane_width_min = float(rospy.get_param("~lane_width_px_min", 150.0))
        self.lane_width_max = float(rospy.get_param("~lane_width_px_max", 330.0))
        self.lane_width_alpha = float(rospy.get_param("~lane_width_adapt_alpha", 0.12))
        self.fork_count = int(rospy.get_param("~fork_candidate_count", 3))

        self.base_speed = float(rospy.get_param("~base_speed", 0.24))
        self.min_speed = float(rospy.get_param("~min_speed", 0.09))
        self.kp = float(rospy.get_param("~kp", 0.0042))
        self.kd = float(rospy.get_param("~kd", 0.0010))
        self.max_wz = abs(float(rospy.get_param("~max_angular_speed", 0.62)))
        self.slow_error = float(rospy.get_param("~error_slowdown_px", 150.0))
        self.error_alpha = clamp(float(rospy.get_param("~error_filter_alpha", 0.62)), 0.0, 1.0)
        self.command_alpha = clamp(float(rospy.get_param("~command_filter_alpha", 0.55)), 0.0, 1.0)
        self.max_accel = abs(float(rospy.get_param("~max_linear_accel", 0.55)))
        self.max_decel = abs(float(rospy.get_param("~max_linear_decel", 0.90)))
        self.max_wz_rate = abs(float(rospy.get_param("~max_angular_accel", 2.2)))
        self.lost_coast = float(rospy.get_param("~lost_line_coast_s", 0.16))
        self.image_timeout = float(rospy.get_param("~image_timeout_s", 0.45))

        self.finish_enable_delay = float(rospy.get_param("~finish_enable_delay_s", 8.0))
        self.finish_confirm_frames = int(rospy.get_param("~finish_confirm_frames", 4))
        self.finish_width_ratio = float(rospy.get_param("~finish_min_width_ratio", 0.64))
        self.finish_side_ratio = float(rospy.get_param("~finish_min_side_height_ratio", 0.24))
        self.finish_approach_distance = float(
            rospy.get_param("~finish_approach_distance_m", 0.34))
        self.finish_approach_speed = float(
            rospy.get_param("~finish_approach_speed", 0.10))
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=5, latch=True)
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        rospy.Subscriber(self.handoff_topic, String, self.handoff_callback, queue_size=10)
        rospy.Subscriber(self.image_topic, Image, self.image_callback,
                         queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=20)

        self.triggered = False
        self.active = False
        self.finished = False
        self.phase = "waiting_handoff"
        self.yaw = None
        self.odom_xy = None
        self.odom_stamp = 0.0
        self.last_image_stamp = 0.0
        self.track_started = 0.0
        self.last_detection = 0.0
        self.last_control_time = None
        self.filtered_error = 0.0
        self.last_error = 0.0
        self.last_vx = 0.0
        self.last_wz = 0.0
        self.last_lane_center = None
        self.finish_frames = 0
        self.finish_approach_start = None

        self.watchdog = rospy.Timer(rospy.Duration(0.10), self.watchdog_callback)
        rospy.on_shutdown(self.shutdown)
        self.publish_state("WAITING_HANDOFF", mode=self.mode)

    def publish_state(self, state, **values):
        self.phase = state.lower()
        payload = {"state": state, "stamp": time.time(), "mode": self.mode}
        payload.update(values)
        text = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(String(data=text))
        rospy.logwarn("REFERENCE_LINE_FOLLOW %s", text)

    def handoff_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if payload.get("state") != "POST_SIM_LINE_FOLLOW_STARTED":
            return
        with self.lock:
            if self.triggered:
                return
            expected = str(payload.get("mode", self.mode)).lower()
            if expected != self.mode:
                rospy.logerr("handoff mode=%s, takeover mode=%s", expected, self.mode)
                return
            self.triggered = True
        worker = threading.Thread(target=self.takeover_worker)
        worker.daemon = True
        worker.start()

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        with self.lock:
            self.yaw = yaw
            self.odom_xy = (float(p.x), float(p.y))
            self.odom_stamp = time.monotonic()

    def takeover_worker(self):
        self.publish_state("TAKEOVER_STARTING")
        self.stop_robot(12)
        try:
            subprocess.run(["rosnode", "kill", "/follow_test"],
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=1.5, check=False)
        except Exception as exc:
            rospy.logwarn("legacy follower stop returned: %s", exc)
        self.stop_robot(12)

        deadline = time.monotonic() + 3.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                ready = (self.yaw is not None and
                         time.monotonic() - self.odom_stamp < 0.5 and
                         time.monotonic() - self.last_image_stamp < 0.8)
            if ready:
                break
            rospy.sleep(0.03)
        else:
            self.publish_state("FAILED", reason="fresh odom/image unavailable")
            self.stop_robot(20)
            return

        if self.hard_turn_enabled and self.mode in ("left", "right"):
            if not self.perform_hard_turn():
                return

        self.stop_robot(5)
        rospy.sleep(max(0.0, self.hard_turn_settle))
        with self.lock:
            self.active = True
            self.track_started = time.monotonic()
            self.last_detection = self.track_started
            self.last_control_time = None
            self.filtered_error = 0.0
            self.last_error = 0.0
            self.last_vx = 0.0
            self.last_wz = 0.0
        self.publish_state("TRACKING")

    def perform_hard_turn(self):
        direction = 1.0 if self.mode == "left" else -1.0
        with self.lock:
            start_yaw = self.yaw
        target = start_yaw + direction * self.hard_turn_angle
        started = time.monotonic()
        rate = rospy.Rate(35)
        self.publish_state("HARD_TURN", angle_deg=math.degrees(self.hard_turn_angle))
        while not rospy.is_shutdown() and time.monotonic() - started < self.hard_turn_timeout:
            with self.lock:
                yaw = self.yaw
                fresh = time.monotonic() - self.odom_stamp < 0.45
            if yaw is None or not fresh:
                self.stop_robot(3)
                rospy.sleep(0.03)
                continue
            remaining = abs(angle_error(target, yaw))
            if remaining <= self.hard_turn_tolerance:
                self.stop_robot(8)
                self.publish_state("HARD_TURN_COMPLETE",
                                   residual_deg=math.degrees(remaining))
                return True
            speed = clamp(1.7 * remaining, self.hard_turn_min_speed,
                          self.hard_turn_speed)
            command = Twist()
            command.angular.z = direction * speed
            self.cmd_pub.publish(command)
            rate.sleep()
        self.stop_robot(20)
        self.publish_state("FAILED", reason="hard turn timeout")
        return False

    def extract_mask(self, frame):
        height = frame.shape[0]
        y0 = clamp(int(height * self.roi_start), 0, height - 1)
        y1 = clamp(int(height * self.roi_end), y0 + 1, height)
        roi = frame[y0:y1, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(hsv, (0, 0, self.white_v_min),
                               (179, self.white_s_max, 255))
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, gray_mask = cv2.threshold(gray, self.gray_threshold, 255,
                                     cv2.THRESH_BINARY)
        mask = cv2.bitwise_or(hsv_mask, gray_mask)
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]
        cleaned = np.zeros_like(mask)
        for contour in contours:
            if cv2.contourArea(contour) >= self.min_area:
                cv2.drawContours(cleaned, [contour], -1, 255, cv2.FILLED)
        size = max(3, self.kernel_size | 1)
        kernel = np.ones((size, size), np.uint8)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
        return cleaned, int(y0)

    def row_segments(self, row):
        active = row > 0
        starts = np.flatnonzero(active & np.r_[True, ~active[:-1]])
        ends = np.flatnonzero(active & np.r_[~active[1:], True])
        segments = []
        max_width = max(1, int(len(row) * self.max_line_width_ratio))
        for left, right in zip(starts, ends):
            width = int(right - left + 1)
            if self.min_line_width <= width <= max_width:
                segments.append([int(left), int(right), (left + right) / 2.0])
        if not segments:
            return []
        merged = [segments[0]]
        for segment in segments[1:]:
            if segment[0] - merged[-1][1] <= self.merge_gap:
                merged[-1][1] = segment[1]
                merged[-1][2] = (merged[-1][0] + segment[1]) / 2.0
            else:
                merged.append(segment)
        return merged

    def choose_center(self, segments, width):
        if len(segments) >= 2:
            multi = len(segments) >= self.fork_count
            if multi and self.mode == "left":
                pair = segments[:2]
            elif multi and self.mode == "right":
                pair = segments[-2:]
            else:
                pair = min(
                    zip(segments, segments[1:]),
                    key=lambda p: abs((p[0][2] + p[1][2]) * 0.5 - width * 0.5)
                    + 0.25 * abs((p[1][2] - p[0][2]) - self.lane_width))
            measured = pair[1][2] - pair[0][2]
            if self.lane_width_min <= measured <= self.lane_width_max and not multi:
                self.lane_width = ((1.0 - self.lane_width_alpha) * self.lane_width
                                   + self.lane_width_alpha * measured)
            return (pair[0][2] + pair[1][2]) * 0.5, multi
        if len(segments) == 1:
            center = segments[0][2]
            if center < width * 0.5:
                return center + self.lane_width * 0.5, False
            return center - self.lane_width * 0.5, False
        return None, False

    def observe_lane(self, mask):
        centers = []
        rows_debug = []
        fork_rows = 0
        height, width = mask.shape[:2]
        for index, ratio in enumerate(self.scan_rows):
            y = clamp(int(height * ratio), 0, height - 1)
            segments = self.row_segments(mask[y, :])
            center, multi = self.choose_center(segments, width)
            rows_debug.append((int(y), segments, center))
            if multi:
                fork_rows += 1
            if center is not None:
                weight = 1.0 + index / max(1.0, len(self.scan_rows) - 1.0) * (
                    self.bottom_weight - 1.0)
                centers.append((center, weight))
        if not centers:
            return None, fork_rows, rows_debug
        value = sum(c * w for c, w in centers) / sum(w for _, w in centers)
        return clamp(value, 0.0, width - 1.0), fork_rows, rows_debug

    def finish_candidate(self, mask):
        height, width = mask.shape[:2]
        bottom = mask[int(height * 0.48):, :]
        if bottom.size == 0:
            return False
        row_counts = np.count_nonzero(bottom, axis=1)
        horizontal = int(np.max(row_counts)) >= int(width * self.finish_width_ratio)
        col_counts = np.count_nonzero(bottom, axis=0)
        required = int(bottom.shape[0] * self.finish_side_ratio)
        left = bool(np.any(col_counts[:max(1, width // 3)] >= required))
        right = bool(np.any(col_counts[(2 * width) // 3:] >= required))
        return horizontal and left and right

    def image_callback(self, msg):
        with self.lock:
            self.last_image_stamp = time.monotonic()
            if not self.active or self.finished:
                return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "camera conversion failed: %s", exc)
            return

        mask, roi_y = self.extract_mask(frame)
        lane_center, fork_rows, rows_debug = self.observe_lane(mask)
        now = time.monotonic()

        if now - self.track_started >= self.finish_enable_delay:
            self.finish_frames = self.finish_frames + 1 if self.finish_candidate(mask) else 0
        if self.finish_approach_start is None and self.finish_frames >= self.finish_confirm_frames:
            with self.lock:
                if self.odom_xy is None:
                    self.publish_state("FAILED", reason="finish found without odom")
                    self.active = False
                    self.stop_robot(15)
                    return
                self.finish_approach_start = self.odom_xy
            self.publish_state("FINISH_APPROACH")

        if self.finish_approach_start is not None:
            with self.lock:
                xy = self.odom_xy
            distance = 0.0 if xy is None else math.hypot(
                xy[0] - self.finish_approach_start[0],
                xy[1] - self.finish_approach_start[1])
            if distance >= self.finish_approach_distance:
                with self.lock:
                    self.active = False
                    self.finished = True
                self.stop_robot(24)
                self.publish_state("FINISHED", distance_m=distance)
                return

        if lane_center is None:
            if now - self.last_detection <= self.lost_coast:
                self.publish_smoothed(self.last_vx * 0.55, self.last_wz * 0.55, now)
                self.publish_state_throttled("LINE_TEMPORARILY_LOST")
            else:
                self.stop_robot(3)
                self.publish_state_throttled("LINE_LOST_STOPPED")
            self.publish_debug_image(frame, mask, roi_y, rows_debug, None)
            return

        self.last_detection = now
        raw_error = lane_center - frame.shape[1] * 0.5
        self.filtered_error = (self.error_alpha * raw_error
                               + (1.0 - self.error_alpha) * self.filtered_error)
        dt = 0.04 if self.last_control_time is None else clamp(
            now - self.last_control_time, 0.015, 0.20)
        derivative = (self.filtered_error - self.last_error) / dt
        target_wz = clamp(-(self.kp * self.filtered_error + self.kd * derivative),
                          -self.max_wz, self.max_wz)
        slowdown = clamp(1.0 - abs(self.filtered_error) / max(1.0, self.slow_error),
                         0.0, 1.0)
        target_vx = self.min_speed + (self.base_speed - self.min_speed) * slowdown
        if self.finish_approach_start is not None:
            target_vx = min(target_vx, self.finish_approach_speed)
            target_wz = clamp(target_wz, -0.38, 0.38)
        self.publish_smoothed(target_vx, target_wz, now)
        self.last_error = self.filtered_error
        self.last_control_time = now
        self.last_lane_center = lane_center
        self.publish_state_throttled("TRACKING", error_px=round(self.filtered_error, 1),
                                     vx=round(self.last_vx, 3), wz=round(self.last_wz, 3),
                                     fork_rows=fork_rows,
                                     lane_width_px=round(self.lane_width, 1))
        self.publish_debug_image(frame, mask, roi_y, rows_debug, lane_center)

    def publish_smoothed(self, target_vx, target_wz, now):
        dt = 0.04 if self.last_control_time is None else clamp(
            now - self.last_control_time, 0.015, 0.20)
        accel = self.max_accel if target_vx >= self.last_vx else self.max_decel
        target_vx = clamp(target_vx, self.last_vx - accel * dt,
                          self.last_vx + accel * dt)
        target_wz = clamp(target_wz, self.last_wz - self.max_wz_rate * dt,
                          self.last_wz + self.max_wz_rate * dt)
        vx = self.command_alpha * target_vx + (1.0 - self.command_alpha) * self.last_vx
        wz = self.command_alpha * target_wz + (1.0 - self.command_alpha) * self.last_wz
        command = Twist()
        command.linear.x = vx
        command.angular.z = wz
        self.cmd_pub.publish(command)
        self.last_vx, self.last_wz = vx, wz

    def publish_state_throttled(self, state, **values):
        key = "~last_state_log_" + state
        now = time.monotonic()
        previous = getattr(self, key, 0.0)
        if now - previous >= 0.7:
            setattr(self, key, now)
            self.publish_state(state, **values)

    def publish_debug_image(self, frame, mask, roi_y, rows, lane_center):
        if not self.publish_debug or self.debug_pub.get_num_connections() == 0:
            return
        debug = frame.copy()
        for y, segments, center in rows:
            fy = roi_y + y
            for left, right, _ in segments:
                cv2.line(debug, (left, fy), (right, fy), (0, 180, 255), 2)
            if center is not None:
                cv2.circle(debug, (int(center), fy), 4, (255, 0, 0), -1)
        if lane_center is not None:
            cv2.line(debug, (int(lane_center), roi_y),
                     (int(lane_center), frame.shape[0] - 1), (0, 255, 0), 2)
        cv2.line(debug, (frame.shape[1] // 2, roi_y),
                 (frame.shape[1] // 2, frame.shape[0] - 1), (0, 0, 255), 1)
        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))
        except CvBridgeError:
            pass

    def watchdog_callback(self, _event):
        with self.lock:
            should_stop = (self.active and not self.finished and
                           time.monotonic() - self.last_image_stamp > self.image_timeout)
        if should_stop:
            self.stop_robot(2)
            self.publish_state_throttled("CAMERA_TIMEOUT_STOPPED")

    def stop_robot(self, repeats=1):
        command = Twist()
        for _ in range(max(1, repeats)):
            self.cmd_pub.publish(command)
            if repeats > 1:
                rospy.sleep(0.012)
        self.last_vx = 0.0
        self.last_wz = 0.0

    def shutdown(self):
        with self.lock:
            self.active = False
        self.stop_robot(20)


if __name__ == "__main__":
    ReferenceLineFollowTakeover()
    rospy.spin()
