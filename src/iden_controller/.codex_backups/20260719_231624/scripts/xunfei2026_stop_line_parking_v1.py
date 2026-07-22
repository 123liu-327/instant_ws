#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Camera closed-loop centering and crossing of a transverse white line.

The perspective calibration is inherited from the real-car ucar_followline
implementation.  Unlike its historical single-column test, this node requires
a wide, nearly horizontal component and confirms distance, heading and lateral
centering over several fresh frames.  It then advances at bounded speed and
stops on the first confirmed disappearance of that previously tracked line.
"""

import json
import math
import threading
import time
from collections import deque

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String


def clamp(value, low, high):
    return max(low, min(high, value))


class StopLineDetector(object):
    """Detect a transverse white strip in a metric bird's-eye image."""

    def __init__(self, params=None):
        params = params or {}
        self.width = int(params.get("image_width", 640))
        self.height = int(params.get("image_height", 360))
        self.ground_width = float(params.get("ground_width_m", 0.78))
        self.ground_depth = float(params.get("ground_depth_m", 0.50))
        self.camera_height = float(params.get("camera_height_m", 0.11))
        self.pitch_deg = float(params.get("camera_pitch_deg", 18.0))
        self.white_value_min = int(params.get("white_value_min", 155))
        self.white_saturation_max = int(params.get("white_saturation_max", 90))
        self.local_contrast_min = int(params.get("local_contrast_min", 8))
        self.min_width_m = float(params.get("line_min_width_m", 0.25))
        self.max_width_m = float(params.get("line_max_width_m", 0.74))
        self.max_angle_deg = float(params.get("line_max_angle_deg", 18.0))
        self._homography = self._make_homography()

    def _make_homography(self):
        # The supplied calibration is 1280x720 and is scaled to 640x360 in
        # instant_ws/src/ucar_nav/scripts/ucar_followline.py.
        camera_matrix = np.array([
            [637.5526471889214 * 0.5, 0.0, 639.0844243459007 * 0.5],
            [0.0, 637.5149155824262 * 0.5, 359.5701497245531 * 0.5],
            [0.0, 0.0, 1.0],
        ], dtype=np.float32)
        pitch = math.radians(self.pitch_deg)
        ground = np.array([
            [-self.ground_width / 2.0, 0.0],
            [self.ground_width / 2.0, 0.0],
            [self.ground_width / 2.0, self.ground_depth],
            [-self.ground_width / 2.0, self.ground_depth],
        ], dtype=np.float32)
        source = []
        for x_ground, y_ground in ground:
            x_camera = x_ground
            y_camera = (self.camera_height * math.cos(pitch) -
                        y_ground * math.sin(pitch))
            z_camera = (self.camera_height * math.sin(pitch) +
                        y_ground * math.cos(pitch))
            source.append([
                camera_matrix[0, 0] * x_camera / z_camera + camera_matrix[0, 2],
                camera_matrix[1, 1] * y_camera / z_camera + camera_matrix[1, 2],
            ])
        destination = np.array([
            [0.0, self.height - 1.0],
            [self.width - 1.0, self.height - 1.0],
            [self.width - 1.0, 0.0],
            [0.0, 0.0],
        ], dtype=np.float32)
        return cv2.getPerspectiveTransform(np.asarray(source, np.float32),
                                           destination)

    @staticmethod
    def _line_angle(contour):
        points = contour.reshape(-1, 2).astype(np.float32)
        if len(points) < 2:
            return 90.0
        vx, vy, _, _ = cv2.fitLine(
            points, cv2.DIST_L2, 0, 0.01, 0.01).reshape(-1)
        angle = math.degrees(math.atan2(float(vy), float(vx)))
        while angle > 90.0:
            angle -= 180.0
        while angle < -90.0:
            angle += 180.0
        return angle

    def _detect_raw_transverse(self, mask):
        """Fallback for the real camera when bird-view blobs merge.

        The blue competition floor has several bright regions which can join
        the stop stripe to distant markings after perspective warping.  In the
        original image the required stripe is still a very long, coherent,
        near-horizontal edge.  Hough detection recovers that geometry without
        weakening the bird-view component filters used in normal scenes.
        """
        edges = cv2.Canny(mask, 45, 120)
        minimum_span = int(0.55 * self.width)
        lines = cv2.HoughLinesP(
            edges, 1.0, np.pi / 180.0, threshold=65,
            minLineLength=minimum_span, maxLineGap=38)
        if lines is None:
            return None, None
        best = None
        best_score = -1.0e9
        y_min = 0.20 * self.height
        y_max = 0.65 * self.height
        for raw in lines[:, 0, :]:
            x1, y1, x2, y2 = [int(value) for value in raw]
            if x2 < x1:
                x1, x2, y1, y2 = x2, x1, y2, y1
            span = x2 - x1
            if span < minimum_span:
                continue
            angle = math.degrees(math.atan2(y2 - y1, max(1, span)))
            if abs(angle) > self.max_angle_deg:
                continue
            center_x = 0.5 * (x1 + x2)
            center_y = 0.5 * (y1 + y2)
            if not y_min <= center_y <= y_max:
                continue
            # Prefer a line spanning the complete camera width.  This rejects
            # short floor scratches and parking-box side fragments.
            score = (span - 2.0 * abs(angle) -
                     0.35 * abs(center_x - 0.5 * (self.width - 1.0)))
            if score > best_score:
                best_score = score
                best = (x1, y1, x2, y2, angle, center_x, center_y, span)
        if best is None:
            return None, None

        x1, y1, x2, y2, angle, center_x, center_y, span = best
        source_point = np.asarray([[[center_x, center_y]]], np.float32)
        bird_point = cv2.perspectiveTransform(
            source_point, self._homography)[0, 0]
        longitudinal = ((self.height - 1.0 - float(bird_point[1])) /
                        (self.height - 1.0) * self.ground_depth)
        longitudinal = clamp(longitudinal, 0.0, 1.2 * self.ground_depth)
        lateral = ((center_x - 0.5 * (self.width - 1.0)) /
                   (self.width - 1.0) * self.ground_width)
        result = {
            "longitudinal_m": float(longitudinal),
            "lateral_m": float(lateral),
            "angle_deg": float(angle),
            "width_m": float(span / self.width * self.ground_width),
            "center_px": [float(center_x), float(center_y)],
            "bbox": [int(x1), int(min(y1, y2)), int(span),
                     int(abs(y2 - y1) + 1)],
            "score": float(best_score),
            "source": "raw_hough_fallback",
        }
        debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.line(debug, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.line(debug, (self.width // 2, 0),
                 (self.width // 2, self.height - 1), (0, 0, 255), 1)
        return result, debug

    def detect(self, bgr):
        if bgr is None or bgr.size == 0:
            return None, None
        frame = cv2.resize(bgr, (self.width, self.height),
                           interpolation=cv2.INTER_AREA)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        local_mean = cv2.GaussianBlur(gray, (31, 31), 0)
        contrast = cv2.subtract(gray, local_mean)
        absolute_white = ((hsv[:, :, 1] <= self.white_saturation_max) &
                          (hsv[:, :, 2] >= self.white_value_min))
        locally_white = contrast >= self.local_contrast_min
        very_white = hsv[:, :, 2] >= min(245, self.white_value_min + 60)
        mask = np.where(absolute_white & (locally_white | very_white),
                        255, 0).astype(np.uint8)
        bird = cv2.warpPerspective(mask, self._homography,
                                   (self.width, self.height))
        # Horizontal morphology preserves a cross-line and rejects small glare.
        bird = cv2.morphologyEx(
            bird, cv2.MORPH_OPEN, np.ones((2, 9), np.uint8))
        bird = cv2.morphologyEx(
            bird, cv2.MORPH_CLOSE, np.ones((5, 17), np.uint8))
        bird[:8, :] = 0
        bird[self.height - 2:, :] = 0
        bird[:, :10] = 0
        bird[:, self.width - 10:] = 0

        contours = cv2.findContours(
            bird, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        min_width_px = self.min_width_m / self.ground_width * self.width
        max_width_px = self.max_width_m / self.ground_width * self.width
        best = None
        best_score = -1.0e9
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            area = float(cv2.contourArea(contour))
            if (width < min_width_px or width > max_width_px or
                    height < 2 or height > 75 or
                    width / float(max(1, height)) < 3.2 or area < 180.0):
                continue
            angle = self._line_angle(contour)
            if abs(angle) > self.max_angle_deg:
                continue
            moments = cv2.moments(contour)
            center_x = (float(moments["m10"] / moments["m00"])
                        if abs(moments["m00"]) > 1.0e-6
                        else x + width / 2.0)
            center_y = (float(moments["m01"] / moments["m00"])
                        if abs(moments["m00"]) > 1.0e-6
                        else y + height / 2.0)
            center_error_px = center_x - (self.width - 1.0) / 2.0
            # Prefer a broad line near the optical center.  A mild near-field
            # preference prevents a remote floor highlight winning the score.
            score = (width - 1.6 * abs(center_error_px) +
                     0.12 * center_y - 2.0 * abs(angle))
            if score > best_score:
                nearest_y = min(self.height - 1.0, y + height - 1.0)
                longitudinal = ((self.height - 1.0 - nearest_y) /
                                (self.height - 1.0) * self.ground_depth)
                lateral = (center_error_px / (self.width - 1.0) *
                           self.ground_width)
                best = {
                    "longitudinal_m": float(longitudinal),
                    "lateral_m": float(lateral),
                    "angle_deg": float(angle),
                    "width_m": float(width / self.width * self.ground_width),
                    "center_px": [float(center_x), float(center_y)],
                    "bbox": [int(x), int(y), int(width), int(height)],
                    "score": float(score),
                }
                best_score = score

        debug = cv2.cvtColor(bird, cv2.COLOR_GRAY2BGR)
        if best is not None:
            x, y, width, height = best["bbox"]
            cv2.rectangle(debug, (x, y), (x + width, y + height),
                          (0, 255, 0), 2)
            cv2.line(debug, (self.width // 2, 0),
                     (self.width // 2, self.height - 1), (0, 0, 255), 1)
        if best is None:
            fallback, fallback_debug = self._detect_raw_transverse(mask)
            if fallback is not None:
                return fallback, fallback_debug
        return best, debug


class StopLineParkingNode(object):
    def __init__(self):
        rospy.init_node("xunfei2026_stop_line_parking")
        self.image_topic = rospy.get_param(
            "~image_topic", "/ucar_camera/image_raw")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.status_topic = rospy.get_param(
            "~status_topic", "/factory/stop_line_parking_status")
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/factory/stop_line_parking_debug")
        self.timeout = float(rospy.get_param("~timeout_s", 28.0))
        self.sensor_timeout = float(rospy.get_param("~sensor_timeout_s", 0.45))
        self.center_tolerance = float(rospy.get_param(
            "~center_tolerance_m", 0.030))
        self.angle_tolerance = float(rospy.get_param(
            "~angle_tolerance_deg", 3.5))
        self.center_confirm_frames = int(rospy.get_param(
            "~center_confirm_frames", 4))
        self.disappear_confirm_frames = int(rospy.get_param(
            "~disappear_confirm_frames", 4))
        self.crossing_speed = float(rospy.get_param(
            "~crossing_speed_mps", 0.065))
        self.crossing_near_distance = float(rospy.get_param(
            "~crossing_near_distance_m", 0.105))
        self.crossing_min_distance = float(rospy.get_param(
            "~crossing_min_distance_m", 0.025))
        self.crossing_max_distance = float(rospy.get_param(
            "~crossing_max_distance_m", 0.42))
        self.line_switch_reject = float(rospy.get_param(
            "~line_switch_reject_m", 0.10))
        self.max_lateral_speed = float(rospy.get_param(
            "~max_lateral_speed_mps", 0.055))
        self.max_angular_speed = float(rospy.get_param(
            "~max_angular_speed_rps", 0.18))
        self.lateral_kp = float(rospy.get_param("~lateral_kp", 1.2))
        self.angle_kp = float(rospy.get_param("~angle_kp", 0.018))
        self.lateral_sign = float(rospy.get_param("~lateral_sign", -1.0))
        self.angle_sign = float(rospy.get_param("~angle_sign", -1.0))
        self.line_lost_timeout = float(rospy.get_param(
            "~line_lost_timeout_s", 0.65))
        self.search_speed = float(rospy.get_param(
            "~search_speed_mps", 0.035))
        self.search_delay = float(rospy.get_param("~search_delay_s", 1.0))
        self.max_search_distance = float(rospy.get_param(
            "~max_search_distance_m", 0.18))
        detector_params = {
            "white_value_min": rospy.get_param("~white_value_min", 155),
            "white_saturation_max": rospy.get_param(
                "~white_saturation_max", 90),
            "local_contrast_min": rospy.get_param("~local_contrast_min", 8),
            "line_min_width_m": rospy.get_param("~line_min_width_m", 0.25),
            "line_max_width_m": rospy.get_param("~line_max_width_m", 0.74),
            "line_max_angle_deg": rospy.get_param(
                "~line_max_angle_deg", 18.0),
        }
        self.detector = StopLineDetector(detector_params)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest_detection = None
        self.latest_stamp = 0.0
        self.latest_debug = None
        self.odom_position = None
        self.start_position = None
        self.finished = False
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=5, latch=True)
        self.debug_pub = rospy.Publisher(
            self.debug_topic, Image, queue_size=1)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_callback,
                         queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback,
                         queue_size=10)
        rospy.on_shutdown(self.stop)
        self.publish_status("WAITING_FOR_STOP_LINE")

    def publish_status(self, state, **values):
        payload = {"state": state, "stamp": time.time()}
        payload.update(values)
        text = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(String(data=text))
        rospy.logwarn("STOP_LINE_STATE %s", text)

    def odom_callback(self, msg):
        point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        with self.lock:
            self.odom_position = point
            if self.start_position is None:
                self.start_position = point

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            detection, debug = self.detector.detect(frame)
        except (CvBridgeError, cv2.error, ValueError) as exc:
            rospy.logwarn_throttle(1.0, "stop-line image error: %s", exc)
            return
        with self.lock:
            self.latest_detection = detection
            self.latest_debug = debug
            self.latest_stamp = time.monotonic()
        if self.debug_pub.get_num_connections() > 0:
            try:
                out = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                out.header = msg.header
                self.debug_pub.publish(out)
            except CvBridgeError:
                pass

    def publish_command(self, vx=0.0, vy=0.0, wz=0.0):
        command = Twist()
        command.linear.x = vx
        command.linear.y = vy
        command.angular.z = wz
        self.cmd_pub.publish(command)

    def stop(self):
        for _ in range(12):
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.015)

    def travelled(self, anchor=None):
        with self.lock:
            start = self.start_position if anchor is None else anchor
            current = self.odom_position
        if start is None or current is None:
            return 0.0
        return math.hypot(current[0] - start[0], current[1] - start[1])

    def fail(self, reason, **values):
        self.stop()
        self.publish_status("STOP_LINE_FAILED", reason=reason, **values)
        return False

    def run(self):
        started = time.monotonic()
        stable = 0
        seen_line = False
        last_line_seen = 0.0
        history = deque(maxlen=5)
        phase = "CENTERING"
        crossing_anchor = None
        near_line_seen = False
        closest_crossing_line = None
        disappeared_frames = 0
        processed_stamp = -1.0
        rate = rospy.Rate(15)
        while not rospy.is_shutdown() and time.monotonic() - started < self.timeout:
            now = time.monotonic()
            with self.lock:
                detection = (None if self.latest_detection is None else
                             dict(self.latest_detection))
                stamp = self.latest_stamp
            if stamp <= 0.0 or now - stamp > self.sensor_timeout:
                self.publish_command()
                stable = 0
                rate.sleep()
                continue
            if stamp == processed_stamp:
                rate.sleep()
                continue
            processed_stamp = stamp

            travelled = self.travelled()
            if detection is None:
                stable = 0
                if phase == "CROSSING":
                    self.publish_command()
                    crossing_distance = self.travelled(crossing_anchor)
                    if near_line_seen and \
                            crossing_distance >= self.crossing_min_distance:
                        disappeared_frames += 1
                        rospy.logwarn_throttle(
                            0.20, "STOP_LINE_DISAPPEARED_CONFIRM %d/%d "
                            "distance=%.3f", disappeared_frames,
                            self.disappear_confirm_frames, crossing_distance)
                        if disappeared_frames >= self.disappear_confirm_frames:
                            self.stop()
                            self.publish_status(
                                "STOP_LINE_PARKED", mode="line_disappeared",
                                centered_before_crossing=True,
                                crossing_distance_m=crossing_distance)
                            self.finished = True
                            return True
                    elif now - last_line_seen >= self.line_lost_timeout:
                        return self.fail(
                            "line_lost_before_near_field_confirmation",
                            crossing_distance_m=crossing_distance,
                            near_line_seen=near_line_seen)
                    rate.sleep()
                    continue
                if seen_line:
                    self.publish_command()
                    if now - last_line_seen >= self.line_lost_timeout:
                        return self.fail("line_lost_during_centering",
                                         travelled_m=travelled)
                    rate.sleep()
                    continue
                if now - started < self.search_delay:
                    self.publish_command()
                elif travelled < self.max_search_distance:
                    self.publish_command(vx=self.search_speed)
                    rospy.logwarn_throttle(
                        0.5, "STOP_LINE_SEARCH travelled=%.3f/%.3f",
                        travelled, self.max_search_distance)
                else:
                    return self.fail("line_not_found_before_search_limit",
                                     travelled_m=travelled)
                rate.sleep()
                continue

            # After the tracked stripe has entered the near field, a newly
            # detected line much farther away is not the same target.  This is
            # common in the competition room because another white marking is
            # visible behind the stop stripe.  Treat the jump exactly like the
            # tracked stripe disappearing: brake on the first frame and only
            # finish after several fresh confirmations.
            raw_line_distance = float(detection["longitudinal_m"])
            if (phase == "CROSSING" and near_line_seen and
                    closest_crossing_line is not None and
                    raw_line_distance >
                    closest_crossing_line + self.line_switch_reject):
                self.publish_command()
                crossing_distance = self.travelled(crossing_anchor)
                if crossing_distance >= self.crossing_min_distance:
                    disappeared_frames += 1
                    rospy.logwarn_throttle(
                        0.20, "STOP_LINE_TARGET_SWITCH_REJECTED %d/%d "
                        "old=%.3f new=%.3f travelled=%.3f",
                        disappeared_frames, self.disappear_confirm_frames,
                        closest_crossing_line, raw_line_distance,
                        crossing_distance)
                    if disappeared_frames >= self.disappear_confirm_frames:
                        self.stop()
                        self.publish_status(
                            "STOP_LINE_PARKED",
                            mode="tracked_line_disappeared",
                            centered_before_crossing=True,
                            crossing_distance_m=crossing_distance,
                            rejected_far_line_m=raw_line_distance)
                        self.finished = True
                        return True
                rate.sleep()
                continue

            seen_line = True
            last_line_seen = now
            disappeared_frames = 0
            history.append(detection)
            if len(history) < 3:
                self.publish_command()
                rate.sleep()
                continue
            # Median filtering rejects a one-frame glare contour or camera
            # vibration without adding a noticeable parking delay.
            detection = dict(detection)
            for key in ("longitudinal_m", "lateral_m", "angle_deg", "width_m"):
                detection[key] = float(np.median([item[key] for item in history]))
            line_distance = detection["longitudinal_m"]
            lateral = detection["lateral_m"]
            angle_deg = detection["angle_deg"]
            centered = abs(lateral) <= self.center_tolerance
            aligned = abs(angle_deg) <= self.angle_tolerance
            wz = clamp(self.angle_sign * self.angle_kp * angle_deg,
                       -self.max_angular_speed, self.max_angular_speed)
            vy = clamp(self.lateral_sign * self.lateral_kp * lateral,
                       -self.max_lateral_speed, self.max_lateral_speed)

            if phase == "CENTERING":
                if centered and aligned:
                    stable += 1
                    self.publish_command()
                else:
                    stable = 0
                    self.publish_command(vy=vy, wz=wz)
                if stable >= self.center_confirm_frames:
                    self.stop()
                    with self.lock:
                        crossing_anchor = self.odom_position
                    if crossing_anchor is None:
                        return self.fail("odom_unavailable_before_crossing")
                    phase = "CROSSING"
                    stable = 0
                    near_line_seen = \
                        line_distance <= self.crossing_near_distance
                    closest_crossing_line = line_distance
                    self.publish_status(
                        "STOP_LINE_CENTERED", lateral_m=lateral,
                        angle_deg=angle_deg, line_distance_m=line_distance,
                        line_width_m=detection["width_m"])
            else:
                crossing_distance = self.travelled(crossing_anchor)
                if crossing_distance >= self.crossing_max_distance:
                    return self.fail(
                        "line_did_not_disappear_before_distance_limit",
                        crossing_distance_m=crossing_distance,
                        line_distance_m=line_distance)
                if line_distance <= self.crossing_near_distance:
                    near_line_seen = True
                if closest_crossing_line is None:
                    closest_crossing_line = line_distance
                else:
                    closest_crossing_line = min(
                        closest_crossing_line, line_distance)
                # Once the line reaches the near field, preserve the already
                # confirmed centre/heading instead of chasing clipped endpoints.
                correction_scale = 0.0 if near_line_seen else 0.45
                self.publish_command(
                    vx=self.crossing_speed,
                    vy=correction_scale * vy,
                    wz=correction_scale * wz)

            rospy.logwarn_throttle(
                0.25, "STOP_LINE_TRACK phase=%s distance=%.3f lateral=%.3f "
                "angle=%.2f width=%.3f centered=%d/%d near=%s",
                phase, line_distance, lateral, angle_deg,
                detection["width_m"], stable, self.center_confirm_frames,
                str(near_line_seen))
            rate.sleep()
        return self.fail("parking_timeout", travelled_m=self.travelled())


if __name__ == "__main__":
    node = StopLineParkingNode()
    rospy.sleep(0.25)
    success = node.run()
    # Keep the latched result available briefly for the manager subscriber.
    rospy.sleep(0.5)
    raise SystemExit(0 if success else 2)
