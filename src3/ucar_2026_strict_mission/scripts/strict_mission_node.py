#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fail-safe mission coordinator from warehouse completion to track finish."""

from __future__ import annotations

import json
import math
import subprocess
import threading
import time
from collections import deque

import actionlib
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from ucar_2026_strict_mission.logic import (
    ApproachPolicy,
    ConsecutiveBandFilter,
    DistanceCalibration,
    lowest_horizontal_band,
    forward_progress,
    track_launch_for_decision,
    traffic_decision_from_payload,
    valid_stop_line_geometry,
)


TERMINAL_STATES = frozenset(("DONE", "FAULT"))


def quaternion_from_yaw(yaw):
    half = float(yaw) * 0.5
    return math.sin(half), math.cos(half)


def clamp(value, low, high):
    return max(low, min(high, value))


class CenteredStopLineDetector:
    """Detect the transverse white line in a calibrated bird-view image."""

    def __init__(self, params):
        self.width = int(params.get("image_width", 640))
        self.height = int(params.get("image_height", 360))
        self.ground_width = float(params.get("ground_width_m", 0.78))
        self.ground_depth = float(params.get("ground_depth_m", 0.50))
        self.camera_height = float(params.get("camera_height_m", 0.11))
        self.pitch_deg = float(params.get("camera_pitch_deg", 18.0))
        self.white_value_min = int(params.get("white_value_min", 155))
        self.white_saturation_max = int(
            params.get("white_saturation_max", 90))
        self.local_contrast_min = int(params.get("local_contrast_min", 8))
        # 黄色警戒线检测（蓝色地面上的黄色胶带）
        self.yellow_h_min = int(params.get("yellow_h_min", 15))
        self.yellow_h_max = int(params.get("yellow_h_max", 38))
        self.yellow_s_min = int(params.get("yellow_s_min", 100))
        self.yellow_v_min = int(params.get("yellow_v_min", 100))
        self.min_width_m = float(params.get("line_min_width_m", 0.25))
        self.max_width_m = float(params.get("line_max_width_m", 0.74))
        self.max_angle_deg = float(params.get("line_max_angle_deg", 18.0))
        self.fallback_min_y_ratio = float(params.get(
            "line_fallback_min_y_ratio", 0.20))
        self.fallback_max_y_ratio = float(params.get(
            "line_fallback_max_y_ratio", 0.65))
        self.fallback_min_span_ratio = float(params.get(
            "line_fallback_min_span_ratio", 0.55))
        self.homography = self._make_homography()

    def _make_homography(self):
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
            y_camera = (self.camera_height * math.cos(pitch)
                        - y_ground * math.sin(pitch))
            z_camera = (self.camera_height * math.sin(pitch)
                        + y_ground * math.cos(pitch))
            source.append([
                camera_matrix[0, 0] * x_camera / z_camera
                + camera_matrix[0, 2],
                camera_matrix[1, 1] * y_camera / z_camera
                + camera_matrix[1, 2],
            ])
        destination = np.array([
            [0.0, self.height - 1.0],
            [self.width - 1.0, self.height - 1.0],
            [self.width - 1.0, 0.0],
            [0.0, 0.0],
        ], dtype=np.float32)
        return cv2.getPerspectiveTransform(
            np.asarray(source, np.float32), destination)

    def _prepare_frame(self, bgr):
        return cv2.resize(
            bgr, (self.width, self.height), interpolation=cv2.INTER_AREA)

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

    def _raw_hough_fallback(self, mask):
        edges = cv2.Canny(mask, 45, 120)
        minimum_span = int(self.fallback_min_span_ratio * self.width)
        lines = cv2.HoughLinesP(
            edges, 1.0, np.pi / 180.0, threshold=45,
            minLineLength=minimum_span, maxLineGap=38)
        if lines is None:
            return None, None
        best = None
        best_score = -1.0e9
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
            if not (self.fallback_min_y_ratio * self.height <= center_y <=
                    self.fallback_max_y_ratio * self.height):
                continue
            score = (span - 2.0 * abs(angle)
                     - 0.35 * abs(center_x - 0.5 * (self.width - 1.0)))
            if score > best_score:
                best_score = score
                best = (x1, y1, x2, y2, angle, center_x, center_y, span)
        if best is None:
            return None, None
        x1, y1, x2, y2, angle, center_x, center_y, span = best
        bird_point = cv2.perspectiveTransform(
            np.asarray([[[center_x, center_y]]], np.float32),
            self.homography)[0, 0]
        longitudinal = ((self.height - 1.0 - float(bird_point[1]))
                        / (self.height - 1.0) * self.ground_depth)
        lateral = ((center_x - 0.5 * (self.width - 1.0))
                   / (self.width - 1.0) * self.ground_width)
        result = {
            "longitudinal_m": float(clamp(
                longitudinal, 0.0, 1.2 * self.ground_depth)),
            "lateral_m": float(lateral),
            "angle_deg": float(angle),
            "width_m": float(span / self.width * self.ground_width),
            "bbox": [int(x1), int(min(y1, y2)), int(span),
                     int(abs(y2 - y1) + 1)],
            "source": "raw_hough_fallback",
        }
        debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.line(debug, (x1, y1), (x2, y2), (0, 255, 0), 3)
        return result, debug

    def detect(self, bgr):
        if bgr is None or bgr.size == 0:
            return None, None
        frame = self._prepare_frame(bgr)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        local_mean = cv2.GaussianBlur(gray, (31, 31), 0)
        contrast = cv2.subtract(gray, local_mean)
        absolute_white = ((hsv[:, :, 1] <= self.white_saturation_max)
                          & (hsv[:, :, 2] >= self.white_value_min))
        locally_white = contrast >= self.local_contrast_min
        very_white = hsv[:, :, 2] >= min(245, self.white_value_min + 60)
        white_mask = np.where(
            absolute_white & (locally_white | very_white),
            255, 0).astype(np.uint8)
        # 黄色警戒线 mask（与白色取并集，兼容白线和黄线）
        yellow_mask = cv2.inRange(
            hsv,
            (self.yellow_h_min, self.yellow_s_min, self.yellow_v_min),
            (self.yellow_h_max, 255, 255),
        )
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        bird = cv2.warpPerspective(
            mask, self.homography, (self.width, self.height))
        bird = cv2.morphologyEx(
            bird, cv2.MORPH_OPEN, np.ones((2, 9), np.uint8))
        bird = cv2.morphologyEx(
            bird, cv2.MORPH_CLOSE, np.ones((5, 17), np.uint8))
        bird[:8, :] = 0
        bird[self.height - 2:, :] = 0
        bird[:, :10] = 0
        bird[:, self.width - 10:] = 0

        # OpenCV 3 returns image, contours, hierarchy; OpenCV 4 returns two.
        contours = cv2.findContours(
            bird, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        min_width_px = self.min_width_m / self.ground_width * self.width
        max_width_px = self.max_width_m / self.ground_width * self.width
        best = None
        best_score = -1.0e9
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            area = float(cv2.contourArea(contour))
            if (width < min_width_px or width > max_width_px
                    or height < 2 or height > 75
                    or width / float(max(1, height)) < 3.2
                    or area < 180.0):
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
            score = (width - 1.6 * abs(center_error_px)
                     + 0.12 * center_y - 2.0 * abs(angle))
            if score <= best_score:
                continue
            nearest_y = min(self.height - 1.0, y + height - 1.0)
            best = {
                "longitudinal_m": float(
                    (self.height - 1.0 - nearest_y)
                    / (self.height - 1.0) * self.ground_depth),
                "lateral_m": float(
                    center_error_px / (self.width - 1.0)
                    * self.ground_width),
                "angle_deg": float(angle),
                "width_m": float(width / self.width * self.ground_width),
                "bbox": [int(x), int(y), int(width), int(height)],
                "source": "bird_contour",
            }
            best_score = score
        debug = cv2.cvtColor(bird, cv2.COLOR_GRAY2BGR)
        if best is not None:
            x, y, width, height = best["bbox"]
            cv2.rectangle(
                debug, (x, y), (x + width, y + height), (0, 255, 0), 2)
            cv2.line(
                debug, (self.width // 2, 0),
                (self.width // 2, self.height - 1), (0, 0, 255), 1)
            return best, debug
        return self._raw_hough_fallback(mask)


class StrictMissionNode:
    def __init__(self):
        self.lock = threading.RLock()
        self.bridge = CvBridge()
        self.state = "WAIT_START"
        self.fault_reason = ""
        self.started = False
        self.last_image_at = 0.0
        self.line_missing_since = None
        self.last_distance_m = None
        self.odom_pose = None
        self.odom_received_at = 0.0
        self.traffic_hits = 0
        self.last_traffic_decision = None
        self.selected_decision = None
        self.track_status = {}
        self.track_process = None
        self.start_event = threading.Event()
        self.parked_event = threading.Event()
        self.traffic_event = threading.Event()
        self.shutdown_event = threading.Event()

        calibration_points = rospy.get_param(
            "~distance_calibration",
            [[0.55, 0.50], [0.65, 0.32], [0.75, 0.20],
             [0.85, 0.11], [0.90, 0.07], [0.94, 0.03]],
        )
        self.calibration = DistanceCalibration(calibration_points)
        self.target_min_m = float(rospy.get_param("~target_min_m", 0.05))
        self.target_max_m = float(rospy.get_param("~target_max_m", 0.07))
        self.policy = ApproachPolicy(
            self.target_min_m,
            self.target_max_m,
            float(rospy.get_param("~absolute_max_m", 0.10)),
            float(rospy.get_param("~calibration_error_m", 0.03)),
            speed_far=float(rospy.get_param("~speed_far", 0.10)),
            speed_medium=float(rospy.get_param("~speed_medium", 0.06)),
            speed_near=float(rospy.get_param("~speed_near", 0.05)),
            speed_creep=float(rospy.get_param("~speed_creep", 0.045)),
        )
        self.band_filter = ConsecutiveBandFilter(
            int(rospy.get_param("~stop_confirm_frames", 5)),
            self.target_min_m,
            self.target_max_m,
        )
        self.line_detector = CenteredStopLineDetector({
            "white_value_min": rospy.get_param("~white_value_min", 155),
            "white_saturation_max": rospy.get_param(
                "~white_saturation_max", 90),
            "local_contrast_min": rospy.get_param(
                "~local_contrast_min", 8),
            "yellow_h_min": rospy.get_param("~yellow_h_min", 15),
            "yellow_h_max": rospy.get_param("~yellow_h_max", 38),
            "yellow_s_min": rospy.get_param("~yellow_s_min", 100),
            "yellow_v_min": rospy.get_param("~yellow_v_min", 100),
            "line_min_width_m": rospy.get_param(
                "~line_min_width_m", 0.25),
            "line_max_width_m": rospy.get_param(
                "~line_max_width_m", 0.74),
            "line_max_angle_deg": rospy.get_param(
                "~line_max_angle_deg", 18.0),
            "line_fallback_min_y_ratio": rospy.get_param(
                "~line_fallback_min_y_ratio", 0.20),
            "line_fallback_max_y_ratio": rospy.get_param(
                "~line_fallback_max_y_ratio", 0.65),
            "line_fallback_min_span_ratio": rospy.get_param(
                "~line_fallback_min_span_ratio", 0.55),
        })
        self.line_center_tolerance = abs(float(rospy.get_param(
            "~line_center_tolerance_m", 0.030)))
        self.line_angle_tolerance = abs(float(rospy.get_param(
            "~line_angle_tolerance_deg", 3.5)))
        self.line_center_confirm_frames = max(2, int(rospy.get_param(
            "~line_center_confirm_frames", 4)))
        self.line_disappear_confirm_frames = max(2, int(rospy.get_param(
            "~line_disappear_confirm_frames", 4)))
        self.line_crossing_speed = abs(float(rospy.get_param(
            "~line_crossing_speed_mps", 0.065)))
        self.line_crossing_near_distance = abs(float(rospy.get_param(
            "~line_crossing_near_distance_m", 0.105)))
        self.line_crossing_min_distance = abs(float(rospy.get_param(
            "~line_crossing_min_distance_m", 0.025)))
        self.line_crossing_max_distance = abs(float(rospy.get_param(
            "~line_crossing_max_distance_m", 0.42)))
        self.line_stop_before_enabled = bool(rospy.get_param(
            "~line_stop_before_enabled", False))
        self.line_stop_before_distance = abs(float(rospy.get_param(
            "~line_stop_before_distance_m", 0.10)))
        self.line_stop_before_confirm_frames = max(1, int(rospy.get_param(
            "~line_stop_before_confirm_frames", 3)))
        self.line_switch_reject = abs(float(rospy.get_param(
            "~line_switch_reject_m", 0.10)))
        self.line_acquire_max_distance = abs(float(rospy.get_param(
            "~line_acquire_max_distance_m", 0.36)))
        self.line_center_reacquire_timeout = max(0.5, float(rospy.get_param(
            "~line_center_reacquire_timeout_s", 2.0)))
        self.line_max_lateral_speed = abs(float(rospy.get_param(
            "~line_max_lateral_speed_mps", 0.055)))
        self.line_max_angular_speed = abs(float(rospy.get_param(
            "~line_max_angular_speed_rps", 0.18)))
        self.line_lateral_kp = abs(float(rospy.get_param(
            "~line_lateral_kp", 1.2)))
        self.line_angle_kp = abs(float(rospy.get_param(
            "~line_angle_kp", 0.018)))
        self.line_lateral_sign = (1.0 if float(rospy.get_param(
            "~line_lateral_sign", 1.0)) >= 0.0 else -1.0)
        self.line_angle_sign = (1.0 if float(rospy.get_param(
            "~line_angle_sign", -1.0)) >= 0.0 else -1.0)
        self.line_lost_timeout = max(0.2, float(rospy.get_param(
            "~line_lost_timeout_s", 0.65)))
        self.line_search_speed = abs(float(rospy.get_param(
            "~line_search_speed_mps", 0.035)))
        self.line_search_delay = max(0.0, float(rospy.get_param(
            "~line_search_delay_s", 1.0)))
        self.line_max_search_distance = abs(float(rospy.get_param(
            "~line_max_search_distance_m", 0.18)))
        self.line_phase = "CENTERING"
        self.line_started_at = 0.0
        self.line_seen = False
        self.line_last_seen_at = 0.0
        self.line_stable_frames = 0
        self.line_disappeared_frames = 0
        self.line_near_seen = False
        self.line_closest_distance = None
        self.line_center_reference_distance = None
        self.line_center_reject_frames = 0
        self.line_search_anchor = None
        self.line_crossing_anchor = None
        self.line_history = deque(maxlen=5)

        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.status_topic = rospy.get_param(
            "~status_topic", "/strict_mission/status")
        self.traffic_topic = rospy.get_param(
            "~traffic_topic", "/traffic_light_rknn_test/detections")
        self.competition_status_topic = rospy.get_param(
            "~competition_status_topic", "/competition/status")
        self.auto_start = bool(rospy.get_param(
            "~auto_start_on_warehouse_status", False))
        self.warehouse_complete_stage = str(rospy.get_param(
            "~warehouse_complete_stage", "task3"))
        self.required_traffic_frames = max(
            1, int(rospy.get_param("~traffic_confirm_frames", 3)))

        self.cmd_pub = rospy.Publisher(
            self.cmd_vel_topic, Twist, queue_size=1)
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=10, latch=True)
        self.debug_pub = rospy.Publisher(
            "~debug_image", Image, queue_size=1)
        rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1,
            buff_size=2 ** 24,
        )
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=5)
        rospy.Subscriber(
            self.traffic_topic, String, self.traffic_callback, queue_size=10)
        rospy.Subscriber(
            self.competition_status_topic, String,
            self.competition_status_callback, queue_size=10,
        )
        for topic in (
            "/track_end_stop/status",
            "/right_track_end_stop/status",
            "/stable_right_track_end_stop/status",
        ):
            rospy.Subscriber(
                topic, String, self.track_status_callback,
                callback_args=topic, queue_size=10,
            )
        rospy.Service("~start", Trigger, self.start_service)
        rospy.Service("~abort", Trigger, self.abort_service)
        self.move_base = actionlib.SimpleActionClient(
            rospy.get_param("~move_base_action", "move_base"),
            MoveBaseAction,
        )
        self.watchdog = rospy.Timer(
            rospy.Duration(0.05), self.watchdog_callback)
        rospy.on_shutdown(self.shutdown)
        self.publish_status("waiting for explicit start")
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def publish_status(self, detail="", **extra):
        if self.state == "FAULT" and "error" not in extra:
            extra["error"] = self.fault_reason
        payload = {
            "state": self.state,
            "detail": detail,
            "distance_m": self.last_distance_m,
            "decision": self.selected_decision,
            "stamp": rospy.Time.now().to_sec(),
        }
        payload.update(extra)
        self.status_pub.publish(String(
            data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def set_fault(self, reason):
        with self.lock:
            if self.state in TERMINAL_STATES:
                return
            self.state = "FAULT"
            self.fault_reason = str(reason)
            self.shutdown_event.set()
        self.move_base.cancel_all_goals()
        self.publish_stop()
        self.publish_status("fail-safe stop", error=self.fault_reason)
        rospy.logerr("strict mission fault: %s", self.fault_reason)

    def start_service(self, _request):
        with self.lock:
            if self.started:
                return TriggerResponse(
                    success=False, message="mission already started")
            self.started = True
            self.start_event.set()
        return TriggerResponse(success=True, message="strict mission started")

    def abort_service(self, _request):
        self.set_fault("operator abort")
        return TriggerResponse(success=True, message="vehicle stopped")

    def competition_status_callback(self, msg):
        if not self.auto_start or self.started:
            return
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        stage = str(payload.get("stage") or payload.get("task") or "")
        state = str(payload.get("state") or payload.get("status") or "")
        if stage == self.warehouse_complete_stage and state == "completed":
            with self.lock:
                if not self.started:
                    self.started = True
                    self.start_event.set()
                    self.publish_status("warehouse completion trigger accepted")

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                         + orientation.z * orientation.z),
        )
        position = msg.pose.pose.position
        with self.lock:
            self.odom_pose = (position.x, position.y, yaw)
            self.odom_received_at = time.monotonic()

    def _legacy_detect_stop_line(self, frame):
        height, width = frame.shape[:2]
        roi_start = float(rospy.get_param("~line_roi_start_ratio", 0.45))
        y0 = max(0, min(height - 1, int(height * roi_start)))
        roi = frame[y0:, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = (
            0,
            0,
            int(rospy.get_param("~white_v_min", 165)),
        )
        upper = (
            180,
            int(rospy.get_param("~white_s_max", 85)),
            255,
        )
        white_mask = cv2.inRange(hsv, lower, upper)
        # 黄色警戒线 mask（与白色取并集）
        yellow_lower = (
            int(rospy.get_param("~yellow_h_min", 15)),
            int(rospy.get_param("~yellow_s_min", 100)),
            int(rospy.get_param("~yellow_v_min", 100)),
        )
        yellow_upper = (
            int(rospy.get_param("~yellow_h_max", 38)),
            255,
            255,
        )
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        kernel_size = max(3, int(rospy.get_param("~morph_kernel_size", 5)))
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        candidates = []
        for contour in contours:
            x, y, box_width, box_height = cv2.boundingRect(contour)
            if box_width <= 0 or box_height <= 0:
                continue
            area = float(cv2.contourArea(contour))
            width_ratio = float(box_width) / float(width)
            height_ratio = float(box_height) / float(height)
            fill_ratio = area / float(box_width * box_height)
            bottom_ratio = float(y0 + y + box_height) / float(height)
            if valid_stop_line_geometry(
                width_ratio,
                height_ratio,
                fill_ratio,
                bottom_ratio,
                min_width_ratio=float(rospy.get_param(
                    "~line_min_width_ratio", 0.45)),
                max_height_ratio=float(rospy.get_param(
                    "~line_max_height_ratio", 0.12)),
                min_fill_ratio=float(rospy.get_param(
                    "~line_min_fill_ratio", 0.55)),
                min_bottom_ratio=roi_start,
            ):
                candidates.append((
                    width_ratio * bottom_ratio,
                    bottom_ratio,
                    (x, y0 + y, box_width, box_height),
                ))
        if not candidates:
            row_occupancies = np.count_nonzero(mask, axis=1) / float(width)
            band = lowest_horizontal_band(
                row_occupancies,
                float(rospy.get_param("~line_min_width_ratio", 0.45)),
                int(round(height * float(rospy.get_param(
                    "~line_max_height_ratio", 0.12)))),
            )
            if band is None:
                return None, mask, None
            start, end = band
            bottom_ratio = float(y0 + end + 1) / float(height)
            return bottom_ratio, mask, (0, y0 + start, width, end - start + 1)
        _, bottom_ratio, box = max(candidates, key=lambda item: item[0])
        return bottom_ratio, mask, box

    def _legacy_image_callback(self, msg):
        now = time.monotonic()
        with self.lock:
            self.last_image_at = now
            if self.state != "APPROACH_LINE":
                return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.set_fault("cv_bridge failed: {}".format(exc))
            return
        bottom_ratio, mask, box = self._legacy_detect_stop_line(frame)
        if bottom_ratio is None:
            self.publish_stop()
            self.band_filter.reset()
            if self.line_missing_since is None:
                self.line_missing_since = now
            self.publish_status("stop line not trusted; holding stop")
            return
        self.line_missing_since = None
        distance = self.calibration.distance_for_ratio(bottom_ratio)
        self.last_distance_m = distance
        if distance is None:
            self.publish_stop()
            self.band_filter.reset()
            self.publish_status(
                "line outside calibrated range; holding stop",
                line_bottom_ratio=bottom_ratio,
            )
            return
        command = Twist()
        command.linear.x = self.policy.command_for_distance(distance)
        self.cmd_pub.publish(command)
        if self.band_filter.push(distance):
            self.publish_stop()
            with self.lock:
                self.state = "FINAL_ADVANCE"
                self.parked_event.set()
            self.publish_status(
                "visual stop band confirmed; arming odometry final advance",
                line_bottom_ratio=bottom_ratio,
            )
        else:
            self.publish_status(
                "closed-loop line approach",
                line_bottom_ratio=bottom_ratio,
                commanded_speed_mps=command.linear.x,
            )
        if box is not None and self.debug_pub.get_num_connections() > 0:
            x, y, box_width, box_height = box
            cv2.rectangle(
                frame, (x, y), (x + box_width, y + box_height),
                (0, 0, 255), 2,
            )
            cv2.putText(
                frame, "distance={:.3f}m".format(distance), (15, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2,
            )
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

    def _line_travelled(self, anchor):
        with self.lock:
            pose = self.odom_pose
        if anchor is None or pose is None:
            return 0.0
        return math.hypot(pose[0] - anchor[0], pose[1] - anchor[1])

    def _publish_line_command(self, vx=0.0, vy=0.0, wz=0.0):
        command = Twist()
        command.linear.x = float(vx)
        command.linear.y = float(vy)
        command.angular.z = float(wz)
        self.cmd_pub.publish(command)

    def _reset_centered_line_controller(self):
        with self.lock:
            self.parked_event.clear()
            self.line_phase = "CENTERING"
            self.line_started_at = time.monotonic()
            self.line_seen = False
            self.line_last_seen_at = 0.0
            self.line_stable_frames = 0
            self.line_disappeared_frames = 0
            self.line_near_seen = False
            self.line_closest_distance = None
            self.line_stop_before_frames = 0
            self.line_center_reference_distance = None
            self.line_center_reject_frames = 0
            self.line_search_anchor = self.odom_pose
            self.line_crossing_anchor = None
            self.line_history.clear()
            self.line_missing_since = None

    def _finish_centered_line_crossing(self, mode, crossing_distance):
        self.publish_stop()
        with self.lock:
            if self.state != "APPROACH_LINE":
                return
            self.line_phase = "DONE"
            self.state = "LINE_CROSSED"
            self.parked_event.set()
        self.publish_status(
            "centered white line crossed; line disappearance confirmed",
            line_mode=mode,
            crossing_distance_m=float(crossing_distance),
            centered_before_crossing=True,
        )

    def _confirm_line_disappearance(self, mode, crossing_distance, **extra):
        if crossing_distance < self.line_crossing_min_distance:
            return False
        self.line_disappeared_frames += 1
        rospy.loginfo_throttle(
            0.5,
            "[STOP_LINE] disappearance %d/%d progress=%.3fm",
            self.line_disappeared_frames,
            self.line_disappear_confirm_frames,
            crossing_distance,
        )
        if self.line_disappeared_frames < self.line_disappear_confirm_frames:
            return False
        self._finish_centered_line_crossing(mode, crossing_distance)
        return True

    def _process_centered_line_detection(self, detection, now):
        phase = self.line_phase
        if detection is None:
            self.publish_stop()
            self.line_stable_frames = 0
            if phase == "CROSSING":
                crossing_distance = self._line_travelled(
                    self.line_crossing_anchor)
                if self.line_near_seen:
                    self._confirm_line_disappearance(
                        "line_disappeared", crossing_distance)
                    return
                if (self.line_last_seen_at > 0.0 and
                        now - self.line_last_seen_at >=
                        self.line_lost_timeout):
                    self.set_fault(
                        "white line lost before near-field confirmation")
                return

            if self.line_seen:
                if (now - self.line_last_seen_at >=
                        self.line_center_reacquire_timeout):
                    self.set_fault("white line lost during centering")
                return
            if now - self.line_started_at < self.line_search_delay:
                return
            travelled = self._line_travelled(self.line_search_anchor)
            if travelled >= self.line_max_search_distance:
                self.set_fault("white line not found before search limit")
                return
            self._publish_line_command(vx=self.line_search_speed)
            rospy.loginfo_throttle(
                0.8, "[STOP_LINE] searching %.3f/%.3fm",
                travelled, self.line_max_search_distance)
            return

        raw_distance = float(detection["longitudinal_m"])
        if (phase == "CENTERING" and
                raw_distance > self.line_acquire_max_distance):
            self.line_center_reject_frames += 1
            self.publish_stop()
            rospy.logwarn_throttle(
                0.5,
                "[STOP_LINE] rejecting remote candidate distance=%.3f "
                "limit=%.3f rejects=%d",
                raw_distance,
                self.line_acquire_max_distance,
                self.line_center_reject_frames,
            )
            self._process_centered_line_detection(None, now)
            return
        if (phase == "CENTERING" and
                self.line_center_reference_distance is not None and
                abs(raw_distance - self.line_center_reference_distance) >
                self.line_switch_reject):
            self.line_center_reject_frames += 1
            self.publish_stop()
            rospy.logwarn_throttle(
                0.5,
                "[STOP_LINE] keeping locked line old=%.3f candidate=%.3f "
                "rejects=%d",
                self.line_center_reference_distance,
                raw_distance,
                self.line_center_reject_frames,
            )
            self._process_centered_line_detection(None, now)
            return
        if (phase == "CROSSING" and self.line_near_seen
                and self.line_closest_distance is not None
                and raw_distance >
                self.line_closest_distance + self.line_switch_reject):
            self.publish_stop()
            crossing_distance = self._line_travelled(
                self.line_crossing_anchor)
            self._confirm_line_disappearance(
                "tracked_line_replaced", crossing_distance)
            return

        self.line_seen = True
        self.line_last_seen_at = now
        self.line_center_reject_frames = 0
        if (phase == "CENTERING" and
                self.line_center_reference_distance is None):
            self.line_center_reference_distance = raw_distance
        self.line_missing_since = None
        self.line_disappeared_frames = 0
        self.line_history.append(dict(detection))
        if len(self.line_history) < 3:
            self.publish_stop()
            return
        filtered = dict(detection)
        for key in ("longitudinal_m", "lateral_m", "angle_deg", "width_m"):
            filtered[key] = float(np.median([
                item[key] for item in self.line_history
            ]))
        distance = filtered["longitudinal_m"]
        lateral = filtered["lateral_m"]
        angle_deg = filtered["angle_deg"]
        self.last_distance_m = distance
        centered = abs(lateral) <= self.line_center_tolerance
        aligned = abs(angle_deg) <= self.line_angle_tolerance
        vy = clamp(
            self.line_lateral_sign * self.line_lateral_kp * lateral,
            -self.line_max_lateral_speed,
            self.line_max_lateral_speed,
        )
        wz = clamp(
            self.line_angle_sign * self.line_angle_kp * angle_deg,
            -self.line_max_angular_speed,
            self.line_max_angular_speed,
        )

        if phase == "CENTERING":
            if centered and aligned:
                self.line_stable_frames += 1
                self.publish_stop()
            else:
                self.line_stable_frames = 0
                self._publish_line_command(vy=vy, wz=wz)
            if self.line_stable_frames >= self.line_center_confirm_frames:
                with self.lock:
                    anchor = self.odom_pose
                if anchor is None:
                    self.set_fault(
                        "odometry unavailable before white-line crossing")
                    return
                self.publish_stop()
                self.line_crossing_anchor = anchor
                self.line_phase = "CROSSING"
                self.line_near_seen = (
                    distance <= self.line_crossing_near_distance)
                self.line_closest_distance = distance
                self.line_stable_frames = 0
                self.publish_status(
                    "white line center and heading confirmed; crossing",
                    line_distance_m=distance,
                    line_lateral_m=lateral,
                    line_angle_deg=angle_deg,
                    center_source="legacy_white_line_center",
                )
        else:
            crossing_distance = self._line_travelled(
                self.line_crossing_anchor)
            if crossing_distance >= self.line_crossing_max_distance:
                self.publish_stop()
                self.set_fault(
                    "white line did not disappear before distance limit")
                return
            if distance <= self.line_crossing_near_distance:
                self.line_near_seen = True
            if self.line_closest_distance is None:
                self.line_closest_distance = distance
            else:
                self.line_closest_distance = min(
                    self.line_closest_distance, distance)
            # 停在线前模式：距离达到阈值后连续N帧确认，直接停车（不越线）
            if (self.line_stop_before_enabled
                    and distance <= self.line_stop_before_distance):
                self.line_stop_before_frames += 1
                if (self.line_stop_before_frames
                        >= self.line_stop_before_confirm_frames):
                    self.publish_stop()
                    with self.lock:
                        if self.state != "APPROACH_LINE":
                            return
                        self.line_phase = "DONE"
                        self.state = "LINE_CROSSED"
                        self.parked_event.set()
                    self.publish_status(
                        "stop line reached; parked before line",
                        line_distance_m=distance,
                        stop_before_distance_m=self.line_stop_before_distance,
                        centered_before_crossing=True,
                    )
                    return
            else:
                self.line_stop_before_frames = 0
            correction_scale = 0.0 if self.line_near_seen else 0.45
            self._publish_line_command(
                vx=self.line_crossing_speed,
                vy=correction_scale * vy,
                wz=correction_scale * wz,
            )

        rospy.loginfo_throttle(
            0.5,
            "[STOP_LINE] phase=%s distance=%.3f lateral=%+.3f "
            "angle=%+.2f width=%.3f stable=%d/%d near=%s",
            self.line_phase,
            distance,
            lateral,
            angle_deg,
            filtered["width_m"],
            self.line_stable_frames,
            self.line_center_confirm_frames,
            str(self.line_near_seen),
        )

    def image_callback(self, msg):
        now = time.monotonic()
        with self.lock:
            self.last_image_at = now
            active = self.state == "APPROACH_LINE"
        if not active:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8")
            detection, debug = self.line_detector.detect(frame)
        except (CvBridgeError, cv2.error, TypeError, ValueError) as exc:
            self.publish_stop()
            rospy.logwarn_throttle(
                1.0, "[STOP_LINE] image processing error: %s", exc)
            return
        self._process_centered_line_detection(detection, now)
        if debug is not None and self.debug_pub.get_num_connections() > 0:
            try:
                output = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
                output.header = msg.header
                self.debug_pub.publish(output)
            except CvBridgeError:
                pass

    def traffic_callback(self, msg):
        with self.lock:
            if self.state != "WAIT_TRAFFIC":
                return
        try:
            decision = traffic_decision_from_payload(json.loads(msg.data))
        except (TypeError, ValueError):
            decision = None
        if decision is None:
            self.last_traffic_decision = None
            self.traffic_hits = 0
            return
        self.publish_stop()
        if decision == "stop":
            self.last_traffic_decision = "stop"
            self.traffic_hits = 0
            self.publish_status("red light; holding strict stop")
            return
        if decision == self.last_traffic_decision:
            self.traffic_hits += 1
        else:
            self.last_traffic_decision = decision
            self.traffic_hits = 1
        if self.traffic_hits >= self.required_traffic_frames:
            with self.lock:
                self.selected_decision = decision
                self.traffic_event.set()
            self.publish_status("traffic direction confirmed")

    def track_status_callback(self, msg, topic):
        self.track_status[topic] = str(msg.data).strip()

    def watchdog_callback(self, _event):
        with self.lock:
            state = self.state
            last_image_at = self.last_image_at
        if state in ("LINE_CROSSED", "STOP_CONFIRM", "WAIT_TRAFFIC", "FAULT"):
            self.publish_stop()
        if state != "APPROACH_LINE":
            return
        now = time.monotonic()
        stale_stop_sec = float(rospy.get_param("~image_stale_stop_sec", 0.25))
        stale_fault_sec = float(rospy.get_param("~image_stale_fault_sec", 1.0))
        if last_image_at <= 0.0 or now - last_image_at >= stale_stop_sec:
            self.publish_stop()
        if last_image_at > 0.0 and now - last_image_at >= stale_fault_sec:
            self.set_fault("camera image timeout")

    def navigate_to_staging_pose(self):
        if not bool(rospy.get_param("~traffic_pose_configured", False)):
            raise RuntimeError(
                "traffic_pose_configured is false; set staging coordinates")
        timeout = float(rospy.get_param("~navigation_timeout_sec", 120.0))
        if not self.move_base.wait_for_server(rospy.Duration(10.0)):
            raise RuntimeError("move_base action server unavailable")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = rospy.get_param(
            "~traffic_frame", "map")
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(rospy.get_param(
            "~traffic_staging_x"))
        goal.target_pose.pose.position.y = float(rospy.get_param(
            "~traffic_staging_y"))
        sin_half, cos_half = quaternion_from_yaw(
            float(rospy.get_param("~traffic_staging_yaw")))
        goal.target_pose.pose.orientation.z = sin_half
        goal.target_pose.pose.orientation.w = cos_half
        self.move_base.send_goal(goal)
        if not self.move_base.wait_for_result(rospy.Duration(timeout)):
            self.move_base.cancel_goal()
            raise RuntimeError("navigation to stop-line staging pose timed out")
        if self.move_base.get_state() != 3:
            raise RuntimeError(
                "navigation failed with action state {}".format(
                    self.move_base.get_state()))

    def final_advance(self):
        target = float(rospy.get_param("~final_advance_m", 0.0))
        if target <= 0.0:
            return
        speed = float(rospy.get_param("~final_advance_speed", 0.045))
        timeout = float(rospy.get_param("~final_advance_timeout_sec", 6.0))
        stale = float(rospy.get_param("~final_advance_odom_stale_sec", 0.5))
        if speed <= 0.0 or timeout <= 0.0 or stale <= 0.0:
            raise RuntimeError("final advance parameters must be positive")

        wait_deadline = time.monotonic() + min(2.0, timeout)
        start_pose = None
        while not rospy.is_shutdown() and time.monotonic() < wait_deadline:
            with self.lock:
                pose = self.odom_pose
                age = time.monotonic() - self.odom_received_at
            if pose is not None and age <= stale:
                start_pose = pose
                break
            self.publish_stop()
            time.sleep(0.02)
        if start_pose is None:
            raise RuntimeError("fresh odometry unavailable for final advance")

        deadline = time.monotonic() + timeout
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                pose = self.odom_pose
                age = time.monotonic() - self.odom_received_at
            if pose is None or age > stale:
                self.publish_stop()
                raise RuntimeError("odometry became stale during final advance")
            progress = forward_progress(start_pose, pose)
            if progress >= target:
                self.publish_stop()
                self.publish_status(
                    "odometry final advance completed",
                    final_advance_m=progress,
                )
                return
            command = Twist()
            command.linear.x = speed
            self.cmd_pub.publish(command)
            self.publish_status(
                "odometry final advance",
                final_advance_m=progress,
                final_advance_target_m=target,
                commanded_speed_mps=speed,
            )
            rate.sleep()
        self.publish_stop()
        raise RuntimeError("odometry final advance timed out")

    def launch_track(self, decision):
        launch_file, status_topic, finish_value = track_launch_for_decision(
            decision)
        command = [
            "roslaunch", "ucar_2026_track_end_stop", launch_file,
            "start_driver:=false", "start_camera:=false",
            "start_viewer:=false",
        ]
        self.track_process = subprocess.Popen(command)
        return status_topic, finish_value

    def wait_event(self, event, timeout, description):
        deadline = time.monotonic() + float(timeout)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.state == "FAULT":
                raise RuntimeError(self.fault_reason)
            if event.wait(0.05):
                return
        raise RuntimeError("{} timed out".format(description))

    def run(self):
        self.start_event.wait()
        if rospy.is_shutdown():
            return
        try:
            with self.lock:
                self.state = "NAVIGATING"
            self.publish_status("navigating to calibrated staging pose")
            self.navigate_to_staging_pose()
            self.publish_stop()
            self._reset_centered_line_controller()
            with self.lock:
                self.state = "APPROACH_LINE"
                self.last_image_at = time.monotonic()
            self.publish_status(
                "centered transverse white-line parking armed")
            self.wait_event(
                self.parked_event,
                float(rospy.get_param("~line_approach_timeout_sec", 75.0)),
                "strict line approach",
            )
            if not bool(rospy.get_param(
                    "~line_disappearance_parking", True)):
                self.final_advance()
            with self.lock:
                self.state = "STOP_CONFIRM"
            settle = float(rospy.get_param("~stop_settle_sec", 0.6))
            settle_deadline = time.monotonic() + settle
            while time.monotonic() < settle_deadline:
                self.publish_stop()
                time.sleep(0.02)
            with self.lock:
                self.state = "WAIT_TRAFFIC"
            self.publish_status("vehicle held; waiting for traffic consensus")
            self.wait_event(
                self.traffic_event,
                float(rospy.get_param("~traffic_timeout_sec", 180.0)),
                "traffic recognition",
            )
            self.publish_stop()
            with self.lock:
                self.state = "TRACKING"
            status_topic, finish_value = self.launch_track(
                self.selected_decision)
            self.publish_status(
                "matching track controller launched",
                track_status_topic=status_topic,
                expected_finish=finish_value,
            )
            deadline = time.monotonic() + float(rospy.get_param(
                "~track_timeout_sec", 420.0))
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                if self.track_process.poll() is not None:
                    raise RuntimeError(
                        "track controller exited before finish")
                if self.track_status.get(status_topic) == finish_value:
                    break
                time.sleep(0.05)
            else:
                raise RuntimeError("line following timed out")
            self.publish_stop()
            with self.lock:
                self.state = "DONE"
            self.publish_status("strict post-warehouse mission completed")
        except Exception as exc:
            self.set_fault(str(exc))

    def shutdown(self):
        self.shutdown_event.set()
        try:
            self.move_base.cancel_all_goals()
        except Exception:
            pass
        for _ in range(10):
            self.publish_stop()
        if self.track_process and self.track_process.poll() is None:
            self.track_process.terminate()
            try:
                self.track_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.track_process.kill()


def main():
    rospy.init_node("strict_mission")
    StrictMissionNode()
    rospy.spin()


if __name__ == "__main__":
    main()