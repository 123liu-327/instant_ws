#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Local line follower with tolerant white extraction and bounded recovery."""

import time

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist

from xunfei2026_reference_line_follow_takeover_v2 import (
    ReferenceLineFollowTakeover,
    clamp,
)


class NativeLineFollowTakeoverV1(ReferenceLineFollowTakeover):
    def __init__(self):
        super(NativeLineFollowTakeoverV1, self).__init__()
        self.native_lost_since = None
        self.native_search_forward_s = max(0.0, float(rospy.get_param(
            "~native_search_forward_s", 0.75)))
        self.native_search_half_sweep_s = max(0.4, float(rospy.get_param(
            "~native_search_half_sweep_s", 1.10)))
        self.native_search_forward_speed = abs(float(rospy.get_param(
            "~native_search_forward_speed", 0.045)))
        self.native_search_angular_speed = abs(float(rospy.get_param(
            "~native_search_angular_speed", 0.14)))
        self.native_search_cycle_limit = max(1, int(rospy.get_param(
            "~native_search_cycle_limit", 4)))
        self.publish_state(
            "NATIVE_FOLLOWER_READY",
            detector="local_tolerant_white_corridor",
            recovery="short_creep_then_bounded_yaw_sweep",
            external_reference="none")

    def extract_mask(self, frame):
        """Keep white rails visible under both dim and overexposed lighting."""
        height = frame.shape[0]
        y0 = clamp(int(height * self.roi_start), 0, height - 1)
        y1 = clamp(int(height * self.roi_end), y0 + 1, height)
        roi = frame[y0:y1, :]
        blur = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        hsv_mask = cv2.inRange(
            hsv, (0, 0, int(self.white_v_min)),
            (179, int(self.white_s_max), 255))
        _, gray_mask = cv2.threshold(
            gray, int(self.gray_threshold), 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_or(hsv_mask, gray_mask)

        # If exposure is unusually dark, derive one bounded threshold from the
        # current lower image instead of declaring every frame line-free.
        ratio = float(np.count_nonzero(mask)) / max(1.0, float(mask.size))
        if ratio < 0.0015:
            adaptive = int(clamp(
                float(np.percentile(gray, 92.0)) - 8.0, 105.0,
                float(self.gray_threshold)))
            _, gray_mask = cv2.threshold(
                gray, adaptive, 255, cv2.THRESH_BINARY)
            mask = cv2.bitwise_or(hsv_mask, gray_mask)

        contours = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]
        cleaned = np.zeros_like(mask)
        maximum_area = 0.72 * float(mask.shape[0] * mask.shape[1])
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area <= area <= maximum_area:
                cv2.drawContours(cleaned, [contour], -1, 255, cv2.FILLED)
        size = max(3, self.kernel_size | 1)
        kernel = np.ones((size, size), np.uint8)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
        return cleaned, int(y0)

    def image_callback(self, message):
        before = self.last_detection
        super(NativeLineFollowTakeoverV1, self).image_callback(message)
        now = time.monotonic()
        with self.lock:
            active = self.active and not self.finished
            detected = self.last_detection > before
        if not active:
            self.native_lost_since = None
            return
        if detected:
            if self.native_lost_since is not None:
                self.publish_state("NATIVE_LINE_REACQUIRED")
            self.native_lost_since = None
            return
        if now - self.last_detection <= self.lost_coast:
            return
        if self.native_lost_since is None:
            self.native_lost_since = now

        elapsed = now - self.native_lost_since
        sweep = self.native_search_half_sweep_s
        cycle = self.native_search_forward_s + 2.0 * sweep
        cycle_index = int(elapsed / max(cycle, 0.1))
        if cycle_index >= self.native_search_cycle_limit:
            self.stop_robot(3)
            self.publish_state_throttled(
                "NATIVE_LINE_SEARCH_WAITING",
                reason="bounded search exhausted",
                cycles=cycle_index)
            return
        phase = elapsed - cycle_index * cycle
        command = Twist()
        if phase < self.native_search_forward_s:
            command.linear.x = self.native_search_forward_speed
            phase_name = "creep"
        else:
            turn_phase = phase - self.native_search_forward_s
            branch_sign = (1.0 if self.mode == "left" else
                           -1.0 if self.mode == "right" else
                           (1.0 if self.last_error <= 0.0 else -1.0))
            direction = branch_sign if turn_phase < sweep else -branch_sign
            command.angular.z = direction * self.native_search_angular_speed
            phase_name = "yaw_sweep"
        self.cmd_pub.publish(command)
        self.last_vx = command.linear.x
        self.last_wz = command.angular.z
        self.publish_state_throttled(
            "NATIVE_LINE_SEARCHING", phase=phase_name,
            cycle=cycle_index + 1,
            vx=round(command.linear.x, 3),
            wz=round(command.angular.z, 3))


if __name__ == "__main__":
    NativeLineFollowTakeoverV1()
    rospy.spin()
