#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Three-entry follower using the robust extraction/control policy from src2."""

import cv2
import numpy as np
import rospy

from xunfei2026_reference_line_follow_takeover_v2 import (
    ReferenceLineFollowTakeover,
    clamp,
)


class Src2LineFollowTakeoverV3(ReferenceLineFollowTakeover):
    def __init__(self):
        super(Src2LineFollowTakeoverV3, self).__init__()
        self.src2_min_mask_ratio = float(rospy.get_param(
            "~src2_min_white_mask_ratio", 0.002))
        self.src2_max_mask_ratio = float(rospy.get_param(
            "~src2_max_white_mask_ratio", 0.32))
        self.src2_adaptive_delta = int(rospy.get_param(
            "~src2_adaptive_threshold_delta", 28))
        self.src2_max_component_ratio = float(rospy.get_param(
            "~src2_max_component_area_ratio", 0.70))
        self.src2_left_offset_ref = float(rospy.get_param(
            "~src2_left_offset_px_at_640", 140.0))
        self.src2_right_offset_ref = float(rospy.get_param(
            "~src2_right_offset_px_at_640", 200.0))
        self.src2_max_line_width_ref = float(rospy.get_param(
            "~src2_max_line_segment_width_px_at_640", 90.0))
        self.src2_max_target_jump_ref = float(rospy.get_param(
            "~src2_max_target_jump_px_at_640", 110.0))
        self.src2_last_target = None
        self.src2_end_roi_start = float(rospy.get_param(
            "~src2_end_roi_y_start_ratio", 0.87))
        self.src2_end_width_ratio = float(rospy.get_param(
            "~src2_end_min_width_ratio", 0.45))

        # v2's filter expression weights the new sample by error_alpha.  A
        # value of 0.42 therefore matches src2's 0.58 previous / 0.42 new EMA.
        self.error_alpha = float(rospy.get_param(
            "~src2_error_new_sample_alpha", 0.42))
        self.kp = float(rospy.get_param("~src2_kp", 0.0048))
        self.kd = float(rospy.get_param("~src2_kd", 0.00035))
        self.base_speed = float(rospy.get_param("~src2_base_speed", 0.25))
        self.min_speed = float(rospy.get_param("~src2_min_speed", 0.10))
        self.max_wz = float(rospy.get_param(
            "~src2_max_angular_speed", 0.60))
        self.slow_error = float(rospy.get_param(
            "~src2_error_slowdown_px", 180.0))
        self.lost_coast = float(rospy.get_param(
            "~src2_lost_line_coast_s", 0.35))
        self.finish_enable_delay = float(rospy.get_param(
            "~src2_end_enable_delay_s", 3.0))
        self.finish_confirm_frames = int(rospy.get_param(
            "~src2_end_confirm_frames", 2))
        self.finish_approach_distance = float(rospy.get_param(
            "~src2_end_forward_distance_m", 0.65))
        self.finish_approach_speed = float(rospy.get_param(
            "~src2_end_forward_speed", 0.15))
        self.publish_state(
            "SRC2_FOLLOWER_READY", detector="adaptive_white_mask",
            tracking=("leftmost" if self.mode == "left" else
                      "rightmost" if self.mode == "right" else
                      "middle_corridor"),
            finish="src2_wide_horizontal_segment")

    @staticmethod
    def _white_mask(hsv, gray, s_max, v_min, gray_threshold, require_both):
        hsv_mask = cv2.inRange(hsv, (0, 0, int(v_min)),
                               (179, int(s_max), 255))
        _, gray_mask = cv2.threshold(
            gray, int(gray_threshold), 255, cv2.THRESH_BINARY)
        return (cv2.bitwise_and(hsv_mask, gray_mask) if require_both else
                cv2.bitwise_or(hsv_mask, gray_mask))

    def extract_mask(self, frame):
        height = frame.shape[0]
        y0 = clamp(int(height * self.roi_start), 0, height - 1)
        y1 = clamp(int(height * self.roi_end), y0 + 1, height)
        roi = frame[y0:y1, :]
        blur = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        mask = self._white_mask(
            hsv, gray, self.white_s_max, self.white_v_min,
            self.gray_threshold, False)
        ratio = float(np.count_nonzero(mask)) / max(1.0, float(mask.size))
        if ratio > self.src2_max_mask_ratio:
            mask = self._white_mask(
                hsv, gray,
                max(20, self.white_s_max - self.src2_adaptive_delta),
                min(245, self.white_v_min + self.src2_adaptive_delta),
                min(245, self.gray_threshold + self.src2_adaptive_delta),
                True)
        elif ratio < self.src2_min_mask_ratio:
            mask = self._white_mask(
                hsv, gray,
                min(140, self.white_s_max + self.src2_adaptive_delta),
                max(80, self.white_v_min - self.src2_adaptive_delta),
                max(100, self.gray_threshold - self.src2_adaptive_delta),
                False)

        contours = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]
        cleaned = np.zeros_like(mask)
        maximum_area = mask.shape[0] * mask.shape[1] * \
            self.src2_max_component_ratio
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_area <= area <= maximum_area:
                cv2.drawContours(cleaned, [contour], -1, 255, cv2.FILLED)
        size = max(3, self.kernel_size | 1)
        kernel = np.ones((size, size), np.uint8)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
        cleaned = cv2.medianBlur(cleaned, 5)
        return cleaned, int(y0)

    def _src2_row_line_center(self, segments, width):
        scale = float(width) / 640.0
        maximum_width = self.src2_max_line_width_ref * scale
        lines = [segment for segment in segments
                 if segment[1] - segment[0] + 1 <= maximum_width]
        if not lines:
            return None
        if self.mode == "left":
            return lines[0][2] + self.src2_left_offset_ref * scale
        if self.mode == "right":
            return lines[-1][2] - self.src2_right_offset_ref * scale
        # Straight/middle keeps a true corridor centre.  Prefer the pair near
        # the prior target so a fork cannot swap sides on alternating frames.
        if len(lines) >= 2:
            pairs = list(zip(lines, lines[1:]))
            reference = (width * 0.5 if self.src2_last_target is None else
                         self.src2_last_target)
            pair = min(pairs, key=lambda value: abs(
                (value[0][2] + value[1][2]) * 0.5 - reference))
            return (pair[0][2] + pair[1][2]) * 0.5
        center = lines[0][2]
        return (center + self.lane_width * 0.5 if center < width * 0.5 else
                center - self.lane_width * 0.5)

    def observe_lane(self, mask):
        values = []
        debug = []
        height, width = mask.shape[:2]
        for index, ratio in enumerate(self.scan_rows):
            y = clamp(int(height * ratio), 0, height - 1)
            segments = self.row_segments(mask[y, :])
            target = self._src2_row_line_center(segments, width)
            debug.append((int(y), segments, target))
            if target is not None:
                # Rows are configured bottom-to-top; src2 trusts the nearer
                # bottom observations more than the distant top observations.
                weight = 1.0 + (len(self.scan_rows) - 1 - index) / max(
                    1.0, len(self.scan_rows) - 1.0) * (
                        self.bottom_weight - 1.0)
                values.append((target, weight))
        if not values:
            return None, 0, debug
        target = sum(value * weight for value, weight in values) / sum(
            weight for _value, weight in values)
        if self.src2_last_target is not None:
            jump = self.src2_max_target_jump_ref * float(width) / 640.0
            target = clamp(target, self.src2_last_target - jump,
                           self.src2_last_target + jump)
        target = clamp(target, 0.0, width - 1.0)
        self.src2_last_target = target
        return target, 0, debug

    @staticmethod
    def _unbounded_segments(row):
        active = row > 0
        starts = np.flatnonzero(active & np.r_[True, ~active[:-1]])
        ends = np.flatnonzero(active & np.r_[~active[1:], True])
        return [(int(left), int(right), int(right - left + 1))
                for left, right in zip(starts, ends)]

    def finish_candidate(self, mask):
        height, width = mask.shape[:2]
        start = clamp(int(height * self.src2_end_roi_start), 0, height - 1)
        minimum_y = start + int((height - start) * 0.45)
        minimum_width = int(width * self.src2_end_width_ratio)
        for y in range(height - 1, minimum_y, -1):
            if any(segment[2] >= minimum_width
                   for segment in self._unbounded_segments(mask[y, :])):
                return True
        return False


if __name__ == "__main__":
    Src2LineFollowTakeoverV3()
    rospy.spin()
