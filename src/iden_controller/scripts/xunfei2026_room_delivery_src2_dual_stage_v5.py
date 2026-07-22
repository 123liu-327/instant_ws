#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""src2 dual-stage v5: quality-gated recenter and no skipped anchor."""

import math
import time

import rospy
from geometry_msgs.msg import Twist

from xunfei2026_room_delivery_src2_dual_stage_v4 import (
    Src2DualStageRoomDeliveryManager,
)


class Src2DualStageRoomDeliveryV5(Src2DualStageRoomDeliveryManager):
    def __init__(self):
        super(Src2DualStageRoomDeliveryV5, self).__init__()
        self.src2_recenter_max_error = abs(float(rospy.get_param(
            "~src2_recenter_max_normalized_error", 0.10)))
        self.src2_recenter_max_width_ratio = abs(float(rospy.get_param(
            "~src2_recenter_max_bbox_width_ratio", 0.62)))
        self.src2_recenter_edge_margin_ratio = abs(float(rospy.get_param(
            "~src2_recenter_horizontal_edge_margin_ratio", 0.035)))
        self.src2_recenter_quality_frames = max(2, int(rospy.get_param(
            "~src2_recenter_quality_frames", 3)))
        self.src2_repeat_interrupted_anchor = bool(rospy.get_param(
            "~src2_repeat_interrupted_anchor", True))
        self.src2_nearby_rotation_attempts = max(2, int(rospy.get_param(
            "~src2_nearby_rotation_attempts", 6)))
        self.src2_nearby_rotation_clearance = max(
            self.sweep_rotation_clearance,
            float(rospy.get_param(
                "~src2_nearby_rotation_clearance_m", 0.025)))
        self._src2_last_anchor_pose = None
        self._src2_force_nearby_anchor_index = None
        self.publish_state(
            "SRC2_DUAL_STAGE_V5_READY",
            recenter_policy="quality_gated_optional",
            failed_parking_resume="same_anchor",
            required_recenter_frames=self.src2_recenter_quality_frames)

    def _nearby_scan_pose(self, anchor, excluded, skip_nominal=False):
        """Return the nearest globally reachable unused pose around an anchor."""
        for distance, x, y in self._candidate_positions(anchor):
            if skip_nominal and distance < 0.05:
                continue
            if any(math.hypot(x - old_x, y - old_y) <
                   self.coverage_failed_exclusion
                   for old_x, old_y in excluded):
                continue
            if not self.make_plan_exists(x, y, anchor["yaw"]):
                continue
            return distance, x, y, anchor["yaw"]
        return None

    def _navigate_to_anchor(self, anchor):
        """Retry nearby poses when TEB rejects the nominal rotation pose."""
        if not self.ensure_ocr_running():
            return "OCR_UNAVAILABLE"
        force_nearby = (
            self._src2_force_nearby_anchor_index ==
            int(self.coverage_active_index))
        excluded = ([(anchor["x"], anchor["y"])] if force_nearby else [])
        last_result = "BLOCKED"
        for attempt in range(1, self.src2_nearby_rotation_attempts + 1):
            selected = self._nearby_scan_pose(
                anchor, excluded, skip_nominal=force_nearby)
            if selected is None:
                break
            distance, x, y, yaw = selected
            self.publish_state(
                "SRC2_ROTATABLE_POSE_SELECTED", anchor=anchor["name"],
                attempt=attempt, x=x, y=y, yaw=yaw,
                fallback_distance=distance)
            previous = self._coverage_scan_active
            self._coverage_scan_active = True
            try:
                last_result = self._src2_navigation_goal(
                    x, y, yaw,
                    "src2_anchor_{}_{}".format(anchor["name"], attempt),
                    timeout=self.src2_goal_hard_timeout,
                    watch_target=True)
            finally:
                self._coverage_scan_active = previous
            if last_result == "TARGET":
                return last_result
            if last_result == "SUCCEEDED":
                self._src2_last_anchor_pose = (x, y, yaw)
                self._src2_force_nearby_anchor_index = None
                return last_result
            excluded.append((x, y))
            self.publish_state(
                "SRC2_ROTATABLE_POSE_RETRY", anchor=anchor["name"],
                attempt=attempt, result=last_result)
            self.clear_and_wait(
                "{} nearby rotation pose {} {}".format(
                    anchor["name"], attempt, last_result))
            force_nearby = True
        return last_result

    def _inspect_anchor(self, anchor):
        """Relocate and repeat a sweep if local clearance blocks rotation."""
        result = super(Src2DualStageRoomDeliveryV5,
                       self)._inspect_anchor(anchor)
        recoverable = ("BLOCKED", "TIMEOUT", "NO_ODOM")
        if result not in recoverable:
            return result
        excluded = [(anchor["x"], anchor["y"])]
        if self._src2_last_anchor_pose is not None:
            excluded.append(self._src2_last_anchor_pose[:2])
        for attempt in range(1, self.src2_nearby_rotation_attempts + 1):
            selected = self._nearby_scan_pose(
                anchor, excluded, skip_nominal=True)
            if selected is None:
                break
            distance, x, y, yaw = selected
            self.publish_state(
                "SCAN_ROTATION_RECOVERY_NAVIGATING", anchor=anchor["name"],
                attempt=attempt, reason=result, x=x, y=y,
                fallback_distance=distance)
            nav_result = self._src2_navigation_goal(
                x, y, yaw,
                "rotation_recovery_{}_{}".format(anchor["name"], attempt),
                timeout=self.src2_goal_hard_timeout,
                watch_target=True)
            if nav_result == "TARGET":
                return nav_result
            if nav_result != "SUCCEEDED":
                excluded.append((x, y))
                self.clear_and_wait(
                    "{} rotation recovery {} {}".format(
                        anchor["name"], attempt, nav_result))
                continue
            rospy.sleep(0.18)
            with self.lock:
                clearance = self.rotation_clearance
                scan_age = time.monotonic() - self.scan_stamp
            if (scan_age > self.sweep_sensor_fresh_s or
                    clearance < self.src2_nearby_rotation_clearance):
                self.publish_state(
                    "SCAN_ROTATION_RECOVERY_CLEARANCE_REJECTED",
                    anchor=anchor["name"], attempt=attempt,
                    clearance=clearance, scan_age=scan_age)
                excluded.append((x, y))
                continue
            self._src2_last_anchor_pose = (x, y, yaw)
            self.publish_state(
                "SCAN_ROTATION_RECOVERY_READY", anchor=anchor["name"],
                attempt=attempt, clearance=clearance)
            result = super(Src2DualStageRoomDeliveryV5,
                           self)._inspect_anchor(anchor)
            if result not in recoverable:
                return result
            excluded.append((x, y))
        return result

    def _credible_stationary_target_bbox(self, payload):
        """Do not interrupt an anchor for a huge/side-clipped workshop sign."""
        if not super(Src2DualStageRoomDeliveryV5,
                     self)._credible_stationary_target_bbox(payload):
            return False
        bbox = payload.get("bbox")
        width = float(payload.get("image_width", 0.0) or 0.0)
        left = float(bbox[0])
        right = float(bbox[2])
        margin_ratio = getattr(
            self, "src2_recenter_edge_margin_ratio", 0.035)
        maximum_width_ratio = getattr(
            self, "src2_recenter_max_width_ratio", 0.62)
        margin = margin_ratio * width
        return (left >= margin and right <= width - margin and
                max(0.0, right - left) / width <= maximum_width_ratio)

    def wall_route_resume_index(self):
        """A failed target attempt must not silently consume the current d point."""
        if self.src2_repeat_interrupted_anchor:
            return max(0, min(
                int(self.coverage_active_index),
                len(self.coverage_anchors) - 1))
        return super(Src2DualStageRoomDeliveryV5, self).wall_route_resume_index()

    def _recenter_sample_is_credible(self, sample):
        if sample is None:
            return False, "no_fresh_target"
        error, _stamp, payload = sample
        bbox = payload.get("bbox")
        width = float(payload.get("image_width", 0.0) or 0.0)
        if (not isinstance(bbox, (list, tuple)) or len(bbox) < 4 or
                width <= 1.0):
            return False, "invalid_bbox"
        left = float(bbox[0])
        right = float(bbox[2])
        bbox_width_ratio = max(0.0, right - left) / width
        margin = self.src2_recenter_edge_margin_ratio * width
        if left < margin or right > width - margin:
            return False, "horizontal_edge_clipped"
        if bbox_width_ratio > self.src2_recenter_max_width_ratio:
            return False, "bbox_too_large"
        if abs(float(error)) > self.src2_recenter_max_error:
            return False, "large_close_range_error"
        return True, "credible"

    def _wait_credible_recenter(self):
        deadline = time.monotonic() + self.src2_recenter_wait
        hits = 0
        last_reason = "no_fresh_target"
        last_error = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            sample = self._src2_target_sample()
            credible, reason = self._recenter_sample_is_credible(sample)
            if credible:
                error = float(sample[0])
                # Require a coherent sign centre, not three frames that jump
                # from one adjacent workshop board to another.
                if last_error is None or abs(error - last_error) <= 0.06:
                    hits += 1
                else:
                    hits = 1
                last_error = error
                if hits >= self.src2_recenter_quality_frames:
                    return sample, "credible"
            else:
                hits = 0
                last_error = None
                last_reason = reason
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.04)
        return None, last_reason

    def park(self):
        sample, reason = self._wait_credible_recenter()
        if sample is not None:
            original_tolerance = self.src2_center_tolerance
            self.src2_center_tolerance = self.src2_recenter_tolerance
            self.publish_state(
                "SRC2_PARKING_RECENTER_ACCEPTED",
                normalized_error=sample[0],
                quality_frames=self.src2_recenter_quality_frames)
            try:
                if not self.align_target_for_parking():
                    return "REACQUIRE"
                if not self._src2_compute_wall_goals():
                    return "REACQUIRE"
            finally:
                self.src2_center_tolerance = original_tolerance
        else:
            # This is the normal src2 fallback: the wall and tangent obtained
            # from the first stable, distant view remain authoritative.
            self.publish_state(
                "SRC2_PARKING_RECENTER_SKIPPED_QUALITY_GATE",
                reason=reason, wait_s=self.src2_recenter_wait,
                max_error=self.src2_recenter_max_error,
                max_bbox_width_ratio=self.src2_recenter_max_width_ratio,
                policy="preserve_first_wall_lock")
        if not self._src2_dock():
            return "REACQUIRE"
        if not self._src2_validate_parking():
            return "REACQUIRE"
        self.stop_robot(10)
        return "SUCCEEDED"


if __name__ == "__main__":
    Src2DualStageRoomDeliveryV5()
    rospy.spin()
