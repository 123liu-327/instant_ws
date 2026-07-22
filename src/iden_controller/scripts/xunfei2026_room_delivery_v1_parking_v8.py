#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Current dual-stage mission with the proven complete-delivery-v1 parking.

Navigation between the calibrated d1..d9 anchors, continuous OCR scanning,
the real/simulation two-stage delivery state machine and the final simulation
handoff remain inherited from the current flow.  Everything from visual target
centering through planned pre-parking and lidar/OCR docking is deliberately
dispatched to ``xunfei2026_room_delivery_manager_v1.py``.  A deliberately
small adapter latches an already-valid near-wall pose across a brief lidar fit
dropout; the approach, centring, cone guards and parking state machine remain
the legacy implementation.
"""

import math
import time

import rospy

from xunfei2026_room_delivery_manager_v1 import (
    Xunfei2026RoomDeliveryManager,
)
from xunfei2026_room_delivery_src2_dual_stage_v5 import (
    Src2DualStageRoomDeliveryV5,
)


class V1ParkingDualStageRoomDeliveryV8(Src2DualStageRoomDeliveryV5):
    def __init__(self):
        super(V1ParkingDualStageRoomDeliveryV8, self).__init__()

        # Use the original vehicle PP-OCR node.  Do not start either src2 OCR
        # implementation or its classifier/adapter.  The inherited v1 values
        # remain authoritative: 8 OCR votes, 5 candidate votes and 3 fresh
        # target frames.
        self.ocr_launch_file = str(rospy.get_param(
            "~ocr_launch_file", "xunfei2026_factory_ocr_v1.launch"))
        self.required_ocr_votes = int(rospy.get_param(
            "~required_ocr_votes", 10))
        self.candidate_ocr_votes = int(rospy.get_param(
            "~candidate_ocr_votes", 7))
        self.candidate_hold_s = float(rospy.get_param(
            "~candidate_hold_s", 6.0))
        self.target_confirm_frames = int(rospy.get_param(
            "~target_confirm_frames", 3))

        # The intermediate coverage/dual-stage classes tuned these values for
        # their experimental fused and src2 parking controllers.  Reload the
        # exact v1 values after the full constructor chain so the methods below
        # behave exactly as they do in complete_delivery_v1.
        self.prepark_align_timeout = float(rospy.get_param(
            "~v1_parking_prepark_align_timeout_s", 10.0))
        self.parking_wall_tolerance = float(rospy.get_param(
            "~v1_parking_wall_tolerance_m", 0.012))
        self.parking_target_memory_s = float(rospy.get_param(
            "~parking_target_memory_s", 3.0))
        self.parking_no_progress_timeout = float(rospy.get_param(
            "~parking_no_progress_timeout_s", 2.2))
        self.parking_no_progress_retries = int(rospy.get_param(
            "~parking_no_progress_retries", 2))
        self.parking_planner_stall_command = max(0.04, float(
            rospy.get_param("~parking_planner_stall_command_m", 0.12)))
        self.parking_center_filter_alpha = float(rospy.get_param(
            "~parking_center_filter_alpha", 0.45))
        self.parking_center_max_step = float(rospy.get_param(
            "~parking_center_max_step_px", 70.0))
        self.parking_center_tolerance = float(rospy.get_param(
            "~parking_center_tolerance_px", 20.0))
        self.parking_center_release_tolerance = float(rospy.get_param(
            "~parking_center_release_tolerance_px",
            max(24.0, 1.8 * self.parking_center_tolerance)))
        self.parking_stable_frames = int(rospy.get_param(
            "~parking_stable_frames", 3))
        self.parking_centerline_max_drift = float(rospy.get_param(
            "~parking_centerline_max_drift_m", 0.035))

        # The physical chassis normally coasts from about 0.178 m to 0.155 m
        # while braking for the configured 0.171 m stop.  Both readings are a
        # valid centred parking pose, but the close wall can disappear from
        # the line fit before three stable samples arrive.  Keep only the last
        # stopped, centred, near-goal sample for a short confirmation window.
        self.v1_close_heading_accept = math.radians(abs(float(
            rospy.get_param("~v1_parking_close_heading_accept_deg", 6.0))))
        self.v1_near_goal_memory_s = max(0.15, float(rospy.get_param(
            "~v1_parking_near_goal_memory_s", 0.85)))
        self._v1_last_near_goal_values = None
        self._v1_last_near_goal_stamp = 0.0
        self._v1_near_goal_latch_reported = False

        # v1 does not hand an in-box lateral failure to a second experimental
        # recovery controller.  It stops, releases the target and lets the
        # existing d-point planner resume the search safely.
        self.parking_planner_recovery_enabled = False

        self.publish_state(
            "V1_PARKING_V8_READY",
            parking_logic="complete_delivery_v1_with_near_goal_latch",
            preparking="v1_live_wall_and_teb",
            centering="v1_ocr_lateral",
            docking="v1_lidar_wall",
            factory_ocr="original_xunfei2026_factory_ocr_v1",
            candidate_policy="physical_sign_pause_then_continue_ocr",
            candidate_votes=self.candidate_ocr_votes,
            required_votes=self.required_ocr_votes,
            floor_frame_detector=False,
            dual_stage=True)

    # Do not use super() here.  Every method is intentionally pinned to the
    # named v1 implementation so later classes in the inheritance chain cannot
    # silently reintroduce fused/src2/frame-center parking behaviour.
    def align_target_for_parking(self):
        return Xunfei2026RoomDeliveryManager.align_target_for_parking(self)

    def refresh_centered_target_wall(self):
        """Keep a confirmed sign while rebuilding its physical wall lock.

        The old 0.35 s lock window can contain fewer than two fresh lidar
        scans on the real car.  If that transient lock fails, retain the
        already-confirmed target and use the lidar wall geometry captured at
        the same physical text box.  Never fall back to a map-projected wall.
        """
        original_lock_s = self.preparking_wall_lock_s
        self.preparking_wall_lock_s = max(1.0, original_lock_s)
        try:
            if Xunfei2026RoomDeliveryManager.refresh_centered_target_wall(self):
                return True
        finally:
            self.preparking_wall_lock_s = original_lock_s

        with self.lock:
            snapshot = (None if self.target_snapshot is None else
                        dict(self.target_snapshot))
            payload = (None if self.parking_target_ocr is None else
                       dict(self.parking_target_ocr))
        geometry = (snapshot.get("wall_geometry")
                    if isinstance(snapshot, dict) else None)
        if not isinstance(geometry, dict) or not isinstance(payload, dict):
            self.publish_state(
                "CONFIRMED_TARGET_WALL_REACQUIRE_UNAVAILABLE",
                reason="no_cached_lidar_wall")
            return False

        try:
            goal_x = float(geometry["goal_x"])
            goal_y = float(geometry["goal_y"])
            goal_yaw = float(geometry["goal_yaw"])
            wall_distance = float(geometry["wall_distance"])
            wall_span = float(geometry["wall_span"])
            wall_residual = float(geometry["wall_residual"])
            wall_points = int(geometry["wall_points"])
        except (KeyError, TypeError, ValueError):
            self.publish_state(
                "CONFIRMED_TARGET_WALL_REACQUIRE_UNAVAILABLE",
                reason="invalid_cached_lidar_wall")
            return False

        cached_wall_safe = all((
            payload.get("label") == self.target_warehouse,
            payload.get("frame_label") == self.target_warehouse,
            0.20 <= wall_distance <= 2.20,
            wall_span >= 0.24,
            wall_residual <= 0.05,
            wall_points >= 8,
            self.room_min_x + 0.06 <= goal_x <= self.room_max_x - 0.06,
            self.room_min_y + 0.06 <= goal_y <= self.room_max_y - 0.06,
        ))
        if not cached_wall_safe:
            self.publish_state(
                "CONFIRMED_TARGET_WALL_REACQUIRE_REJECTED",
                reason="cached_lidar_geometry_quality")
            return False

        # The chassis only rotated during visual centring, so this previously
        # fitted map-frame standoff remains the same physical wall target.
        with self.lock:
            self.parking_target_wall_yaw = goal_yaw
            self.parking_target_stamp = time.monotonic()
        self.publish_state(
            "CONFIRMED_TARGET_WALL_LOCK_REUSED",
            wall_distance=wall_distance, wall_points=wall_points,
            policy="keep_target_and_reuse_same_bbox_lidar_wall")
        return True

    def approach_target(self):
        return Xunfei2026RoomDeliveryManager.approach_target(self)

    def handoff_directly_to_parking(self):
        return Xunfei2026RoomDeliveryManager.handoff_directly_to_parking(self)

    def parking_front_wall_estimate(self):
        return Xunfei2026RoomDeliveryManager.parking_front_wall_estimate(self)

    def parking_commands(self, allow_forward):
        values = Xunfei2026RoomDeliveryManager.parking_commands(
            self, allow_forward)
        now = time.monotonic()
        if values is not None:
            (vx, vy, wz, distance, heading, points, pixel_error,
             target_live, target_age) = values
            near_goal = (
                allow_forward and self.parking_center_aligned and
                abs(distance - self.parking_wall_distance) <=
                self.parking_wall_tolerance and
                abs(heading) <= self.v1_close_heading_accept)
            if near_goal:
                # Do not turn the long rectangular chassis after it has
                # reached the valid stop band and locked the bay centreline.
                self.parking_heading_correction_active = False
                values = (0.0, 0.0, 0.0, distance, heading, points,
                          pixel_error, target_live, target_age)
                self._v1_last_near_goal_values = values
                self._v1_last_near_goal_stamp = now
            return values

        if (not allow_forward or not self.parking_center_aligned or
                self._v1_last_near_goal_values is None or
                now - self._v1_last_near_goal_stamp >
                self.v1_near_goal_memory_s):
            return None

        # Returning a zero command lets the unchanged v1 park() loop perform
        # its normal three-frame stability and odometry-drift checks.  This is
        # not blind motion: the robot remains stopped throughout the latch.
        self.parking_heading_correction_active = False
        if not self._v1_near_goal_latch_reported:
            cached = self._v1_last_near_goal_values
            self.publish_state(
                "V1_PARKING_NEAR_GOAL_WALL_LOSS_LATCHED",
                wall_distance=cached[3],
                wall_heading_error_deg=math.degrees(cached[4]),
                memory_s=self.v1_near_goal_memory_s,
                policy="stop_and_finish_legacy_stability_check")
            self._v1_near_goal_latch_reported = True
        return self._v1_last_near_goal_values

    def park(self):
        self._v1_last_near_goal_values = None
        self._v1_last_near_goal_stamp = 0.0
        self._v1_near_goal_latch_reported = False
        return Xunfei2026RoomDeliveryManager.park(self)


if __name__ == "__main__":
    V1ParkingDualStageRoomDeliveryV8()
    rospy.spin()
