#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Two-stage real-car delivery before simulation handoff.

Stage 1 parks at the real-item workshop and announces that delivery.  Stage 2
uses the still-running OCR, lidar and planner to reach the simulation item's
workshop, parks there, and publishes the sole event allowed to start Gazebo.
"""

import json
import math
import threading
import time

import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from xunfei2026_room_delivery_anchor_coverage_parking_lock_v2 import (
    ParkingLockAnchorCoverageManager,
)
from xunfei2026_room_delivery_anchor_coverage_v1 import (
    AnchorCoverageRoomDeliveryManager,
)
from xunfei2026_room_delivery_manager_v1 import canonical_workshop


class DualStageRoomDeliveryManager(ParkingLockAnchorCoverageManager):
    def __init__(self):
        self.real_selected_item = ""
        self.real_target_warehouse = ""
        self.sim_selected_item = ""
        self.sim_target_warehouse = ""
        self.delivery_stage = "real"
        self.workshop_observations = {}
        self.second_stage_started = False
        self.sim_viewpoint_candidate = None
        self.sim_viewpoint_candidate_frames = 0
        self.sim_viewpoint_cache = None
        self._dual_wall_candidate = None
        self._dual_wall_candidate_frames = 0
        self._dual_wall_candidate_stamp = 0.0
        self._dual_wall_odom_anchor = None
        self._dual_wall_anchor_distance = None
        self._dual_wall_anchor_stamp = 0.0
        self._dual_last_parking_requested_vy = 0.0
        self._dual_last_parking_pixel_error = 0.0
        super(DualStageRoomDeliveryManager, self).__init__()
        self.real_delivery_tts_wait = max(1.5, float(rospy.get_param(
            "~dual_stage_real_tts_wait_s", 4.0)))
        self.exit_anchor_timeout = max(5.0, float(rospy.get_param(
            "~dual_stage_exit_anchor_timeout_s", 12.0)))
        self.exit_anchor_attempts = max(2, int(rospy.get_param(
            "~dual_stage_exit_anchor_attempts", 4)))
        self.same_workshop_confirm_s = max(0.5, float(rospy.get_param(
            "~dual_stage_same_workshop_confirm_s", 0.8)))
        self.navigation_target_hold_enabled = bool(rospy.get_param(
            "~dual_stage_navigation_target_hold_enabled", True))
        self.move_base_release_timeout = max(0.4, float(rospy.get_param(
            "~dual_stage_move_base_release_timeout_s", 1.2)))
        # OCR boxes can alternate between a complete sign and a side fragment
        # while the chassis is already close to the wall.  Do not translate
        # for three seconds from an old centre, and hand an ineffective local
        # body-y correction to TEB sooner.
        self.parking_target_memory_s = min(
            self.parking_target_memory_s, float(rospy.get_param(
                "~dual_stage_parking_target_memory_s", 0.8)))
        self.parking_no_progress_timeout = max(
            self.parking_no_progress_timeout, float(rospy.get_param(
                "~dual_stage_parking_no_progress_timeout_s", 3.0)))
        self.parking_no_progress_retries = max(
            self.parking_no_progress_retries, max(1, int(rospy.get_param(
                "~dual_stage_parking_no_progress_retries", 2))))
        self.parking_planner_stall_command = max(
            self.parking_planner_stall_command, max(0.04, float(
                rospy.get_param(
                    "~dual_stage_parking_planner_stall_command_m", 0.12))))
        self.parking_center_filter_alpha = min(
            self.parking_center_filter_alpha, float(rospy.get_param(
                "~dual_stage_parking_center_filter_alpha", 0.22)))
        self.parking_center_max_step = min(
            self.parking_center_max_step, float(rospy.get_param(
                "~dual_stage_parking_center_max_step_px", 40.0)))
        self.parking_center_tolerance = min(
            self.parking_center_tolerance, float(rospy.get_param(
                "~dual_stage_parking_center_tolerance_px", 20.0)))
        self.parking_center_release_tolerance = min(
            self.parking_center_release_tolerance, float(rospy.get_param(
                "~dual_stage_parking_center_release_tolerance_px", 28.0)))
        # Match the proven centerline-only controller: finish the wall-angle
        # correction before lateral centering instead of blending both motions.
        self.fusion_yaw_priority = min(
            self.fusion_yaw_priority, math.radians(float(rospy.get_param(
                "~dual_stage_parking_yaw_priority_deg", 2.5))))
        self.parking_heading_tolerance = min(
            self.parking_heading_tolerance, math.radians(float(rospy.get_param(
                "~dual_stage_parking_heading_tolerance_deg", 2.5))))
        self.parking_heading_release_tolerance = min(
            self.parking_heading_release_tolerance,
            math.radians(float(rospy.get_param(
                "~dual_stage_parking_heading_release_deg", 1.5))))
        # Six lidar frames made the chassis sit in the valid distance band for
        # about a quarter second before accepting it.  At the real base's
        # braking latency that is long enough to overshoot, reverse, and start
        # another yaw correction.  Three coherent frames still reject a scan
        # spike, while latching the already-correct pose before that cycle.
        self.parking_stable_frames = min(
            self.parking_stable_frames, max(3, int(rospy.get_param(
                "~dual_stage_final_parking_stable_frames", 3))))
        self.dual_final_heading_accept = math.radians(max(
            4.0, float(rospy.get_param(
                "~dual_stage_final_heading_accept_deg", 5.5))))
        self.dual_final_slow_band = max(
            self.parking_wall_tolerance, float(rospy.get_param(
                "~dual_stage_final_slow_band_m", 0.055)))
        self.dual_final_slow_vx = max(0.02, min(
            self.parking_slow_vx, float(rospy.get_param(
                "~dual_stage_final_slow_vx_mps", 0.040))))
        self.parking_fast_vx = min(
            self.parking_fast_vx, float(rospy.get_param(
                "~dual_stage_final_fast_vx_mps", 0.10)))
        self.dual_wall_odom_bridge_s = max(0.25, min(1.5, float(
            rospy.get_param("~dual_stage_wall_odom_bridge_s", 1.0))))
        # Mecanum odometry updates in coarse position steps on the real base.
        # A 35.0 mm limit rejected the latest otherwise-valid run at 35.6 mm.
        # The final phase is already visual-center locked and forbids body-y,
        # so 50 mm remains a tight safety check without reacting to one tick.
        self.parking_centerline_max_drift = max(
            self.parking_centerline_max_drift, float(rospy.get_param(
                "~dual_stage_centerline_max_drift_m", 0.050)))
        self.recovery_target_shift_min = max(0.04, float(rospy.get_param(
            "~dual_stage_recovery_target_shift_min_m", 0.08)))
        self.recovery_target_shift_max = max(
            self.recovery_target_shift_min, float(rospy.get_param(
                "~dual_stage_recovery_target_shift_max_m", 0.18)))
        self.sim_viewpoint_confirm_frames = max(2, int(rospy.get_param(
            "~dual_stage_sim_viewpoint_confirm_frames", 3)))
        self.sim_viewpoint_sequence_s = max(0.4, float(rospy.get_param(
            "~dual_stage_sim_viewpoint_sequence_s", 1.2)))
        self.sim_viewpoint_nav_timeout = max(5.0, float(rospy.get_param(
            "~dual_stage_sim_viewpoint_nav_timeout_s", 12.0)))
        self.sim_viewpoint_reconfirm_s = max(1.0, float(rospy.get_param(
            "~dual_stage_sim_viewpoint_reconfirm_s", 3.0)))
        # Two verified parking operations share one overall watchdog.  This is
        # a safety ceiling only and does not slow either navigation stage.
        self.mission_timeout = max(self.mission_timeout, float(rospy.get_param(
            "~dual_stage_mission_timeout_s", 600.0)))

    def _wait_for_move_base_idle(self, context, result):
        """Release an accepted/cancelled goal before calling make_plan again.

        The coverage controller accepts a planner goal as soon as the robot is
        close enough.  actionlib cancellation is asynchronous, so immediately
        asking move_base/make_plan for the next escape/return route can race
        the still-ACTIVE goal and move_base rejects the external plan request.
        """
        active_states = (
            GoalStatus.PENDING, GoalStatus.ACTIVE,
            GoalStatus.PREEMPTING, GoalStatus.RECALLING)
        state_before = self.move_base.get_state()
        if state_before in active_states:
            self.move_base.cancel_goal()
        deadline = time.monotonic() + self.move_base_release_timeout
        state_after = state_before
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state_after = self.move_base.get_state()
            if state_after not in active_states:
                break
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.03)
        # Give move_base's internal state machine one callback cycle to leave
        # the planning state before an external make_plan request is issued.
        if not rospy.is_shutdown() and state_after not in active_states:
            rospy.sleep(0.12)
        self.smooth_stop_robot()
        idle = state_after not in active_states
        self.publish_state(
            "MOVE_BASE_GOAL_RELEASED", context=context, result=result,
            state_before=state_before, state_after=state_after, idle=idle)
        return idle

    def _send_anchor_goal_fast(self, x, y, yaw, name, timeout):
        result = super(
            DualStageRoomDeliveryManager, self)._send_anchor_goal_fast(
                x, y, yaw, name, timeout)
        self._wait_for_move_base_idle("anchor_navigation:" + name, result)
        return result

    def _send_parking_recovery_goal_fast(self, x, y, yaw, name, timeout):
        result = super(
            DualStageRoomDeliveryManager,
            self)._send_parking_recovery_goal_fast(
                x, y, yaw, name, timeout)
        self._wait_for_move_base_idle("parking_recovery:" + name, result)
        return result

    def _navigate_to_anchor(self, anchor):
        """Stop and verify a strong exact target even between scan anchors."""
        previous = self._coverage_scan_active
        if self.navigation_target_hold_enabled:
            self._coverage_scan_active = True
            self.publish_state(
                "OCR_NAVIGATION_TARGET_HOLD_ACTIVE",
                anchor=anchor.get("name", ""),
                confirm_frames=self.coverage_suspect_confirm_frames,
                policy="exact_stable_target_then_stationary_confirm")
        try:
            return super(
                DualStageRoomDeliveryManager, self)._navigate_to_anchor(anchor)
        finally:
            self._coverage_scan_active = previous
            self._coverage_suspect_frames = 0
            if not self.target_event.is_set():
                self.candidate_event.clear()

    def parking_front_wall_estimate(self):
        """Use the reference controller's continuous wall-fit reseeding.

        coverage-v1 kept returning the previous wall for 0.35 s after a jump,
        but never cleared that previous sample after the memory expired.  A
        genuine change caused by forward motion was therefore rejected on
        every 20 Hz control tick forever.  Confirm a new coherent fit for
        several frames, then reseed it; rejection logging is throttled so the
        ARM control loop is not flooded with JSON messages.
        """
        estimate = super(
            AnchorCoverageRoomDeliveryManager,
            self).parking_front_wall_estimate()
        now = time.monotonic()
        previous = self._fusion_last_wall
        if estimate is None:
            # Very close to the wall, a 10 Hz scan can briefly contain too few
            # points for the line fit.  During the already-locked straight final
            # approach, bridge that short gap with measured forward odometry.
            with self.lock:
                odom_pose = (None if self.odom_pose is None else
                             tuple(self.odom_pose))
            anchor = self._dual_wall_odom_anchor
            if (self.parking_center_aligned and previous is not None and
                    anchor is not None and odom_pose is not None and
                    self._dual_wall_anchor_distance is not None and
                    now - self._dual_wall_anchor_stamp <=
                    self.dual_wall_odom_bridge_s):
                dx = odom_pose[0] - anchor[0]
                dy = odom_pose[1] - anchor[1]
                forward = dx * math.cos(anchor[2]) + dy * math.sin(anchor[2])
                lateral = -dx * math.sin(anchor[2]) + dy * math.cos(anchor[2])
                predicted = self._dual_wall_anchor_distance - forward
                if (-0.05 <= forward <= 0.25 and abs(lateral) <= 0.07 and
                        0.08 <= predicted <=
                        self._dual_wall_anchor_distance + 0.05):
                    rospy.logwarn_throttle(
                        0.25, "FINAL_WALL_ODOM_BRIDGE distance=%.3f "
                        "forward=%.3f lateral=%.3f age=%.2f",
                        predicted, forward, lateral,
                        now - self._dual_wall_anchor_stamp)
                    return predicted, previous[1], previous[2]
            if (previous is not None and
                    now - self._fusion_last_wall_stamp <=
                    self.fusion_wall_memory_s):
                return previous
            return None

        # A forward command legitimately reduces wall distance faster than the
        # generic jump threshold.  Once the visual centerline is locked, accept
        # that monotonic change but retain the last good heading if the close
        # scan momentarily fits a diagonal edge.
        if self.parking_center_aligned and previous is not None:
            distance_drop = previous[0] - estimate[0]
            plausible_distance = (
                0.08 <= estimate[0] <= previous[0] + 0.04 and
                distance_drop <= 0.18)
            if plausible_distance:
                heading = estimate[1]
                if abs(heading) > self.dual_final_heading_accept:
                    heading = previous[1]
                accepted = (estimate[0], heading, estimate[2])
                self._fusion_last_wall = accepted
                self._fusion_last_wall_stamp = now
                self._dual_wall_candidate = None
                self._dual_wall_candidate_frames = 0
                with self.lock:
                    self._dual_wall_odom_anchor = (
                        None if self.odom_pose is None else
                        tuple(self.odom_pose))
                self._dual_wall_anchor_distance = accepted[0]
                self._dual_wall_anchor_stamp = now
                rospy.logwarn_throttle(
                    0.25, "FINAL_WALL_FORWARD_UPDATE old=%.3f new=%.3f "
                    "raw_heading=%.1fdeg used_heading=%.1fdeg",
                    previous[0], estimate[0], math.degrees(estimate[1]),
                    math.degrees(heading))
                return accepted

        continuous = (
            previous is None or
            (abs(estimate[0] - previous[0]) <=
             self.fusion_wall_distance_jump and
             abs(self.normalize_angle_for_wall(
                 estimate[1] - previous[1])) <=
             self.fusion_wall_heading_jump))
        if continuous:
            self._fusion_last_wall = estimate
            self._fusion_last_wall_stamp = now
            self._dual_wall_candidate = None
            self._dual_wall_candidate_frames = 0
            with self.lock:
                self._dual_wall_odom_anchor = (
                    None if self.odom_pose is None else tuple(self.odom_pose))
            self._dual_wall_anchor_distance = estimate[0]
            self._dual_wall_anchor_stamp = now
            return estimate

        candidate = self._dual_wall_candidate
        candidate_continuous = (
            candidate is not None and
            abs(estimate[0] - candidate[0]) <=
            max(0.035, self.fusion_wall_distance_jump) and
            abs(self.normalize_angle_for_wall(
                estimate[1] - candidate[1])) <=
            max(math.radians(6.0), self.fusion_wall_heading_jump))
        if candidate_continuous:
            self._dual_wall_candidate_frames += 1
        else:
            self._dual_wall_candidate = estimate
            self._dual_wall_candidate_frames = 1
            self._dual_wall_candidate_stamp = now

        if (self._dual_wall_candidate_frames >= 3 and
                now - self._fusion_last_wall_stamp >=
                self.fusion_wall_memory_s):
            self._fusion_last_wall = estimate
            self._fusion_last_wall_stamp = now
            frames = self._dual_wall_candidate_frames
            self._dual_wall_candidate = None
            self._dual_wall_candidate_frames = 0
            with self.lock:
                self._dual_wall_odom_anchor = (
                    None if self.odom_pose is None else tuple(self.odom_pose))
            self._dual_wall_anchor_distance = estimate[0]
            self._dual_wall_anchor_stamp = now
            self.publish_state(
                "FUSION_PARKING_WALL_RESEEDED",
                previous_distance=previous[0], distance=estimate[0],
                previous_heading=previous[1], heading=estimate[1],
                confirm_frames=frames)
            return estimate

        rospy.logwarn_throttle(
            0.5,
            "FUSION_PARKING_WALL_CANDIDATE old=(%.3f,%.1fdeg) "
            "new=(%.3f,%.1fdeg) frames=%d/3",
            previous[0], math.degrees(previous[1]), estimate[0],
            math.degrees(estimate[1]), self._dual_wall_candidate_frames)
        if (now - self._fusion_last_wall_stamp <=
                self.fusion_wall_memory_s):
            return previous
        return None

    @staticmethod
    def normalize_angle_for_wall(value):
        return math.atan2(math.sin(value), math.cos(value))

    def parking_commands(self, allow_forward):
        """Latch a valid final pose before braking latency starts a new cycle."""
        centerline_locked = allow_forward and self.parking_center_aligned
        saved_heading_tolerance = self.parking_heading_tolerance
        saved_final_heading_gate = self.parking_final_heading_gate
        if centerline_locked:
            # Once phase 1 has locked the visual centreline, small close-range
            # lidar heading fluctuations must not create a translate+rotate
            # arc.  Widen only this one command calculation; phase-1 alignment
            # retains the original strict wall-angle limits.
            self.parking_heading_correction_active = False
            self.parking_heading_tolerance = max(
                saved_heading_tolerance, self.dual_final_heading_accept)
            self.parking_final_heading_gate = max(
                saved_final_heading_gate, self.dual_final_heading_accept)
        try:
            values = super(
                DualStageRoomDeliveryManager, self).parking_commands(
                    allow_forward)
        finally:
            self.parking_heading_tolerance = saved_heading_tolerance
            self.parking_final_heading_gate = saved_final_heading_gate
        if values is None:
            return None
        (vx, vy, wz, distance, heading, points, pixel_error,
         target_live, target_age) = values

        # These are deliberately refreshed on every parking control sample.
        # NO_PROGRESS_REACQUIRE does not populate the legacy stall fields, so
        # using those alone can shift a recovery goal in the direction left by
        # the previous (real-item) parking run.
        self._dual_last_parking_requested_vy = self.parking_lateral_requested
        self._dual_last_parking_pixel_error = pixel_error

        if allow_forward and self.parking_center_aligned:
            distance_error = distance - self.parking_wall_distance
            if abs(heading) <= self.dual_final_heading_accept:
                # The sign centerline is already locked.  A 4--5 degree lidar
                # fluctuation must not restart rotation and bend the final
                # approach away from that physical line.
                self.parking_heading_correction_active = False
                vy = 0.0
                wz = 0.0
            else:
                # A material wall-angle error is corrected in place.  Never
                # combine that correction with forward motion.
                vx = 0.0
                vy = 0.0
            if (abs(distance_error) <= self.parking_wall_tolerance and
                    abs(heading) <= self.dual_final_heading_accept):
                vx = 0.0
            elif (0.0 < distance_error <= self.dual_final_slow_band and
                  vx > 0.0):
                # Brake early enough that the delayed base feedback cannot
                # carry the robot through the complete distance window.
                vx = min(vx, self.dual_final_slow_vx)
            elif (distance_error < -self.parking_wall_tolerance and
                  abs(heading) <= self.dual_final_heading_accept):
                # If an old run is already slightly too close, retreat only
                # along the locked centerline; never combine it with a turn.
                vy = 0.0
                wz = 0.0

        return (vx, vy, wz, distance, heading, points, pixel_error,
                target_live, target_age)

    def park(self):
        # Recovery direction belongs to this parking attempt only.
        self._dual_last_parking_requested_vy = 0.0
        self._dual_last_parking_pixel_error = 0.0
        self._dual_wall_odom_anchor = None
        self._dual_wall_anchor_distance = None
        self._dual_wall_anchor_stamp = 0.0
        return super(DualStageRoomDeliveryManager, self).park()

    def planner_reposition_after_parking_stall(self, attempt):
        """Escape through a safe anchor, then return with a planner shift.

        The coverage-v1 recovery safely leaves the cone cluster, but returns
        to the exact same inspection anchor.  When a local lateral command did
        not move the chassis this recreates the same visual error indefinitely.
        Preserve the safe-anchor escape and apply the measured centring side to
        the planner-controlled return pose; no direct escape translation is
        introduced.
        """
        if not self.coverage_far_anchor_parking_recovery:
            return AnchorCoverageRoomDeliveryManager.planner_reposition_after_parking_stall(
                self, attempt)
        self.smooth_stop_robot()
        self.cone_control_pub.publish(Bool(data=True))
        self.ocr_control_pub.publish(String(data="enable"))
        count = len(self.coverage_anchors)
        active = max(0, min(self.coverage_active_index, count - 1))
        neighbor_indices = []
        for offset in range(1, count):
            for index in (active - offset, active + offset):
                if 0 <= index < count and index not in neighbor_indices:
                    neighbor_indices.append(index)
            if len(neighbor_indices) >= self.coverage_parking_escape_anchors:
                break

        target_anchor = dict(self.coverage_anchors[active])
        if active < len(self.wall_route_yaws):
            target_anchor["yaw"] = self.wall_route_yaws[active]
        requested_vy = self._dual_last_parking_requested_vy
        pixel_error = self._dual_last_parking_pixel_error
        if (abs(requested_vy) < 1.0e-4 and abs(pixel_error) > 1.0 and
                not self.parking_center_aligned):
            requested_vy = -pixel_error
        shift = 0.0
        if abs(requested_vy) >= 1.0e-4:
            pixel_ratio = min(1.0, abs(pixel_error) / 400.0)
            shift = (self.recovery_target_shift_min + pixel_ratio *
                     (self.recovery_target_shift_max -
                      self.recovery_target_shift_min))
            shift = min(
                self.recovery_target_shift_max,
                shift * (1.0 + 0.20 * max(0, attempt - 1)))
            lateral_sign = 1.0 if requested_vy >= 0.0 else -1.0
            tangent_x = -math.sin(target_anchor["yaw"])
            tangent_y = math.cos(target_anchor["yaw"])
            target_anchor["x"] += lateral_sign * shift * tangent_x
            target_anchor["y"] += lateral_sign * shift * tangent_y
            margin = self.coverage_boundary_margin
            target_anchor["x"] = min(
                self.room_max_x - margin,
                max(self.room_min_x + margin, target_anchor["x"]))
            target_anchor["y"] = min(
                self.room_max_y - margin,
                max(self.room_min_y + margin, target_anchor["y"]))

        self.publish_state(
            "PARKING_RECOVERY_PLANNER_SHIFT_PREPARED", attempt=attempt,
            target_anchor=target_anchor["name"],
            shifted_x=target_anchor["x"], shifted_y=target_anchor["y"],
            shift=shift, requested_vy=requested_vy,
            pixel_error=pixel_error, direct_lateral_escape=False)
        attempted = 0
        self.clear_and_wait("anchor parking planner escape")
        for neighbor_index in neighbor_indices:
            escape_anchor = self.coverage_anchors[neighbor_index]
            selected = self._nearest_scan_pose(escape_anchor)
            if selected is None:
                continue
            attempted += 1
            self.publish_state(
                "PARKING_RECOVERY_SAFE_ANCHOR_SELECTED", attempt=attempt,
                anchor=escape_anchor["name"],
                anchor_index=neighbor_index,
                target_anchor=target_anchor["name"])
            result = self._send_parking_recovery_goal_fast(
                selected[0], selected[1], selected[2],
                "parking_escape_{}_{}".format(attempt, neighbor_index),
                self.coverage_parking_escape_timeout)
            if result != "SUCCEEDED":
                self.publish_state(
                    "PARKING_RECOVERY_SAFE_ANCHOR_FAILED", attempt=attempt,
                    anchor=escape_anchor["name"], result=result)
                self.clear_and_wait(
                    "parking escape {} {}".format(
                        escape_anchor["name"], result))
                continue

            selected_target = self._nearest_scan_pose(target_anchor)
            if selected_target is None:
                self.publish_state(
                    "PARKING_RECOVERY_SHIFTED_RETURN_UNREACHABLE",
                    attempt=attempt, target_anchor=target_anchor["name"],
                    shift=shift)
                continue
            result = self._send_parking_recovery_goal_fast(
                selected_target[0], selected_target[1], selected_target[2],
                "parking_shifted_target_return_{}".format(attempt),
                self.coverage_parking_return_timeout)
            if result != "SUCCEEDED":
                self.publish_state(
                    "PARKING_RECOVERY_SHIFTED_RETURN_FAILED",
                    attempt=attempt, result=result,
                    target_anchor=target_anchor["name"], shift=shift)
                continue

            # Never carry the box centre that caused the stall into the new
            # observation pose.  The target identity and wall lock stay valid;
            # only the visual centre is freshly measured.
            with self.lock:
                self.parking_center_filtered = None
                self.parking_center_width = 0.0
                self.parking_center_source_stamp = 0.0
            if not self.align_target_for_parking():
                self.publish_state(
                    "PARKING_RECOVERY_ALIGNMENT_FAILED", attempt=attempt)
                continue
            if attempt == 0:
                # PREPARK alignment recovery is called from
                # align_target_for_parking().  Its caller performs the wall
                # lock and planned approach next, so doing them here as well
                # produced two consecutive stop/start pre-parking cycles.
                self.publish_state(
                    "PARKING_RECOVERY_ALIGNMENT_ONLY_REACQUIRED",
                    attempt=attempt, escape_anchor=escape_anchor["name"],
                    target_anchor=target_anchor["name"], shift=shift,
                    next="single_wall_lock_and_preparking")
                return True
            if not self.refresh_centered_target_wall():
                self.publish_state(
                    "PARKING_RECOVERY_WALL_LOCK_FAILED", attempt=attempt)
                continue
            if not self.approach_target():
                self.publish_state(
                    "PARKING_RECOVERY_PREPARK_FAILED", attempt=attempt)
                continue
            self.publish_state(
                "PARKING_RECOVERY_SHIFTED_REACQUIRED", attempt=attempt,
                escape_anchor=escape_anchor["name"],
                target_anchor=target_anchor["name"], shift=shift)
            return True

        self.publish_state(
            "PARKING_RECOVERY_NO_SAFE_ANCHOR", attempt=attempt,
            attempted=attempted, shifted_return=True)
        return False

    def result_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return super(DualStageRoomDeliveryManager, self).result_callback(
                msg)
        if str(payload.get("status", "")).lower() == "success":
            real_item = str(payload.get("selected_item", "")).strip()
            real_warehouse = canonical_workshop(
                payload.get("target_warehouse", ""))
            sim_item = str(payload.get("sim_selected_item", "")).strip()
            sim_warehouse = canonical_workshop(
                payload.get("sim_target_warehouse", ""))
            if all((real_item, real_warehouse, sim_item, sim_warehouse)):
                with self.lock:
                    self.real_selected_item = real_item
                    self.real_target_warehouse = real_warehouse
                    self.sim_selected_item = sim_item
                    self.sim_target_warehouse = sim_warehouse
        return super(DualStageRoomDeliveryManager, self).result_callback(msg)

    def _sim_viewpoint_observation(self, payload, label, frame_label,
                                   stable, votes):
        """Return a reproducible centred view of the simulation workshop."""
        with self.lock:
            sim_target = self.sim_target_warehouse
            stage = self.delivery_stage
        if (stage != "real" or not sim_target or label != sim_target or
                frame_label != sim_target or not stable or
                votes < self.required_ocr_votes):
            return None
        bbox = payload.get("bbox")
        width = float(payload.get("image_width", 0) or 0)
        height = float(payload.get("image_height", 0) or 0)
        if (not isinstance(bbox, (list, tuple)) or len(bbox) < 4 or
                width <= 1.0 or height <= 1.0):
            return None
        try:
            left, top, right, bottom = [float(value) for value in bbox[:4]]
        except (TypeError, ValueError):
            return None
        margin_x = self.target_edge_margin_ratio * width
        visible_height = max(0.0, min(bottom, height) - max(top, 0.0))
        visible_width = max(0.0, min(right, width) - max(left, 0.0))
        center = 0.5 * (left + right)
        center_error = center - 0.5 * width
        horizontally_complete = (
            left >= margin_x and right <= width - margin_x)
        usable = (
            horizontally_complete and
            visible_width >= max(48.0, 0.05 * width) and
            visible_height >= max(
                18.0, self.target_min_visible_height_ratio * height))
        centered = (
            abs(center_error) <= self.target_handoff_center_ratio * width)
        if not usable or not centered:
            return None
        pose = self.current_pose()
        if pose is None:
            return None
        # Lower quality is better: centre error dominates, then prefer a
        # larger physical sign and stronger vote count.
        quality = (abs(center_error) / width -
                   0.08 * min(1.0, visible_width / width) -
                   0.002 * min(votes, 12))
        return {
            "label": label, "x": pose[0], "y": pose[1], "yaw": pose[2],
            "anchor_index": int(getattr(self, "coverage_active_index", 0)),
            "votes": votes, "bbox": list(bbox[:4]),
            "center_error_px": center_error, "quality": quality,
            "stamp": time.time(), "monotonic": time.monotonic(),
        }

    def _update_sim_viewpoint_cache(self, payload, label, frame_label,
                                    stable, votes):
        observation = self._sim_viewpoint_observation(
            payload, label, frame_label, stable, votes)
        if observation is None:
            return
        candidate = self.sim_viewpoint_candidate
        same_sequence = (
            candidate is not None and
            observation["monotonic"] - candidate["last_monotonic"] <=
            self.sim_viewpoint_sequence_s and
            math.hypot(observation["x"] - candidate["last_x"],
                       observation["y"] - candidate["last_y"]) <= 0.15 and
            abs(self.normalize_angle_for_wall(
                observation["yaw"] - candidate["last_yaw"])) <=
            math.radians(20.0))
        if same_sequence:
            self.sim_viewpoint_candidate_frames += 1
            best = candidate["best"]
            if observation["quality"] < best["quality"]:
                candidate["best"] = observation
            candidate["last_x"] = observation["x"]
            candidate["last_y"] = observation["y"]
            candidate["last_yaw"] = observation["yaw"]
            candidate["last_monotonic"] = observation["monotonic"]
        else:
            self.sim_viewpoint_candidate_frames = 1
            self.sim_viewpoint_candidate = {
                "best": observation,
                "last_x": observation["x"],
                "last_y": observation["y"],
                "last_yaw": observation["yaw"],
                "last_monotonic": observation["monotonic"],
            }

        if self.sim_viewpoint_candidate_frames < self.sim_viewpoint_confirm_frames:
            return
        best = dict(self.sim_viewpoint_candidate["best"])
        best["confirm_frames"] = self.sim_viewpoint_candidate_frames
        previous = self.sim_viewpoint_cache
        if previous is not None and previous["quality"] <= best["quality"]:
            return
        with self.lock:
            self.sim_viewpoint_cache = best
        self.publish_state(
            "SIM_TARGET_VIEWPOINT_CACHED", sim_target=best["label"],
            x=best["x"], y=best["y"], yaw=best["yaw"],
            anchor_index=best["anchor_index"], votes=best["votes"],
            center_error_px=best["center_error_px"],
            confirm_frames=best["confirm_frames"],
            policy="continue_real_target_scan_without_parking")

    def ocr_callback(self, msg):
        """Cache every reliable workshop view while preserving v2 filtering."""
        try:
            payload = json.loads(msg.data)
            label = canonical_workshop(payload.get("label", ""))
            frame_label = canonical_workshop(payload.get("frame_label", ""))
            stable = bool(payload.get("stable", False))
            votes = int(payload.get("votes", 0) or 0)
        except Exception:
            label = ""
            frame_label = ""
            stable = False
            votes = 0
            payload = {}
        if (hasattr(self, "lock") and
                hasattr(self, "required_ocr_votes") and
                hasattr(self, "target_handoff_center_ratio")):
            self._update_sim_viewpoint_cache(
                payload, label, frame_label, stable, votes)
        if (stable and label and frame_label == label and
                votes >= int(getattr(self, "required_ocr_votes", 8))):
            pose = self.current_pose() if hasattr(self, "lock") else None
            if pose is not None:
                observation = {
                    "label": label,
                    "x": pose[0], "y": pose[1], "yaw": pose[2],
                    "anchor_index": int(getattr(
                        self, "coverage_active_index", 0)),
                    "votes": votes, "bbox": payload.get("bbox"),
                    "stamp": time.time(),
                }
                with self.lock:
                    previous = self.workshop_observations.get(label)
                    if previous is None or votes >= previous.get("votes", 0):
                        self.workshop_observations[label] = observation
        return super(DualStageRoomDeliveryManager, self).ocr_callback(msg)

    def wait_stopped(self, duration, state, **values):
        self.publish_state(state, wait_s=duration, **values)
        deadline = time.monotonic() + max(0.0, duration)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self.cmd_pub.publish(Twist())
            rate.sleep()

    def clear_target_lock_for_second_stage(self):
        with self.lock:
            self.selected_item = self.sim_selected_item
            self.target_warehouse = self.sim_target_warehouse
            self.latest_ocr = None
            self.target_snapshot = None
            self.parking_target_wall_yaw = None
            self.parking_target_ocr = None
            self.parking_target_stamp = 0.0
            self.parking_center_filtered = None
            self.parking_center_width = 0.0
            self.parking_center_source_stamp = 0.0
            self.parking_center_aligned = False
            self.parking_center_lock_odom_pose = None
            self.parking_wrong_label = ""
            self.non_target_view_label = ""
            self.non_target_view_anchor = None
            self.non_target_odom_anchor = None
            self.non_target_view_locked_at = 0.0
            self.non_target_target_frames = 0
            self.non_target_blank_since = None
            self.target_confirm_count = 0
            self.candidate_cooldown_until = 0.0
            self._coverage_suspect_frames = 0
            self._coverage_stationary_target_frames = 0
            self._fusion_last_wall = None
            self._fusion_last_wall_stamp = 0.0
        self.target_event.clear()
        self.candidate_event.clear()
        self.parking_wrong_event.clear()
        # Reset clears only the voting history; OCR stays enabled continuously.
        self.ocr_control_pub.publish(String(data="reset"))
        self.publish_state(
            "SECOND_TARGET_ACTIVATED", sim_item=self.sim_selected_item,
            sim_warehouse=self.sim_target_warehouse,
            ocr="continuous_reset_votes_only")

    def cached_second_start_index(self):
        with self.lock:
            observation = (self.sim_viewpoint_cache or
                           self.workshop_observations.get(
                               self.sim_target_warehouse))
            pose = self.current_pose()
        if observation is not None:
            index = max(0, min(
                int(observation.get("anchor_index", 0)),
                len(self.coverage_anchors) - 1))
            self.publish_state(
                "SECOND_TARGET_CACHE_SELECTED",
                cached_anchor=self.coverage_anchors[index]["name"],
                cached_x=observation.get("x"),
                cached_y=observation.get("y"),
                cached_yaw=observation.get("yaw"),
                observation_age_s=max(
                    0.0, time.time() - observation.get("stamp", time.time())))
            return index
        if pose is None:
            return 0
        index = min(
            range(len(self.coverage_anchors)),
            key=lambda value: math.hypot(
                self.coverage_anchors[value]["x"] - pose[0],
                self.coverage_anchors[value]["y"] - pose[1]))
        self.publish_state(
            "SECOND_TARGET_NEAREST_ANCHOR_SELECTED",
            anchor=self.coverage_anchors[index]["name"],
            anchor_index=index)
        return index

    def try_cached_second_target(self, deadline):
        """Planner-return to the saved camera pose, then run normal parking."""
        with self.lock:
            cached = (None if self.sim_viewpoint_cache is None else
                      dict(self.sim_viewpoint_cache))
        if cached is None:
            self.publish_state("SIM_TARGET_VIEWPOINT_CACHE_UNAVAILABLE")
            return False
        index = max(0, min(
            int(cached.get("anchor_index", 0)),
            len(self.coverage_anchors) - 1))
        anchor = {
            "name": "sim_cached_viewpoint",
            "display_name": "仿真目标已缓存观察位姿",
            "x": cached["x"], "y": cached["y"], "yaw": cached["yaw"],
            "sweeps": [],
        }
        selected = self._nearest_scan_pose(anchor)
        if selected is None:
            self.publish_state(
                "SIM_TARGET_VIEWPOINT_BLOCKED_FALLBACK_SCAN",
                cached_x=cached["x"], cached_y=cached["y"],
                cached_yaw=cached["yaw"], reason="no reachable nearby pose")
            return False
        self.coverage_active_index = index
        self.clear_and_wait("return to cached simulation target viewpoint")
        self.publish_state(
            "SIM_TARGET_VIEWPOINT_NAVIGATION_START",
            cached_x=cached["x"], cached_y=cached["y"],
            cached_yaw=cached["yaw"], selected_x=selected[0],
            selected_y=selected[1], selected_yaw=selected[2],
            anchor_index=index)
        result = self._send_parking_recovery_goal_fast(
            selected[0], selected[1], selected[2],
            "sim_cached_viewpoint_return", self.sim_viewpoint_nav_timeout)
        if result != "SUCCEEDED":
            self.publish_state(
                "SIM_TARGET_VIEWPOINT_NAVIGATION_FAILED_FALLBACK_SCAN",
                result=result, anchor_index=index)
            return False

        previous_scan_active = self._coverage_scan_active
        self._coverage_scan_active = True
        self.candidate_event.clear()
        self._coverage_suspect_frames = 0
        self.publish_state(
            "SIM_TARGET_VIEWPOINT_RECONFIRMING",
            wait_s=self.sim_viewpoint_reconfirm_s,
            target=self.sim_target_warehouse)
        confirm_deadline = min(
            deadline, time.monotonic() + self.sim_viewpoint_reconfirm_s)
        try:
            rate = rospy.Rate(20)
            while (not rospy.is_shutdown() and
                   time.monotonic() < confirm_deadline and
                   not self.target_event.is_set()):
                if self.candidate_event.is_set():
                    if self.hold_ocr_candidate("sim_cached_viewpoint"):
                        break
                self.cmd_pub.publish(Twist())
                rate.sleep()
        finally:
            self._coverage_scan_active = previous_scan_active
            self._coverage_suspect_frames = 0

        if not self.target_event.is_set():
            self.publish_state(
                "SIM_TARGET_VIEWPOINT_RECONFIRM_FAILED_FALLBACK_SCAN",
                anchor_index=index)
            self.candidate_event.clear()
            return False

        self.publish_state(
            "TARGET_FOUND", target=self.target_warehouse,
            delivery_stage=self.delivery_stage,
            source="cached_simulation_viewpoint")
        if (not self.align_target_for_parking() or
                not self.refresh_centered_target_wall() or
                not self.approach_target()):
            self.publish_state(
                "SIM_TARGET_VIEWPOINT_PREPARK_FAILED_FALLBACK_SCAN",
                anchor_index=index)
            self.reset_target_and_resume_wall_scan("CACHE_REACQUIRE")
            return False

        self.handoff_directly_to_parking()
        parking_result = self.park()
        planner_attempt = 0
        while (parking_result == "REPLAN" and
               planner_attempt < self.parking_planner_recovery_attempts and
               not rospy.is_shutdown()):
            planner_attempt += 1
            if not self.planner_reposition_after_parking_stall(
                    planner_attempt):
                parking_result = "REACQUIRE"
                break
            self.handoff_directly_to_parking()
            parking_result = self.park()
        if parking_result == "SUCCEEDED":
            self.publish_state(
                "SIM_TARGET_VIEWPOINT_PARKING_SUCCEEDED",
                anchor_index=index, planner_attempts=planner_attempt)
            return True
        self.publish_state(
            "SIM_TARGET_VIEWPOINT_PARKING_FAILED_FALLBACK_SCAN",
            anchor_index=index, result=parking_result)
        self.reset_target_and_resume_wall_scan(
            parking_result if parking_result else "CACHE_REACQUIRE")
        return False

    def exit_first_parking_with_planner(self):
        """Leave the first bay only through TEB and known inspection anchors."""
        self.smooth_stop_robot()
        self.cone_control_pub.publish(Bool(data=True))
        self.target_event.clear()
        self.candidate_event.clear()
        pose = self.current_pose()
        if pose is None:
            self.publish_state(
                "SECOND_STAGE_EXIT_FAILED", reason="current pose unavailable")
            return False
        candidates = sorted(
            range(len(self.coverage_anchors)),
            key=lambda index: math.hypot(
                self.coverage_anchors[index]["x"] - pose[0],
                self.coverage_anchors[index]["y"] - pose[1]))
        attempted = 0
        for index in candidates:
            if attempted >= self.exit_anchor_attempts:
                break
            anchor = self.coverage_anchors[index]
            if math.hypot(anchor["x"] - pose[0], anchor["y"] - pose[1]) < 0.28:
                continue
            selected = self._nearest_scan_pose(anchor)
            if selected is None:
                continue
            attempted += 1
            self.publish_state(
                "SECOND_STAGE_PLANNER_EXIT_START", attempt=attempted,
                anchor=anchor["name"], x=selected[0], y=selected[1],
                yaw=selected[2])
            self.clear_and_wait(
                "dual-stage exit to {}".format(anchor["name"]))
            result = self._send_parking_recovery_goal_fast(
                selected[0], selected[1], selected[2],
                "dual_stage_exit_{}".format(anchor["name"]),
                self.exit_anchor_timeout)
            if result == "SUCCEEDED":
                self.coverage_active_index = index
                self.publish_state(
                    "SECOND_STAGE_SAFE_ANCHOR_REACHED",
                    anchor=anchor["name"], attempts=attempted)
                return True
            self.publish_state(
                "SECOND_STAGE_EXIT_RETRY", anchor=anchor["name"],
                attempt=attempted, result=result)
        self.smooth_stop_robot()
        self.publish_state(
            "SECOND_STAGE_EXIT_FAILED", reason="no reachable safe anchor",
            attempts=attempted)
        return False

    def search_and_park_target(self, deadline, start_index=0, allow_wrap=False):
        resume_index = max(0, int(start_index))
        approach_start = True
        wrapped = False
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            route_result = self.run_wall_route(
                start_index=resume_index, approach_start=approach_start)
            found = route_result == "TARGET" or self.target_event.is_set()
            if not found:
                if allow_wrap and not wrapped and resume_index > 0:
                    wrapped = True
                    resume_index = 0
                    approach_start = False
                    self.publish_state("SECOND_TARGET_SEARCH_WRAPPED_TO_D1")
                    continue
                return False, "inspection route completed without target"

            self.publish_state(
                "TARGET_FOUND", target=self.target_warehouse,
                delivery_stage=self.delivery_stage)
            if not self.align_target_for_parking():
                resume_index = self.reset_target_and_resume_wall_scan(
                    "REACQUIRE")
                approach_start = False
                continue
            if not self.refresh_centered_target_wall():
                resume_index = self.reset_target_and_resume_wall_scan(
                    "REACQUIRE")
                approach_start = False
                continue
            if not self.approach_target():
                self.publish_state(
                    "PLANNED_PREPARK_REACQUIRE_NO_DIRECT_FALLBACK")
                resume_index = self.reset_target_and_resume_wall_scan(
                    "REACQUIRE")
                approach_start = False
                continue
            self.handoff_directly_to_parking()
            parking_result = self.park()
            planner_attempt = 0
            while (parking_result == "REPLAN" and
                   planner_attempt < self.parking_planner_recovery_attempts and
                   not rospy.is_shutdown()):
                planner_attempt += 1
                if not self.planner_reposition_after_parking_stall(
                        planner_attempt):
                    parking_result = "REACQUIRE"
                    break
                self.handoff_directly_to_parking()
                parking_result = self.park()
            if parking_result == "SUCCEEDED":
                return True, ""
            if parking_result == "REPLAN":
                parking_result = "REACQUIRE"
            if (parking_result.startswith("WRONG_WORKSHOP:") or
                    parking_result in ("REACQUIRE", "TIMEOUT")):
                resume_index = self.reset_target_and_resume_wall_scan(
                    parking_result)
                if resume_index >= len(self.wall_route_points) - 1:
                    resume_index = max(0, len(self.wall_route_points) - 2)
                approach_start = False
                continue
            return False, "parking result {}".format(parking_result)
        return False, "mission timeout"

    def announce_real_delivery(self):
        text = "已将{}放入{}".format(
            self.real_selected_item, self.real_target_warehouse)
        self.stop_robot(20)
        self.tts_pub.publish(String(data=text))
        self.publish_state(
            "REAL_DELIVERY_ANNOUNCED", announcement=text,
            real_item=self.real_selected_item,
            real_warehouse=self.real_target_warehouse,
            simulation_triggered=False)
        self.wait_stopped(
            self.real_delivery_tts_wait, "WAITING_REAL_DELIVERY_TTS_COMPLETE",
            announcement=text)

    def publish_second_parking_success(self, same_workshop=False):
        self.stop_robot(20)
        self.publish_state(
            "SIM_TARGET_PARKED", parking_success=True,
            same_workshop=bool(same_workshop),
            real_item=self.real_selected_item,
            real_warehouse=self.real_target_warehouse,
            sim_item=self.sim_selected_item,
            sim_warehouse=self.sim_target_warehouse,
            simulation_trigger_authorized=True)
        self.finished = True

    def mission_thread(self):
        deadline = time.monotonic() + self.mission_timeout
        ocr_start = {"error": None}
        ocr_thread = None
        failure_reason = ""

        def preload_ocr():
            try:
                self.start_ocr()
            except Exception as exc:
                ocr_start["error"] = exc

        try:
            if not all((self.real_selected_item, self.real_target_warehouse,
                        self.sim_selected_item, self.sim_target_warehouse)):
                raise RuntimeError("dual target order is incomplete")
            self.publish_state(
                "DUAL_STAGE_MISSION_ACCEPTED",
                real_item=self.real_selected_item,
                real_warehouse=self.real_target_warehouse,
                sim_item=self.sim_selected_item,
                sim_warehouse=self.sim_target_warehouse)
            self.publish_state("WAITING_INITIAL_TTS", wait_s=self.wait_after_tts)
            rospy.sleep(max(0.0, self.wait_after_tts))
            ocr_thread = threading.Thread(target=preload_ocr)
            ocr_thread.daemon = True
            ocr_thread.start()
            if not self.enter_room_with_first_stage_navigation():
                raise RuntimeError("first-stage room entry not crossed")
            ocr_thread.join(self.ocr_ready_timeout + 2.0)
            if ocr_thread.is_alive():
                raise RuntimeError("OCR preload did not finish")
            if ocr_start["error"] is not None:
                raise ocr_start["error"]
            self.replace_move_base()
            self.room_search_active = True
            self.ocr_control_pub.publish(String(data="reset"))
            self.ocr_control_pub.publish(String(data="enable"))

            self.delivery_stage = "real"
            real_success, failure_reason = self.search_and_park_target(
                deadline, start_index=0, allow_wrap=False)
            if not real_success:
                raise RuntimeError("real delivery failed: {}".format(
                    failure_reason))
            self.publish_state(
                "REAL_TARGET_PARKED", parking_success=True,
                real_item=self.real_selected_item,
                real_warehouse=self.real_target_warehouse)
            self.announce_real_delivery()

            self.delivery_stage = "simulation_target"
            self.second_stage_started = True
            same_workshop = (
                self.real_target_warehouse == self.sim_target_warehouse)
            if same_workshop:
                self.clear_target_lock_for_second_stage()
                self.wait_stopped(
                    self.same_workshop_confirm_s,
                    "SECOND_TARGET_SAME_WORKSHOP_RECONFIRMING",
                    sim_warehouse=self.sim_target_warehouse)
                self.publish_second_parking_success(same_workshop=True)
                return

            if not self.exit_first_parking_with_planner():
                raise RuntimeError(
                    "second-stage planner could not leave first parking bay")
            self.clear_target_lock_for_second_stage()
            start_index = self.cached_second_start_index()
            self.coverage_active_index = start_index
            second_success = self.try_cached_second_target(deadline)
            if second_success:
                failure_reason = ""
            else:
                self.publish_state(
                    "SECOND_TARGET_CACHE_FALLBACK_TO_COVERAGE",
                    start_index=start_index,
                    anchor=self.coverage_anchors[start_index]["name"])
                second_success, failure_reason = self.search_and_park_target(
                    deadline, start_index=start_index, allow_wrap=True)
            if not second_success:
                raise RuntimeError("simulation target parking failed: {}".format(
                    failure_reason))
            self.publish_second_parking_success(same_workshop=False)
        except Exception as exc:
            failure_reason = str(exc)
            rospy.logerr("XUNFEI2026_DUAL_STAGE_EXCEPTION %s", exc)
            self.smooth_stop_robot()
            self.publish_state(
                "DUAL_STAGE_FAILED", parking_success=False,
                failed_stage=self.delivery_stage, reason=failure_reason,
                simulation_trigger_authorized=False)
        finally:
            if ocr_thread is not None and ocr_thread.is_alive():
                ocr_thread.join(self.ocr_ready_timeout + 2.0)
            self.room_search_active = False
            self.cone_control_pub.publish(Bool(data=False))
            self.ocr_control_pub.publish(String(data="disable"))


if __name__ == "__main__":
    DualStageRoomDeliveryManager()
    rospy.spin()
