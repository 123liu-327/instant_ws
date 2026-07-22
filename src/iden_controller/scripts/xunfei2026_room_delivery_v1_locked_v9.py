#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Src2 anchor travel with an exclusively legacy-v1, stage-locked parking.

Src2 is allowed to move between the calibrated inspection anchors only.  Once
the requested workshop is physically confirmed, the current delivery stage is
latched and handed straight to the original v1 parking entry.  There is no
extra alignment/wall-lock/pre-parking gate in this adapter and parking can
never fall back to anchor scanning.
"""

import json
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from xunfei2026_room_delivery_manager_v1 import (
    Xunfei2026RoomDeliveryManager,
    canonical_workshop,
)
from xunfei2026_room_delivery_src2_dual_stage_v5 import (
    Src2DualStageRoomDeliveryV5,
)


class V1LockedDualStageRoomDeliveryV9(Src2DualStageRoomDeliveryV5):
    def __init__(self):
        self._v1_parking_stage_locked = False
        super(V1LockedDualStageRoomDeliveryV9, self).__init__()

        # Restore the effective values used by complete_delivery_v1 after the
        # coverage/src2/dual-stage constructors have applied their own tuning.
        self.prepark_align_timeout = float(rospy.get_param(
            "~v1_locked_prepark_align_timeout_s", 10.0))
        self.parking_wall_tolerance = float(rospy.get_param(
            "~v1_locked_parking_wall_tolerance_m", 0.012))
        self.parking_target_memory_s = float(rospy.get_param(
            "~v1_locked_parking_target_memory_s", 3.0))
        self.parking_no_progress_timeout = float(rospy.get_param(
            "~parking_no_progress_timeout_s", 2.2))
        self.parking_no_progress_retries = int(rospy.get_param(
            "~parking_no_progress_retries", 2))
        self.parking_planner_stall_command = max(0.04, float(
            rospy.get_param("~parking_planner_stall_command_m", 0.12)))
        self.parking_center_filter_alpha = float(rospy.get_param(
            "~parking_center_filter_alpha", 0.65))
        self.parking_center_max_step = float(rospy.get_param(
            "~parking_center_max_step_px", 100.0))
        self.parking_center_tolerance = float(rospy.get_param(
            "~parking_center_tolerance_px", 14.0))
        self.parking_center_release_tolerance = float(rospy.get_param(
            "~parking_center_release_tolerance_px", 28.0))
        self.parking_stable_frames = int(rospy.get_param(
            "~parking_stable_frames", 3))
        self.parking_centerline_max_drift = float(rospy.get_param(
            "~parking_centerline_max_drift_m", 0.035))
        # complete_delivery_v1 does not leave local parking to a coverage/src2
        # escape controller.  The original park() owns its internal retries.
        self.parking_planner_recovery_enabled = False
        self.v1_preparking_ocr_confirm_s = max(0.5, float(rospy.get_param(
            "~v1_preparking_ocr_confirm_s", 1.0)))
        # Entry already requires five coherent stationary observations of one
        # physical sign plus the OCR vote threshold.  The final one-second
        # guard therefore needs one fresh stable frame (the RKNN period can be
        # slightly longer than one second), while a different label still
        # rejects parking immediately.
        self.v1_preparking_ocr_min_frames = max(1, int(rospy.get_param(
            "~v1_preparking_ocr_min_frames", 1)))
        self._v1_teb_client = None

        self.publish_state(
            "V1_EXCLUSIVE_PARKING_READY",
            anchor_navigation="src2_only_before_target",
            parking_core="complete_delivery_v1_exact",
            failure_policy="final_ocr_reject_rescans_parking_failure_stops",
            dual_stage=True)

    def _set_v1_parking_runtime(self):
        """Restore move_base/TEB settings used by the original v1 flow."""
        try:
            if (self._src2_move_base_client is not None and
                    self._src2_saved_recovery):
                self._src2_move_base_client.update_configuration(
                    self._src2_saved_recovery)
            from dynamic_reconfigure.client import Client
            if self._v1_teb_client is None:
                self._v1_teb_client = Client(
                    "/move_base/TebLocalPlannerROS", timeout=2.0)
            self._v1_teb_client.update_configuration({
                "max_vel_x": 0.80,
                "max_vel_x_backwards": 0.70,
                "max_vel_y": 0.80,
                "max_vel_theta": 2.00,
                "acc_lim_x": 0.50,
                "acc_lim_y": 0.50,
                "acc_lim_theta": 1.30,
            })
            self.publish_state(
                "V1_PARKING_RUNTIME_ACTIVE",
                move_base_recovery="original_v1",
                teb_profile="xunfei2026_room_teb_v1")
            return True
        except Exception as exc:
            self.publish_state(
                "V1_PARKING_RUNTIME_UNAVAILABLE", reason=str(exc))
            return False

    def _set_src2_anchor_runtime(self):
        """Reapply anchor-only settings after the first parking succeeds."""
        try:
            if self._src2_move_base_client is not None:
                self._src2_move_base_client.update_configuration({
                    "recovery_behavior_enabled": False,
                    "clearing_rotation_allowed": False,
                })
            if self._v1_teb_client is not None:
                self._v1_teb_client.update_configuration({
                    "max_vel_x": self.anchor_max_vel_x,
                    "max_vel_x_backwards": self.anchor_max_vel_backwards,
                    "max_vel_y": self.anchor_max_vel_y,
                    "max_vel_theta": self.anchor_max_vel_theta,
                    "acc_lim_x": self.anchor_acc_lim_x,
                    "acc_lim_y": self.anchor_acc_lim_y,
                    "acc_lim_theta": self.anchor_acc_lim_theta,
                })
            self.publish_state("SRC2_ANCHOR_RUNTIME_RESTORED")
        except Exception as exc:
            self.publish_state(
                "SRC2_ANCHOR_RUNTIME_RESTORE_FAILED", reason=str(exc))

    # ------------------------------------------------------------------
    # Hard dispatch boundary: no src2/coverage/dual parking implementation
    # can be reached through Python's dynamic self.* method resolution.
    # ------------------------------------------------------------------
    def align_target_for_parking(self):
        return Xunfei2026RoomDeliveryManager.align_target_for_parking(self)

    def refresh_centered_target_wall(self):
        return Xunfei2026RoomDeliveryManager.refresh_centered_target_wall(self)

    def approach_target(self):
        return Xunfei2026RoomDeliveryManager.approach_target(self)

    def handoff_directly_to_parking(self):
        return Xunfei2026RoomDeliveryManager.handoff_directly_to_parking(self)

    def front_wall_estimate(self):
        return Xunfei2026RoomDeliveryManager.front_wall_estimate(self)

    def latest_target_center(self):
        return Xunfei2026RoomDeliveryManager.latest_target_center(self)

    def target_approach_goal(self):
        return Xunfei2026RoomDeliveryManager.target_approach_goal(self)

    def prepare_target_wall_orientation(self):
        return Xunfei2026RoomDeliveryManager.prepare_target_wall_orientation(
            self)

    def parking_front_wall_estimate(self):
        return Xunfei2026RoomDeliveryManager.parking_front_wall_estimate(self)

    def parking_center_observation_valid(self, payload):
        return Xunfei2026RoomDeliveryManager.parking_center_observation_valid(
            self, payload)

    def parking_center_horizontally_complete(self, payload):
        return Xunfei2026RoomDeliveryManager.parking_center_horizontally_complete(
            self, payload)

    def locked_parking_target_center(self):
        return Xunfei2026RoomDeliveryManager.locked_parking_target_center(self)

    def parking_abort_reason(self):
        return Xunfei2026RoomDeliveryManager.parking_abort_reason(self)

    def parking_footprint_side_clearance(self, vy, wall_distance):
        return Xunfei2026RoomDeliveryManager.parking_footprint_side_clearance(
            self, vy, wall_distance)

    def parking_side_guard(self, vy, wall_distance):
        return Xunfei2026RoomDeliveryManager.parking_side_guard(
            self, vy, wall_distance)

    def parking_rotation_guard(self, wz):
        return Xunfei2026RoomDeliveryManager.parking_rotation_guard(self, wz)

    def parking_raw_sector_clearance(self, center_angle, half_angle):
        return Xunfei2026RoomDeliveryManager.parking_raw_sector_clearance(
            self, center_angle, half_angle)

    def parking_commands(self, allow_forward):
        return Xunfei2026RoomDeliveryManager.parking_commands(
            self, allow_forward)

    def planner_reposition_after_parking_stall(self, attempt):
        return Xunfei2026RoomDeliveryManager.planner_reposition_after_parking_stall(
            self, attempt)

    def park(self):
        return Xunfei2026RoomDeliveryManager.park(self)

    # ------------------------------------------------------------------
    # Parking-stage target lock and direct legacy-v1 handoff.
    # ------------------------------------------------------------------
    def ocr_callback(self, msg):
        if not self._v1_parking_stage_locked:
            return super(V1LockedDualStageRoomDeliveryV9, self).ocr_callback(msg)
        try:
            payload = json.loads(msg.data)
            label = canonical_workshop(payload.get("label", ""))
            frame_label = canonical_workshop(payload.get("frame_label", ""))
        except Exception:
            return
        with self.lock:
            target = self.target_warehouse
        if label == target and frame_label == target:
            return super(V1LockedDualStageRoomDeliveryV9, self).ocr_callback(msg)
        rospy.logwarn_throttle(
            1.0, "PARKING_TARGET_LOCKED target=%s other OCR ignored", target)

    def _confirm_target_before_parking(self):
        """Hold still for one second and require fresh stable target OCR."""
        self.smooth_stop_robot()
        self.ocr_control_pub.publish(String(data="enable"))
        self.publish_state(
            "V1_PREPARKING_OCR_CHECK", target=self.target_warehouse,
            duration_s=self.v1_preparking_ocr_confirm_s)
        started = time.monotonic()
        deadline = started + self.v1_preparking_ocr_confirm_s
        seen_stamps = set()
        matching_frames = 0
        last_match_at = 0.0
        rejected_label = ""
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            now = time.monotonic()
            with self.lock:
                payload = (None if self.latest_ocr is None else
                           dict(self.latest_ocr))
                target = self.target_warehouse
            if payload is not None:
                stamp = float(payload.get("received_monotonic", 0.0) or 0.0)
                if stamp > 0.0 and stamp not in seen_stamps:
                    seen_stamps.add(stamp)
                    label = canonical_workshop(payload.get("label", ""))
                    frame_label = canonical_workshop(
                        payload.get("frame_label", ""))
                    stable = bool(payload.get("stable", False))
                    votes = int(payload.get("votes", 0) or 0)
                    exact_target = (
                        stable and label == target and
                        frame_label == target and
                        votes >= self.required_ocr_votes)
                    if exact_target:
                        matching_frames += 1
                        last_match_at = now
                    elif (stable and label and label != target and
                          frame_label == label and
                          votes >= self.required_ocr_votes):
                        rejected_label = label
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.04)

        now = time.monotonic()
        passed = (
            not rejected_label and
            matching_frames >= self.v1_preparking_ocr_min_frames and
            now - last_match_at <= max(0.35, self.ocr_fresh_s))
        if passed:
            self.publish_state(
                "V1_PREPARKING_OCR_CONFIRMED",
                target=self.target_warehouse, frames=matching_frames)
            return True, ""
        reason = ("stable different workshop {}".format(rejected_label)
                  if rejected_label else
                  "stable target absent in final 1s")
        self.publish_state(
            "V1_PREPARKING_OCR_REJECTED", target=self.target_warehouse,
            frames=matching_frames, reason=reason)
        return False, reason

    def _retry_same_locked_target(self, deadline, source):
        """Enter the original v1 parking immediately and never resume scan.

        Commit d5429bc (the v1 flow used by the user's start script) hands a
        confirmed target directly from TARGET_FOUND to
        handoff_directly_to_parking() and park().  Keep that boundary exact:
        the newer preparking alignment, centered-wall refresh and planner
        approach must not delay or veto parking here.
        """
        self._v1_parking_stage_locked = True
        self._set_v1_parking_runtime()
        self.target_event.set()
        self.candidate_event.set()
        self.publish_state(
            "V1_PARKING_STAGE_LOCKED", target=self.target_warehouse,
            delivery_stage=self.delivery_stage, source=source,
            rescan_allowed=False)
        try:
            if rospy.is_shutdown() or time.monotonic() >= deadline:
                return False, "mission timeout before v1 parking"
            with self.lock:
                self.parking_active = False
                self.parking_wrong_label = ""
            self.parking_wrong_event.clear()
            self.ocr_control_pub.publish(String(data="enable"))
            self.smooth_stop_robot()
            self.publish_state(
                "V1_PARKING_DIRECT_START", target=self.target_warehouse,
                delivery_stage=self.delivery_stage)
            self.handoff_directly_to_parking()
            result = self.park() or "REACQUIRE"
            if result == "SUCCEEDED":
                self.publish_state(
                    "V1_PARKING_STAGE_SUCCEEDED",
                    target=self.target_warehouse,
                    delivery_stage=self.delivery_stage)
                return True, ""
            self.publish_state(
                "V1_PARKING_STAGE_FAILED", target=self.target_warehouse,
                result=result, rescan_allowed=False)
            return False, "v1 parking failed: {}".format(result)
        finally:
            self._v1_parking_stage_locked = False

    def exit_first_parking_with_planner(self):
        result = super(
            V1LockedDualStageRoomDeliveryV9,
            self).exit_first_parking_with_planner()
        if result:
            self._set_src2_anchor_runtime()
        return result

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
            confirmed, reject_reason = self._confirm_target_before_parking()
            if not confirmed:
                resume_index = self.reset_target_and_resume_wall_scan(
                    "FINAL_OCR_REJECTED: {}".format(reject_reason))
                approach_start = False
                continue
            return self._retry_same_locked_target(deadline, "anchor_scan")
        return False, "mission timeout"

    def reset_target_and_resume_wall_scan(self, reason):
        """Resume the same semantic point from a different rotatable pose."""
        rejected_index = int(self.coverage_active_index)
        resume_index = super(
            V1LockedDualStageRoomDeliveryV9,
            self).reset_target_and_resume_wall_scan(reason)
        self._src2_force_nearby_anchor_index = rejected_index
        self.publish_state(
            "PARKING_REJECTED_NEARBY_SCAN_REQUESTED",
            anchor_index=rejected_index, resume_index=resume_index)
        return resume_index

    def try_cached_second_target(self, deadline):
        """Use the cached viewpoint only until target confirmation."""
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
                reason="no reachable nearby pose")
            return False
        self.coverage_active_index = index
        self.clear_and_wait("return to cached simulation target viewpoint")
        self.publish_state(
            "SIM_TARGET_VIEWPOINT_NAVIGATION_START", anchor_index=index)
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
        confirm_deadline = min(
            deadline, time.monotonic() + self.sim_viewpoint_reconfirm_s)
        self.publish_state(
            "SIM_TARGET_VIEWPOINT_RECONFIRMING", anchor_index=index)
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
        success, reason = self._retry_same_locked_target(
            deadline, "cached_simulation_viewpoint")
        if success:
            self.publish_state(
                "SIM_TARGET_VIEWPOINT_PARKING_SUCCEEDED",
                anchor_index=index)
        else:
            self.publish_state(
                "SIM_TARGET_VIEWPOINT_PARKING_FAILED_NO_RESCAN",
                anchor_index=index, reason=reason)
        return success


if __name__ == "__main__":
    V1LockedDualStageRoomDeliveryV9()
    rospy.spin()
