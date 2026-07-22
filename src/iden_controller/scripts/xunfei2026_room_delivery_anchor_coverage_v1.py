#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Full room delivery with ordered, obstacle-aware inspection anchors.

Only the large-room search policy is replaced here.  Speech, OCR target
confirmation, target approach, improved parking and simulation handoff remain
provided by the current complete-delivery implementation.
"""

import json
import math
import time

import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Bool, String

from xunfei2026_room_delivery_manager_v1 import (
    Xunfei2026RoomDeliveryManager,
    canonical_workshop,
    norm_angle,
)


class AnchorCoverageRoomDeliveryManager(Xunfei2026RoomDeliveryManager):
    """Inspect all nine semantic anchors around the room perimeter."""

    def __init__(self):
        # A latched task result can arrive while the base constructor is still
        # subscribing.  Defer it until the replacement route is complete.
        self._coverage_ready = False
        self._pending_result = None
        self._coverage_scan_active = False
        super(AnchorCoverageRoomDeliveryManager, self).__init__()

        self.coverage_goal_timeout = float(rospy.get_param(
            "~coverage_anchor_goal_timeout_s", 25.0))
        self.coverage_goal_attempts = max(1, int(rospy.get_param(
            "~coverage_anchor_goal_attempts", 2)))
        self.coverage_scan_speed = max(0.12, min(0.50, float(
            rospy.get_param("~coverage_continuous_scan_speed_rps", 0.35))))
        self.coverage_scan_tolerance = math.radians(max(
            1.0, min(4.0, float(rospy.get_param(
                "~coverage_continuous_scan_tolerance_deg", 2.0)))))
        self.coverage_scan_timeout_scale = max(1.5, float(rospy.get_param(
            "~coverage_continuous_scan_timeout_scale", 2.4)))
        self.coverage_dwell_s = max(0.0, float(rospy.get_param(
            "~coverage_anchor_settle_s", 0.15)))
        self.coverage_fast_stuck_s = max(1.5, float(rospy.get_param(
            "~coverage_anchor_stuck_s", 3.0)))
        self.coverage_fast_progress_m = max(0.015, float(rospy.get_param(
            "~coverage_anchor_progress_m", 0.03)))
        self.coverage_anchor_acceptance = max(0.08, min(0.60, float(
            rospy.get_param("~coverage_anchor_acceptance_m", 0.30))))
        self.coverage_anchor_yaw_acceptance = math.radians(max(
            3.0, min(30.0, float(rospy.get_param(
                "~coverage_anchor_yaw_acceptance_deg", 10.0)))))
        self.coverage_progressive_attempts = max(1, int(rospy.get_param(
            "~coverage_progressive_attempts", 5)))
        self.coverage_progressive_step = max(0.35, min(0.85, float(
            rospy.get_param("~coverage_progressive_step_m", 0.70))))
        self.coverage_failed_exclusion = max(0.10, float(rospy.get_param(
            "~coverage_failed_exclusion_m", 0.20)))
        self.coverage_suspect_confirm_frames = max(1, int(rospy.get_param(
            "~coverage_suspect_confirm_frames", 2)))
        self.coverage_suspect_hold_s = max(0.6, float(rospy.get_param(
            "~coverage_suspect_hold_s", 1.8)))
        self.coverage_suspect_release_s = max(0.25, float(rospy.get_param(
            "~coverage_suspect_release_s", 0.55)))
        self.coverage_stationary_confirm_frames = max(2, int(
            rospy.get_param("~coverage_stationary_confirm_frames", 3)))
        self.coverage_endpoint_suspect_hold = bool(rospy.get_param(
            "~coverage_endpoint_suspect_hold_enabled", True))
        self._coverage_suspect_frames = 0
        self._coverage_suspect_last_seen = 0.0
        self._coverage_candidate_hold_active = False
        self._coverage_stationary_target_frames = 0
        self._coverage_physical_target_bbox = None
        self._coverage_physical_target_last_seen = 0.0
        self.coverage_physical_bbox_iou = max(0.25, min(0.80, float(
            rospy.get_param("~coverage_physical_bbox_iou", 0.42))))
        self.coverage_physical_bbox_center_ratio = max(0.02, min(0.15, float(
            rospy.get_param(
                "~coverage_physical_bbox_center_tolerance_ratio", 0.07))))
        self.coverage_physical_bbox_gap_s = max(0.45, float(rospy.get_param(
            "~coverage_physical_bbox_max_gap_s", 1.20)))
        self.coverage_search_radii = self._load_search_radii()
        self.coverage_search_samples = max(8, int(rospy.get_param(
            "~coverage_fallback_samples", 16)))
        # Reuse the proven fixed-point continuous-scan v2 navigation policy.
        # Keep every semantic anchor below unchanged: these offsets are used
        # only when a nominal pose cannot be reached or cannot rotate safely.
        self.coverage_v2_candidate_offsets = [
            (0.00, 0.00),
            (0.00, 0.12), (0.00, -0.12),
            (0.12, 0.00), (-0.12, 0.00),
            (0.12, 0.12), (-0.12, 0.12),
            (0.12, -0.12), (-0.12, -0.12),
            (0.00, 0.24), (0.00, -0.24),
            (0.24, 0.00), (-0.24, 0.00),
            (0.18, 0.18), (-0.18, 0.18),
            (0.18, -0.18), (-0.18, -0.18),
        ]
        self.coverage_v2_rotation_margin = max(0.01, float(
            rospy.get_param("~coverage_v2_rotation_margin_m", 0.025)))
        self.coverage_v2_candidate_exclusion = max(0.02, float(
            rospy.get_param("~coverage_v2_candidate_exclusion_m", 0.05)))
        self.coverage_v2_relocation_attempts = max(1, int(rospy.get_param(
            "~coverage_v2_relocation_attempts", 2)))
        self.coverage_boundary_margin = max(
            self.rotation_radius + 0.025,
            float(rospy.get_param("~coverage_boundary_margin_m", 0.24)))
        self.coverage_active_index = 0
        self.coverage_parking_escape_timeout = max(3.0, float(
            rospy.get_param(
                "~coverage_parking_escape_timeout_s", 4.5)))
        self.coverage_parking_return_timeout = max(3.0, float(
            rospy.get_param(
                "~coverage_parking_return_timeout_s", 5.0)))
        self.coverage_parking_stuck_s = max(1.5, float(rospy.get_param(
            "~coverage_parking_planner_stuck_s", 2.5)))
        self.coverage_parking_escape_anchors = max(1, int(rospy.get_param(
            "~coverage_parking_escape_anchor_attempts", 2)))
        self.coverage_far_anchor_parking_recovery = bool(rospy.get_param(
            "~coverage_far_anchor_parking_recovery_enabled", False))
        self.coverage_prepark_planner_recovery = bool(rospy.get_param(
            "~coverage_prepark_planner_recovery_enabled", True))
        self._coverage_alignment_recovery_active = False

        # Fuse the reference parking controller's wall-continuity and
        # rotation-priority rules with the current OCR-centering, cone guards
        # and predictive rectangular-footprint protection.
        self.fusion_wall_distance_jump = max(0.02, float(rospy.get_param(
            "~fusion_parking_wall_distance_jump_m", 0.06)))
        self.fusion_wall_heading_jump = math.radians(max(2.0, float(
            rospy.get_param("~fusion_parking_wall_heading_jump_deg", 8.0))))
        self.fusion_wall_memory_s = max(0.10, float(rospy.get_param(
            "~fusion_parking_wall_memory_s", 0.35)))
        self.fusion_yaw_priority = math.radians(max(2.0, float(
            rospy.get_param("~fusion_parking_yaw_priority_deg", 6.0))))
        self.parking_stable_frames = max(
            self.parking_stable_frames,
            int(rospy.get_param("~fusion_parking_stable_frames", 10)))
        self._fusion_last_wall = None
        self._fusion_last_wall_stamp = 0.0

        # The scan extents reproduce the reference point-checking process, but
        # odometry now closes one uninterrupted constant-speed sweep.  Only an
        # OCR target candidate or a physical rotation hazard pauses motion.
        self.coverage_anchors = [
            self._anchor(
                "south_left_inner", "南侧左内锚点",
                -0.67, -2.55, math.radians(-50),
                [(-math.radians(100), "clockwise")]),
            self._anchor(
                "southwest", "西南锚点",
                -1.55, -2.55, math.radians(-90),
                [(-math.radians(120), "clockwise")]),
            self._anchor(
                "west_mid", "西侧中部锚点",
                -1.53, -2.16, math.radians(180),
                [(-math.radians(90), "clockwise")]),
            self._anchor(
                "north_center", "北侧中部锚点",
                0.33, -1.94, math.radians(51),
                [(math.radians(100), "counterclockwise")]),
            self._anchor(
                "north_right", "北侧右部锚点",
                1.29, -1.94, math.radians(49),
                [(math.radians(100), "counterclockwise")]),
            self._anchor(
                "northeast", "东北锚点",
                2.34, -1.94, math.radians(0),
                [(math.radians(90), "counterclockwise")]),
            self._anchor(
                "southeast", "东南锚点",
                2.34, -2.34, math.radians(-125),
                [(2.25, "counterclockwise")]),
            self._anchor(
                "south_right_inner", "南侧右内锚点",
                1.30, -2.05, math.radians(-45),
                [(-math.radians(90), "clockwise")]),
        ]

        # The inherited parking code uses these arrays to associate a detected
        # target with its expected wall-facing yaw.  A sentinel keeps the base
        # mission's retry-index boundary compatible with eight real anchors.
        self.wall_route_points = [
            (anchor["name"], anchor["x"], anchor["y"])
            for anchor in self.coverage_anchors
        ]
        last = self.coverage_anchors[-1]
        self.wall_route_points.append(
            ("coverage_complete", last["x"], last["y"]))
        self.wall_route_yaws = [
            anchor["yaw"] for anchor in self.coverage_anchors
        ] + [last["yaw"]]

        self._coverage_ready = True
        self.publish_state(
            "ANCHOR_COVERAGE_READY",
            order=[anchor["name"] for anchor in self.coverage_anchors],
            continuous_scan_speed=self.coverage_scan_speed,
            anchor_stuck_s=self.coverage_fast_stuck_s)
        pending = self._pending_result
        self._pending_result = None
        if pending is not None:
            super(AnchorCoverageRoomDeliveryManager, self).result_callback(
                pending)

    @staticmethod
    def _anchor(name, display_name, x, y, yaw, sweeps):
        return {
            "name": name,
            "display_name": display_name,
            "x": float(x),
            "y": float(y),
            "yaw": float(yaw),
            "sweeps": list(sweeps),
        }

    def _load_search_radii(self):
        raw = rospy.get_param(
            "~coverage_fallback_radii_m", [0.12, 0.24, 0.36, 0.48, 0.60])
        if not isinstance(raw, (list, tuple)):
            raw = [0.12, 0.24, 0.36, 0.48, 0.60]
        values = sorted(set(
            float(value) for value in raw if 0.05 <= float(value) <= 0.90))
        return values or [0.12, 0.24, 0.36, 0.48, 0.60]

    def result_callback(self, msg):
        if not getattr(self, "_coverage_ready", False):
            self._pending_result = msg
            return
        super(AnchorCoverageRoomDeliveryManager, self).result_callback(msg)

    def ocr_callback(self, msg):
        """Pause on strong target evidence before the final framing decision."""
        if not getattr(self, "_coverage_ready", False):
            return super(AnchorCoverageRoomDeliveryManager, self).ocr_callback(
                msg)
        try:
            payload = json.loads(msg.data)
        except Exception:
            return super(AnchorCoverageRoomDeliveryManager, self).ocr_callback(
                msg)
        stable = bool(payload.get("stable", False))
        votes = int(payload.get("votes", 0) or 0)
        label = canonical_workshop(payload.get("label", ""))
        frame_label = canonical_workshop(payload.get("frame_label", ""))
        with self.lock:
            target = self.target_warehouse
            ignored_label = self.non_target_view_label
            search_active = self.room_search_active
            parking_active = self.parking_active
        strong_target_evidence = (
            self._coverage_scan_active and search_active and
            not parking_active and target and stable and
            label == target and frame_label == target and
            votes >= self.candidate_ocr_votes)
        if strong_target_evidence:
            self._coverage_suspect_frames += 1
            self._coverage_suspect_last_seen = time.monotonic()
            # The inherited manager intentionally locks a confirmed non-target
            # view.  During a moving coverage sweep, however, the next sign can
            # enter before the old lock's 40-degree release threshold.  Two
            # consecutive strong target frames prove this is worth stopping to
            # confirm, so release only the stale negative-view lock.
            if (ignored_label and self._coverage_suspect_frames >=
                    self.coverage_suspect_confirm_frames):
                with self.lock:
                    self.non_target_view_label = ""
                    self.non_target_view_anchor = None
                    self.non_target_odom_anchor = None
                    self.non_target_blank_since = None
                    self.non_target_target_frames = 0
                    self.target_confirm_count = 0
                self.publish_state(
                    "SUSPECT_TARGET_NEGATIVE_LOCK_RELEASED",
                    ignored_label=ignored_label,
                    target=target,
                    evidence_frames=self._coverage_suspect_frames)
        else:
            self._coverage_suspect_frames = 0

        physical_target_confirmed = self._update_physical_target_track(
            payload, strong_target_evidence, target, frame_label, label)

        # The legacy callback counts labels only.  Keep its useful candidate
        # pause behaviour, but withhold the final stable bit until five
        # spatially coherent observations have proved that all votes belong to
        # one physical workshop sign.  This prevents a rolling vote window
        # from combining different text boxes into TARGET_FOUND.
        guarded_msg = msg
        if (stable and label == target and frame_label == target and
                not physical_target_confirmed):
            guarded_payload = dict(payload)
            guarded_payload["stable"] = False
            guarded_msg = String(data=json.dumps(
                guarded_payload, ensure_ascii=False))
        super(AnchorCoverageRoomDeliveryManager, self).ocr_callback(
            guarded_msg)

        # The inherited detector deliberately waits until the complete sign is
        # already centred before it raises target_event.  That is safe during
        # free-running OCR, but it prevents the later active centring controller
        # from ever running when a genuine sign is initially near an image
        # edge.  While the chassis is deliberately stopped for confirmation,
        # accept several fresh, high-vote observations of the exact requested
        # workshop and hand them to that controller.  This keeps the old
        # "recognise, stop, then centre" strength without accepting a moving
        # single-frame hit or bypassing lidar parking validation.
        stationary_target = (
            self._coverage_candidate_hold_active and
            strong_target_evidence and
            votes >= self.required_ocr_votes and
            physical_target_confirmed)
        if stationary_target:
            if (self._coverage_stationary_target_frames >=
                    self.coverage_stationary_confirm_frames and
                    not self.target_event.is_set()):
                now = time.monotonic()
                with self.lock:
                    self.parking_target_ocr = dict(payload)
                    self.parking_target_stamp = now
                self.target_event.set()
                self.candidate_event.set()
                self.publish_state(
                    "OCR_STATIONARY_TARGET_CONFIRMED_FOR_ALIGNMENT",
                    label=target, votes=votes,
                    frames=self._coverage_stationary_target_frames,
                    bbox=payload.get("bbox"))
        elif self._coverage_candidate_hold_active and not strong_target_evidence:
            # Require consecutive positive frames once OCR is producing a
            # definite contradictory label.  A blank/blurred frame is handled
            # by the hold grace period and does not instantly discard evidence.
            if label and label != target:
                self._reset_physical_target_track()

        # Final target_event still belongs exclusively to the unchanged base
        # OCR framing/vote rules.  This earlier candidate event only stops the
        # chassis long enough for those rules to collect stationary frames.
        if (strong_target_evidence and
                self._coverage_suspect_frames >=
                self.coverage_suspect_confirm_frames and
                not self.target_event.is_set() and
                time.monotonic() >= self.candidate_cooldown_until):
            self.candidate_event.set()
            rospy.logwarn_throttle(
                0.4,
                "SUSPECT_TARGET_HOLD label=%s votes=%d frames=%d",
                target, votes, self._coverage_suspect_frames)

    @staticmethod
    def _credible_stationary_target_bbox(payload):
        """Reject tiny/empty OCR fragments while allowing a side-clipped sign."""
        bbox = payload.get("bbox")
        width = float(payload.get("image_width", 0) or 0)
        height = float(payload.get("image_height", 0) or 0)
        if (not isinstance(bbox, (list, tuple)) or len(bbox) < 4 or
                width <= 1.0 or height <= 1.0):
            return False
        left, top, right, bottom = [float(value) for value in bbox[:4]]
        visible_width = max(0.0, min(right, width) - max(left, 0.0))
        visible_height = max(0.0, min(bottom, height) - max(top, 0.0))
        return (
            visible_width >= max(48.0, 0.05 * width) and
            visible_height >= max(18.0, 0.03 * height))

    @staticmethod
    def _bbox_iou(first, second):
        left = max(first[0], second[0])
        top = max(first[1], second[1])
        right = min(first[2], second[2])
        bottom = min(first[3], second[3])
        intersection = max(0.0, right - left) * max(0.0, bottom - top)
        area_first = max(0.0, first[2] - first[0]) * max(
            0.0, first[3] - first[1])
        area_second = max(0.0, second[2] - second[0]) * max(
            0.0, second[3] - second[1])
        union = area_first + area_second - intersection
        return 0.0 if union <= 1.0 else intersection / union

    def _physical_target_boxes(self, payload):
        """Return only the detector-qualified crop that produced the text."""
        bbox = payload.get("bbox")
        width = float(payload.get("image_width", 0) or 0)
        height = float(payload.get("image_height", 0) or 0)
        if (not isinstance(bbox, (list, tuple)) or len(bbox) < 4 or
                width <= 1.0 or height <= 1.0):
            return None
        try:
            word = tuple(float(value) for value in bbox[:4])
        except (TypeError, ValueError):
            return None
        word_width = max(0.0, word[2] - word[0])
        word_height = max(0.0, word[3] - word[1])
        # Edge fragments may still pause the sweep, but cannot start parking.
        if (word[0] < 0.01 * width or word[2] > 0.99 * width or
                word_width < 0.06 * width or word_width > 0.62 * width or
                word_height < 0.03 * height or word_height > 0.32 * height):
            return None
        return word, width, height

    def _reset_physical_target_track(self):
        self._coverage_stationary_target_frames = 0
        self._coverage_physical_target_bbox = None
        self._coverage_physical_target_last_seen = 0.0

    def _update_physical_target_track(self, payload, strong_target_evidence,
                                      target, frame_label, stable_label):
        """Require consecutive observations of the same physical sign."""
        if not self._coverage_candidate_hold_active:
            self._reset_physical_target_track()
            return False
        definite_frame_label = (
            frame_label if frame_label not in
            ("", "unknown", "none", "未知") else "")
        if definite_frame_label and definite_frame_label != target:
            if self._coverage_stationary_target_frames:
                rospy.logwarn("OCR确认重置 %s -> %s", target,
                              definite_frame_label)
            self._reset_physical_target_track()
            return False
        boxes = self._physical_target_boxes(payload)
        if not strong_target_evidence or stable_label != target:
            return False
        if boxes is None:
            # A target word that cannot be tied to the same complete paper
            # sign breaks the sequence; do not skip it and keep old votes.
            self._reset_physical_target_track()
            return False
        if (self._coverage_physical_target_last_seen > 0.0 and
                time.monotonic() -
                self._coverage_physical_target_last_seen >
                self.coverage_physical_bbox_gap_s):
            self._reset_physical_target_track()
        word, width, _height = boxes
        # Track the exact crop that produced the requested workshop label.
        sign = word
        previous = self._coverage_physical_target_bbox
        coherent = previous is not None
        iou = 0.0
        center_shift = 0.0
        if previous is not None:
            iou = self._bbox_iou(previous, sign)
            center_shift = abs(
                0.5 * (sign[0] + sign[2]) -
                0.5 * (previous[0] + previous[2])) / width
            coherent = (
                iou >= self.coverage_physical_bbox_iou and
                center_shift <= self.coverage_physical_bbox_center_ratio)
        if not coherent:
            self._coverage_stationary_target_frames = 1
        else:
            self._coverage_stationary_target_frames += 1
        # Slowly follow detector jitter without allowing a remote box to take
        # over the accumulated identity.
        if previous is None or not coherent:
            tracked = sign
        else:
            tracked = tuple(
                0.65 * previous[index] + 0.35 * sign[index]
                for index in range(4))
        self._coverage_physical_target_bbox = tracked
        self._coverage_physical_target_last_seen = time.monotonic()
        rospy.logwarn_throttle(
            0.35,
            "OCR确认 %s %d/%d",
            target, self._coverage_stationary_target_frames,
            self.coverage_stationary_confirm_frames)
        return (self._coverage_stationary_target_frames >=
                self.coverage_stationary_confirm_frames)

    def hold_ocr_candidate(self, goal_name):
        """Keep still through brief OCR gaps before abandoning a suspect."""
        self._coverage_candidate_hold_active = True
        self._reset_physical_target_track()
        try:
            self.stop_robot(8)
            self.publish_state(
                "OCR_SUSPECT_TARGET_CONFIRMING", goal=goal_name,
                hold_s=self.coverage_suspect_hold_s,
                stationary_confirm_frames=
                self.coverage_stationary_confirm_frames)
            started = time.monotonic()
            deadline = started + self.coverage_suspect_hold_s
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                if self.target_event.is_set():
                    self.publish_state(
                        "OCR_SUSPECT_TARGET_CONFIRMED", goal=goal_name)
                    return True
                # Do not leave on one blank/blurred frame.  Release only after
                # target evidence has stayed absent for a continuous window.
                absent_s = time.monotonic() - self._coverage_suspect_last_seen
                if (time.monotonic() - started >= 0.60 and
                        absent_s >= self.coverage_suspect_release_s):
                    break
                self.cmd_pub.publish(Twist())
                rospy.sleep(0.04)
            self.candidate_event.clear()
            with self.lock:
                self.candidate_cooldown_until = (
                    time.monotonic() + self.candidate_cooldown_s)
            self.publish_state(
                "OCR_SUSPECT_TARGET_REJECTED_RESUME", goal=goal_name,
                stationary_frames=self._coverage_stationary_target_frames)
            return self.target_event.is_set()
        finally:
            self._coverage_candidate_hold_active = False
            self._reset_physical_target_track()

    def _inside_room(self, x, y):
        margin = self.coverage_boundary_margin
        return (self.room_min_x + margin <= x <= self.room_max_x - margin and
                self.room_min_y + margin <= y <= self.room_max_y - margin)

    def _candidate_positions(self, anchor):
        """Yield the old three-point-v2 direct-first fallback pattern.

        The nominal anchor remains the first choice.  Unlike the later radial
        search, this bounded deterministic pattern does not send move_base
        through a succession of intermediate ring points.  Prefer changing x
        before changing y so a random cone cannot silently create a large
        apparent y-coordinate drift in RViz.
        """
        offsets = sorted(
            self.coverage_v2_candidate_offsets,
            key=lambda offset: (
                round(abs(offset[1]), 6),
                math.hypot(offset[0], offset[1])))
        for offset_x, offset_y in offsets:
            x = anchor["x"] + offset_x
            y = anchor["y"] + offset_y
            if self._spin_pose_static_safe(x, y):
                yield math.hypot(offset_x, offset_y), x, y

    def _spin_pose_static_safe(self, x, y):
        """Check that the complete rectangular chassis can rotate in-room."""
        radius = self.rotation_radius + self.coverage_v2_rotation_margin
        return (
            self.room_min_x + radius <= x <= self.room_max_x - radius and
            self.room_min_y + radius <= y <= self.room_max_y - radius)

    def _spin_pose_live_safe(self):
        """Apply the v2 arrival check using the latest real lidar clearance."""
        with self.lock:
            clearance = self.rotation_clearance
            scan_stamp = self.scan_stamp
        return (
            time.monotonic() - scan_stamp <= self.sweep_sensor_fresh_s and
            clearance >= max(
                self.sweep_rotation_clearance,
                self.coverage_v2_rotation_margin))

    def _nearest_scan_pose(self, anchor, excluded=None):
        excluded = excluded or []
        for distance, x, y in self._candidate_positions(anchor):
            if any(math.hypot(x - old_x, y - old_y) <
                   self.coverage_v2_candidate_exclusion
                   for old_x, old_y in excluded):
                continue
            if not self.make_plan_exists(x, y, anchor["yaw"]):
                continue
            self.publish_state(
                "SCAN_ANCHOR_SELECTED",
                anchor=anchor["name"],
                anchor_label=anchor["display_name"],
                requested_x=anchor["x"], requested_y=anchor["y"],
                selected_x=x, selected_y=y,
                fallback_distance=distance,
                navigation_policy="fixed_point_continuous_v2")
            return x, y, anchor["yaw"]
        return None

    def _remember_target_view_yaw(self):
        pose = self.current_pose()
        if pose is not None:
            index = max(0, min(
                self.coverage_active_index, len(self.wall_route_yaws) - 1))
            self.wall_route_yaws[index] = pose[2]
            self.publish_state(
                "ANCHOR_TARGET_VIEW_LOCKED", anchor_index=index,
                view_yaw=pose[2])

    def _observation_dwell(self, anchor_name, phase):
        deadline = time.monotonic() + self.coverage_dwell_s
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            interrupted = self.motion_interrupted_by_ocr(
                "{}_{}".format(anchor_name, phase))
            if interrupted is not None:
                if interrupted == "TARGET":
                    self._remember_target_view_yaw()
                return interrupted
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.04)
        return "SUCCEEDED"

    def _scan_sweep(self, anchor, sweep_index, signed_angle, direction_name):
        direction = 1.0 if signed_angle >= 0.0 else -1.0
        total_angle = abs(signed_angle)
        self.publish_state(
            "ANCHOR_CONTINUOUS_SWEEP_START", anchor=anchor["name"],
            anchor_label=anchor["display_name"], sweep=sweep_index,
            direction=direction_name,
            total_angle_deg=math.degrees(signed_angle),
            angular_speed=self.coverage_scan_speed)
        with self.lock:
            last_yaw = self.odom_yaw
            odom_stamp = self.odom_stamp
        if last_yaw is None or time.monotonic() - odom_stamp > 0.8:
            return "NO_ODOM"
        progress = 0.0
        started = time.monotonic()
        deadline = (started + total_angle / self.coverage_scan_speed *
                    self.coverage_scan_timeout_scale + 2.0)
        blocked_since = None
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            interruption_started = time.monotonic()
            interrupted = self.motion_interrupted_by_ocr(
                "{}_sweep_{}".format(anchor["name"], sweep_index))
            interruption_elapsed = time.monotonic() - interruption_started
            if interruption_elapsed > 0.10:
                # A candidate confirmation hold is intentional and must not
                # consume the continuous sweep deadline.
                deadline += interruption_elapsed
            if interrupted is not None:
                if interrupted == "TARGET":
                    self._remember_target_view_yaw()
                return interrupted
            with self.lock:
                odom_yaw = self.odom_yaw
                odom_stamp = self.odom_stamp
                clearance = self.rotation_clearance
                scan_stamp = self.scan_stamp
            now = time.monotonic()
            if (odom_yaw is None or now - odom_stamp > 0.8 or
                    now - scan_stamp > self.sweep_sensor_fresh_s):
                self.cmd_pub.publish(Twist())
                rate.sleep()
                continue
            delta = norm_angle(odom_yaw - last_yaw)
            last_yaw = odom_yaw
            directed_delta = direction * delta
            if directed_delta > 0.0:
                progress += directed_delta
            remaining = max(0.0, total_angle - progress)
            if remaining <= self.coverage_scan_tolerance:
                self.smooth_stop_robot()
                # An exact target can first become stable in the final camera
                # frame of a sweep.  Previously the sweep returned immediately,
                # disabled the scan-only suspect layer and started navigation
                # before the required second frame arrived.  Hold only when a
                # strong exact-target frame was seen very recently; blanks and
                # non-target signs still leave without any extra pause.
                suspect_age = now - self._coverage_suspect_last_seen
                endpoint_suspect = (
                    self.coverage_endpoint_suspect_hold and
                    not self.target_event.is_set() and
                    self._coverage_suspect_frames > 0 and
                    suspect_age <= self.coverage_suspect_release_s)
                if endpoint_suspect:
                    self.candidate_event.set()
                    self.publish_state(
                        "OCR_SWEEP_ENDPOINT_SUSPECT_HOLD",
                        anchor=anchor["name"], sweep=sweep_index,
                        evidence_frames=self._coverage_suspect_frames,
                        evidence_age_s=suspect_age,
                        hold_s=self.coverage_suspect_hold_s)
                    if self.hold_ocr_candidate(
                            "{}_sweep_{}_endpoint".format(
                                anchor["name"], sweep_index)):
                        self._remember_target_view_yaw()
                        return "TARGET"
                self.publish_state(
                    "ANCHOR_CONTINUOUS_SWEEP_COMPLETE",
                    anchor=anchor["name"], sweep=sweep_index,
                    swept_angle_deg=math.degrees(progress))
                return "SUCCEEDED"
            if clearance < self.sweep_rotation_clearance:
                self.cmd_pub.publish(Twist())
                if blocked_since is None:
                    blocked_since = now
                if now - blocked_since >= self.sweep_block_timeout:
                    self.stop_robot(8)
                    self.publish_state(
                        "ANCHOR_CONTINUOUS_SWEEP_BLOCKED",
                        anchor=anchor["name"], sweep=sweep_index,
                        clearance=clearance,
                        swept_angle_deg=math.degrees(progress))
                    return "BLOCKED"
                rate.sleep()
                continue
            blocked_since = None
            self.publish_direct_command(
                wz=direction * self.coverage_scan_speed)
            rospy.logwarn_throttle(
                0.6,
                "ANCHOR_CONTINUOUS_SWEEP anchor=%s progress=%.1f/%.1fdeg "
                "wz=%.3f clear=%.3f",
                anchor["name"], math.degrees(progress),
                math.degrees(total_angle),
                direction * self.coverage_scan_speed, clearance)
            rate.sleep()
        self.stop_robot(8)
        self.publish_state(
            "ANCHOR_CONTINUOUS_SWEEP_TIMEOUT", anchor=anchor["name"],
            sweep=sweep_index, swept_angle_deg=math.degrees(progress))
        return "TIMEOUT"

    def _inspect_anchor(self, anchor):
        # Far-away signs can remain readable while move_base is crossing the
        # room.  Suspect-only OCR must not repeatedly cancel that smooth
        # navigation; enable the extra hold layer only after the anchor has
        # actually been reached.  The inherited final-target OCR remains live
        # throughout navigation.
        self.candidate_event.clear()
        self._coverage_suspect_frames = 0
        self._coverage_scan_active = True
        try:
            result = self._observation_dwell(
                anchor["name"], "initial_dwell")
            if result != "SUCCEEDED":
                return result
            for sweep_index, (signed_angle, direction_name) in enumerate(
                    anchor["sweeps"], 1):
                result = self._scan_sweep(
                    anchor, sweep_index, signed_angle, direction_name)
                if result != "SUCCEEDED":
                    return result
            return "SUCCEEDED"
        finally:
            self._coverage_scan_active = False
            self._coverage_suspect_frames = 0
            if not self.target_event.is_set():
                self.candidate_event.clear()

    def _nearest_progress_pose(self, anchor, excluded):
        """Choose a short reachable step that strictly approaches an anchor."""
        pose = self.current_pose()
        if pose is None:
            return None
        dx = anchor["x"] - pose[0]
        dy = anchor["y"] - pose[1]
        remaining = math.hypot(dx, dy)
        if remaining <= self.coverage_anchor_acceptance:
            return (pose[0], pose[1], pose[2])
        heading = math.atan2(dy, dx)
        travel = min(
            self.coverage_progressive_step,
            max(0.35, remaining - self.coverage_anchor_acceptance))
        tangent_x = -math.sin(heading)
        tangent_y = math.cos(heading)
        base_x = pose[0] + travel * math.cos(heading)
        base_y = pose[1] + travel * math.sin(heading)
        candidates = []
        for lateral in (0.0, 0.16, -0.16, 0.30, -0.30):
            x = base_x + lateral * tangent_x
            y = base_y + lateral * tangent_y
            distance_to_anchor = math.hypot(
                anchor["x"] - x, anchor["y"] - y)
            candidates.append((distance_to_anchor, abs(lateral), x, y))
        candidates.sort(key=lambda value: (value[0], value[1]))
        for distance_to_anchor, _, x, y in candidates:
            if not self._inside_room(x, y):
                continue
            if distance_to_anchor >= remaining - 0.10:
                continue
            if any(math.hypot(x - old_x, y - old_y) <
                   self.coverage_failed_exclusion
                   for old_x, old_y in excluded):
                continue
            if self.make_plan_exists(x, y, heading):
                return x, y, heading
        return None

    def _progressively_approach_anchor(self, anchor, excluded):
        """Advance in short reachable hops instead of dropping an anchor."""
        initial_pose = self.current_pose()
        initial_remaining = (None if initial_pose is None else math.hypot(
            anchor["x"] - initial_pose[0],
            anchor["y"] - initial_pose[1]))
        for attempt in range(1, self.coverage_progressive_attempts + 1):
            pose = self.current_pose()
            if pose is None:
                return "NO_POSE"
            remaining = math.hypot(
                anchor["x"] - pose[0], anchor["y"] - pose[1])
            if remaining <= self.coverage_anchor_acceptance:
                self.publish_state(
                    "SCAN_ANCHOR_PROGRESSIVE_REACHED",
                    anchor=anchor["name"], distance=remaining,
                    attempts=attempt - 1)
                return "SUCCEEDED"
            selected = self._nearest_progress_pose(anchor, excluded)
            if selected is None:
                break
            self.publish_state(
                "SCAN_ANCHOR_PROGRESSIVE_NAVIGATION",
                anchor=anchor["name"], attempt=attempt,
                x=selected[0], y=selected[1], yaw=selected[2],
                remaining=remaining)
            result = self._send_anchor_goal_fast(
                selected[0], selected[1], selected[2],
                "{}_progressive_{}".format(anchor["name"], attempt),
                min(10.0, self.coverage_goal_timeout))
            if result == "TARGET":
                return result
            if result != "SUCCEEDED":
                excluded.append((selected[0], selected[1]))
                self.clear_and_wait(
                    "{} progressive {} {}".format(
                        anchor["name"], attempt, result))
        pose = self.current_pose()
        if pose is None:
            return "NO_POSE"
        remaining = math.hypot(
            anchor["x"] - pose[0], anchor["y"] - pose[1])
        self.publish_state(
            "SCAN_ANCHOR_COMPENSATION_AT_CLOSEST_POSE",
            anchor=anchor["name"], x=pose[0], y=pose[1],
            distance=remaining, initial_distance=initial_remaining)
        # Scanning the closest physically reached pose is preferable to
        # silently losing an entire observation sector.
        return "SUCCEEDED_COMPENSATION"

    def _navigate_to_anchor(self, anchor, excluded=None):
        """Use the original three-point-v2 direct-first navigation policy."""
        excluded = list(excluded or [])
        last_result = "UNREACHABLE"
        maximum_attempts = max(
            self.coverage_goal_attempts,
            min(len(self.coverage_v2_candidate_offsets), 6))
        for attempt in range(1, maximum_attempts + 1):
            selected = self._nearest_scan_pose(anchor, excluded)
            if selected is None:
                return "UNREACHABLE"
            self.publish_state(
                "SCAN_ANCHOR_NAVIGATION", anchor=anchor["name"],
                anchor_label=anchor["display_name"], attempt=attempt,
                x=selected[0], y=selected[1], yaw=selected[2])
            last_result = self._send_anchor_goal_fast(
                selected[0], selected[1], selected[2],
                "{}_inspection_anchor".format(anchor["name"]),
                self.coverage_goal_timeout)
            if last_result == "TARGET":
                self._remember_target_view_yaw()
                return last_result
            if last_result == "SUCCEEDED":
                if self._spin_pose_live_safe():
                    return last_result
                pose = self.current_pose()
                if pose is not None:
                    excluded.append((pose[0], pose[1]))
                excluded.append((selected[0], selected[1]))
                self.publish_state(
                    "SCAN_ANCHOR_ROTATION_POSE_REJECTED",
                    anchor=anchor["name"], attempt=attempt,
                    reason="insufficient_live_rotation_clearance")
                continue
            excluded.append((selected[0], selected[1]))
            self.clear_and_wait(
                "{} anchor attempt {} {}".format(
                    anchor["name"], attempt, last_result))
        # Do not fall back to the later progressive-hop/closest-pose scan.
        # The old v2 policy either reaches a verified rotatable pose or lets
        # the ordered route continue to the next real scan anchor.
        return last_result

    def _send_anchor_goal_fast(self, x, y, yaw, name, timeout):
        """Navigate with a translation-only three-second progress watchdog."""
        goal = MoveBaseGoal()
        goal.target_pose = self.pose_message(x, y, yaw)
        self.move_base.send_goal(goal)
        self.publish_state(
            "NAVIGATING", goal=name, x=x, y=y, yaw=yaw,
            fast_stuck_s=self.coverage_fast_stuck_s)
        started = time.monotonic()
        progress_stamp = started
        progress_pose = self.current_pose()
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            if self.target_event.is_set():
                self.move_base.cancel_goal()
                self.stop_robot(6)
                return "TARGET"
            if self.candidate_event.is_set():
                self.move_base.cancel_goal()
                self.stop_robot(6)
                hold_started = time.monotonic()
                if self.hold_ocr_candidate(name):
                    return "TARGET"
                hold_elapsed = time.monotonic() - hold_started
                started += hold_elapsed
                self.move_base.send_goal(goal)
                progress_stamp = time.monotonic()
                progress_pose = self.current_pose()
                rate.sleep()
                continue
            now = time.monotonic()
            pose = self.current_pose()
            distance = None
            if pose is not None:
                distance = math.hypot(x - pose[0], y - pose[1])
                yaw_error = abs(norm_angle(yaw - pose[2]))
                if (distance <= self.coverage_anchor_acceptance and
                        yaw_error <= self.coverage_anchor_yaw_acceptance):
                    self.move_base.cancel_goal()
                    self.stop_robot(5)
                    self.publish_state(
                        "SCAN_ANCHOR_POSITION_ACCEPTED", goal=name,
                        distance=distance,
                        acceptance=self.coverage_anchor_acceptance,
                        yaw_error_deg=math.degrees(yaw_error),
                        yaw_acceptance_deg=math.degrees(
                            self.coverage_anchor_yaw_acceptance))
                    return "SUCCEEDED"
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                return "SUCCEEDED"
            if state not in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                return "FAILED_{}".format(state)
            if pose is not None:
                if progress_pose is None or math.hypot(
                        pose[0] - progress_pose[0],
                        pose[1] - progress_pose[1]) >= self.coverage_fast_progress_m:
                    progress_stamp = now
                    progress_pose = pose
                elif now - progress_stamp >= self.coverage_fast_stuck_s:
                    self.move_base.cancel_goal()
                    self.stop_robot(6)
                    self.publish_state(
                        "SCAN_ANCHOR_FAST_REPLAN", goal=name,
                        no_translation_s=now - progress_stamp,
                        remaining=distance)
                    return "STUCK_FAST"
            if now - started >= timeout:
                self.move_base.cancel_goal()
                self.stop_robot(6)
                return "TIMEOUT"
            rate.sleep()
        return "SHUTDOWN"

    def parking_front_wall_estimate(self):
        """Reject abrupt wall swaps and bridge only very short lidar gaps."""
        estimate = super(
            AnchorCoverageRoomDeliveryManager,
            self).parking_front_wall_estimate()
        now = time.monotonic()
        previous = self._fusion_last_wall
        if estimate is not None:
            continuous = (
                previous is None or
                (abs(estimate[0] - previous[0]) <=
                 self.fusion_wall_distance_jump and
                 abs(norm_angle(estimate[1] - previous[1])) <=
                 self.fusion_wall_heading_jump))
            if continuous:
                self._fusion_last_wall = estimate
                self._fusion_last_wall_stamp = now
                return estimate
            self.publish_state(
                "FUSION_PARKING_WALL_JUMP_REJECTED",
                previous_distance=previous[0], distance=estimate[0],
                previous_heading=previous[1], heading=estimate[1])
        if (previous is not None and
                now - self._fusion_last_wall_stamp <= self.fusion_wall_memory_s):
            return previous
        return None

    def parking_commands(self, allow_forward):
        values = super(
            AnchorCoverageRoomDeliveryManager,
            self).parking_commands(allow_forward)
        if values is None:
            return None
        (vx, vy, wz, distance, heading, points, pixel_error,
         target_live, target_age) = values
        if abs(heading) > self.fusion_yaw_priority:
            # Reference controller strength: never translate while the chassis
            # is still materially oblique to the measured wall.
            vx = 0.0
            vy = 0.0
            # This is an intentional heading-only phase, not a blocked lateral
            # command.  Do not let the stall watchdog integrate a body-y request
            # which is deliberately being withheld here.
            self.parking_lateral_requested = 0.0
            self.parking_lateral_guarded = 0.0
        return (vx, vy, wz, distance, heading, points, pixel_error,
                target_live, target_age)

    def locked_parking_target_center(self):
        """Keep centering from fresh target frames after final confirmation.

        The OCR node alternates stable vote summaries with current-frame
        candidates.  Once the physical target has passed final confirmation,
        a fresh candidate of that same workshop is valid geometric evidence;
        requiring ``stable=True`` on every frame freezes an old pixel center.
        """
        fallback = super(
            AnchorCoverageRoomDeliveryManager,
            self).locked_parking_target_center()
        if (fallback is None or fallback[3] or
                not self.target_event.is_set()):
            return fallback

        now = time.monotonic()
        with self.lock:
            live = None if self.latest_ocr is None else dict(self.latest_ocr)
            target = self.target_warehouse
        candidate_live = (
            live is not None and
            live.get("frame_label") == target and
            int(live.get("votes", 0) or 0) >= self.candidate_ocr_votes and
            self.parking_center_observation_valid(live) and
            now - float(live.get("received_monotonic", 0.0)) <=
            self.ocr_fresh_s)
        if not candidate_live:
            return fallback

        center = self.ocr_center(live)
        if center is None:
            return fallback
        source_stamp = float(live.get("received_monotonic", 0.0))
        with self.lock:
            if source_stamp > self.parking_center_source_stamp + 1.0e-5:
                # ``filtered_center_x`` is already filtered by the OCR node.
                # Re-filtering it here made the control centre trail the real
                # sign by hundreds of pixels after the wall-facing turn.
                self.parking_center_filtered = center[0]
                self.parking_center_width = center[1]
                self.parking_center_source_stamp = source_stamp
            filtered = self.parking_center_filtered
            filtered_width = self.parking_center_width
        rospy.logwarn_throttle(
            0.5, "PARKING_CONFIRMED_TARGET_LIVE_CENTER center=%.1f/%.1f "
            "votes=%d stable=%s", filtered, filtered_width,
            int(live.get("votes", 0) or 0),
            str(bool(live.get("stable", False))))
        return filtered, filtered_width, 0.0, True

    def _send_parking_recovery_goal_fast(self, x, y, yaw, name, timeout):
        """Run a planner goal with a short translation-only stall watchdog."""
        goal = MoveBaseGoal()
        goal.target_pose = self.pose_message(x, y, yaw)
        self.move_base.send_goal(goal)
        self.publish_state(
            "PARKING_RECOVERY_PLANNER_NAVIGATING", goal=name,
            x=x, y=y, yaw=yaw, timeout=timeout,
            stuck_s=self.coverage_parking_stuck_s)
        started = time.monotonic()
        progress_stamp = started
        progress_pose = self.current_pose()
        rate = rospy.Rate(15)
        while not rospy.is_shutdown():
            now = time.monotonic()
            pose = self.current_pose()
            if pose is not None:
                distance = math.hypot(x - pose[0], y - pose[1])
                if distance <= 0.18:
                    self.move_base.cancel_goal()
                    self.smooth_stop_robot()
                    self.publish_state(
                        "PARKING_RECOVERY_PLANNER_REACHED", goal=name,
                        distance=distance)
                    return "SUCCEEDED"
                if (progress_pose is None or math.hypot(
                        pose[0] - progress_pose[0],
                        pose[1] - progress_pose[1]) >=
                        self.coverage_fast_progress_m):
                    progress_stamp = now
                    progress_pose = pose
                elif now - progress_stamp >= self.coverage_parking_stuck_s:
                    self.move_base.cancel_goal()
                    self.smooth_stop_robot()
                    self.publish_state(
                        "PARKING_RECOVERY_PLANNER_STUCK", goal=name,
                        no_translation_s=now - progress_stamp,
                        remaining=distance)
                    return "STUCK_FAST"
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                self.smooth_stop_robot()
                return "SUCCEEDED"
            if state not in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                self.smooth_stop_robot()
                return "FAILED_{}".format(state)
            if now - started >= timeout:
                self.move_base.cancel_goal()
                self.smooth_stop_robot()
                return "TIMEOUT"
            rate.sleep()
        return "SHUTDOWN"

    def planner_reposition_after_parking_stall(self, attempt):
        """Escape with TEB through a known-safe anchor, then reacquire target.

        Tiny local side-shift goals look reachable to the global planner while
        TEB can still reject every initial trajectory when the chassis is close
        to a cone or wall.  Reusing a previously reached inspection anchor gives
        TEB a meaningful free-space route, avoids direct body-y escape motion,
        and prevents four consecutive eight-second waits.
        """
        if not self.coverage_far_anchor_parking_recovery:
            self.publish_state(
                "PARKING_RECOVERY_LOCAL_TARGET_WALL", attempt=attempt)
            return Xunfei2026RoomDeliveryManager.planner_reposition_after_parking_stall(
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
                anchor_index=neighbor_index, target_anchor=target_anchor["name"])
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
                    "PARKING_RECOVERY_TARGET_RETURN_UNREACHABLE",
                    attempt=attempt, target_anchor=target_anchor["name"])
                continue
            result = self._send_parking_recovery_goal_fast(
                selected_target[0], selected_target[1], selected_target[2],
                "parking_target_return_{}".format(attempt),
                self.coverage_parking_return_timeout)
            if result != "SUCCEEDED":
                self.publish_state(
                    "PARKING_RECOVERY_TARGET_RETURN_FAILED",
                    attempt=attempt, result=result,
                    target_anchor=target_anchor["name"])
                continue

            # The exact target remains confirmed; rebuild its live image and
            # lidar centreline only after the planner has restored clearance.
            if not self.align_target_for_parking():
                self.publish_state(
                    "PARKING_RECOVERY_ALIGNMENT_FAILED", attempt=attempt)
                continue
            if not self.refresh_centered_target_wall():
                self.publish_state(
                    "PARKING_RECOVERY_WALL_LOCK_FAILED", attempt=attempt)
                continue
            if not self.approach_target():
                self.publish_state(
                    "PARKING_RECOVERY_PREPARK_FAILED", attempt=attempt)
                continue
            self.publish_state(
                "PARKING_RECOVERY_REACQUIRED", attempt=attempt,
                escape_anchor=escape_anchor["name"],
                target_anchor=target_anchor["name"])
            return True

        self.publish_state(
            "PARKING_RECOVERY_NO_SAFE_ANCHOR", attempt=attempt,
            attempted=attempted)
        return False

    def align_target_for_parking(self):
        """Keep an exact target and use TEB when rotation is physically blocked."""
        aligned = super(
            AnchorCoverageRoomDeliveryManager,
            self).align_target_for_parking()
        if (aligned or not self.coverage_prepark_planner_recovery or
                self._coverage_alignment_recovery_active or
                not self.target_event.is_set()):
            return aligned
        self.publish_state(
            "PREPARK_ALIGNMENT_BLOCKED_PLANNER_RECOVERY",
            anchor_index=self.coverage_active_index)
        self._coverage_alignment_recovery_active = True
        try:
            # Attempt zero denotes recovery before the first parking run.  The
            # re-entry call made inside the planner recovery sees the guard and
            # executes only the inherited visual alignment, avoiding recursion.
            return self.planner_reposition_after_parking_stall(0)
        finally:
            self._coverage_alignment_recovery_active = False

    def park(self):
        self._fusion_last_wall = None
        self._fusion_last_wall_stamp = 0.0
        self.publish_state(
            "FUSION_PARKING_ACTIVE",
            yaw_priority_deg=math.degrees(self.fusion_yaw_priority),
            stable_frames=self.parking_stable_frames,
            wall_memory_s=self.fusion_wall_memory_s)
        result = super(AnchorCoverageRoomDeliveryManager, self).park()
        if (result == "REACQUIRE" and self.target_event.is_set() and
                not self.parking_wrong_event.is_set()):
            # OCR still confirms the requested workshop.  Losing a transient
            # wall fit is a positioning problem, not evidence that the target
            # was wrong; route through the planner recovery instead of clearing
            # the target and continuing to later scan anchors.
            self.publish_state(
                "PARKING_WALL_REACQUIRE_PLANNER_RECOVERY",
                anchor_index=self.coverage_active_index)
            return "REPLAN"
        return result

    def run_wall_route(self, start_index=0, approach_start=True):
        del approach_start  # Every anchor is independently planned and checked.
        count = len(self.coverage_anchors)
        start_index = max(0, min(int(start_index), count))
        if start_index >= count:
            return "SUCCEEDED"

        for index in range(start_index, count):
            self.coverage_active_index = index
            self.active_wall_segment_index = index
            anchor = self.coverage_anchors[index]
            self.publish_state(
                "SCAN_ANCHOR_BEGIN", anchor_index=index,
                anchor=anchor["name"],
                anchor_label=anchor["display_name"])
            result = self._navigate_to_anchor(anchor)
            if result == "TARGET":
                return result
            if result != "SUCCEEDED":
                # One blocked nominal area must not terminate the mission.  It
                # has already exhausted nearest-first alternatives; continue
                # the ordered coverage from the next semantic anchor.
                self.publish_state(
                    "SCAN_ANCHOR_SKIPPED", anchor=anchor["name"],
                    result=result)
                continue
            result = self._inspect_anchor(anchor)
            if result == "TARGET":
                return result
            if result == "OCR_UNAVAILABLE":
                return result
            if result == "BLOCKED":
                # Match fixed-point-v2: a live obstacle invalidates this scan
                # pose, not the semantic anchor.  Move to the nearest verified
                # alternative and restart this anchor's requested sweep.
                for relocation in range(
                        1, self.coverage_v2_relocation_attempts + 1):
                    pose = self.current_pose()
                    excluded = [] if pose is None else [(pose[0], pose[1])]
                    self.publish_state(
                        "SCAN_ANCHOR_ALTERNATE_SEARCH_V2",
                        anchor=anchor["name"], attempt=relocation,
                        reason="rotation_blocked")
                    navigation = self._navigate_to_anchor(
                        anchor, excluded=excluded)
                    if navigation == "TARGET":
                        return navigation
                    if navigation != "SUCCEEDED":
                        continue
                    result = self._inspect_anchor(anchor)
                    if result in ("TARGET", "OCR_UNAVAILABLE"):
                        return result
                    if result == "SUCCEEDED":
                        break
            if result != "SUCCEEDED":
                self.publish_state(
                    "SCAN_ANCHOR_PARTIAL", anchor=anchor["name"],
                    result=result)
                continue
            self.publish_state(
                "SCAN_ANCHOR_COMPLETE", anchor=anchor["name"],
                anchor_label=anchor["display_name"])
        return "SUCCEEDED"

    def wall_route_resume_index(self):
        # A rejected/expired target should continue coverage instead of driving
        # back to the same nominal point and repeating the same camera view.
        return min(
            self.coverage_active_index + 1,
            len(self.coverage_anchors) - 1)


if __name__ == "__main__":
    AnchorCoverageRoomDeliveryManager()
    rospy.spin()
