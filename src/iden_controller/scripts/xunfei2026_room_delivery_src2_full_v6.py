#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Full src2 cone-room motion with the calibrated competition anchors.

The mission adapter (speech/order, two deliveries and Gazebo handoff) remains
outside the room-motion core.  Inside the cone room this class uses src2's
navigation watchdog, stop-and-look scan, target centering, wall geometry,
staging, three-phase docking and footprint validation.  The calibrated anchor
coordinates, start headings, scan directions and scan angles are inherited
unchanged from the current competition route.
"""

import json
import math
import threading
import time

import rospy
from geometry_msgs.msg import Twist

from xunfei2026_room_delivery_src2_dual_stage_v5 import (
    Src2DualStageRoomDeliveryV5,
)
from xunfei2026_src2_navigation_core_v1 import normalize_angle


SRC2_CATEGORY_WORKSHOP = {
    "food": "食品加工车间",
    "daily": "日用品加工车间",
    "electronic": "电子产品生产车间",
}
SRC2_WORKSHOP_CATEGORY = dict(
    (workshop, category)
    for category, workshop in SRC2_CATEGORY_WORKSHOP.items())
SRC2_WORKSHOP_CATEGORY["电子产品加工车间"] = "electronic"


class Src2FullRoomDeliveryV6(Src2DualStageRoomDeliveryV5):
    """Make src2 the sole owner of every cone-room motion phase."""

    def __init__(self):
        # The base constructor installs the subscriber using this override.
        # Keep it inert until every inherited lock/event has been created.
        self._src2_ocr_state_ready = False
        self._src2_ocr_hits = dict(
            (category, []) for category in SRC2_CATEGORY_WORKSHOP)
        super(Src2FullRoomDeliveryV6, self).__init__()
        self.src2_ocr_required_hits = max(1, int(rospy.get_param(
            "~src2_target_required_hits", 2)))
        self.src2_ocr_window_s = max(0.05, float(rospy.get_param(
            "~src2_target_evidence_window_s", 1.5)))
        self.src2_scan_step = math.radians(max(1.0, float(rospy.get_param(
            "~src2_coverage_scan_step_deg", 20.0))))
        self.src2_scan_speed = max(0.08, abs(float(rospy.get_param(
            "~src2_coverage_scan_angular_speed_rps", 0.35))))
        self.src2_scan_dwell = max(0.0, float(rospy.get_param(
            "~src2_coverage_scan_dwell_s", 0.65)))
        self.src2_scan_max_dwell = max(
            self.src2_scan_dwell, float(rospy.get_param(
                "~src2_coverage_scan_max_dwell_s", 2.0)))
        self.src2_scan_pose_timeout = max(0.1, float(rospy.get_param(
            "~src2_coverage_scan_pose_timeout_s", 0.5)))
        self.src2_scan_timeout_margin = max(0.2, float(rospy.get_param(
            "~src2_coverage_scan_timeout_margin_s", 2.0)))
        self.src2_exact_anchor_retry_count = max(0, int(rospy.get_param(
            "~src2_exact_anchor_retry_count", 1)))
        self._apply_src2_exact_anchor_ids()
        self._src2_ocr_state_ready = True
        self.publish_state(
            "SRC2_FULL_ROOM_V6_READY",
            room_motion_owner="src2_only",
            target_decision_owner="src2_temporal_filter",
            scan_policy="20deg_stop_and_look",
            anchor_values="current_calibrated_values_unchanged")

    def _apply_src2_exact_anchor_ids(self):
        """Give the eight retained user anchors their unambiguous d labels."""
        identifiers = ("d1", "d2", "d3", "d5", "d6", "d7", "d8", "d9")
        if len(self.coverage_anchors) != len(identifiers):
            raise RuntimeError(
                "expected {} calibrated anchors, found {}".format(
                    len(identifiers), len(self.coverage_anchors)))
        for anchor, identifier in zip(self.coverage_anchors, identifiers):
            anchor["semantic_name"] = anchor["name"]
            anchor["name"] = identifier
            anchor["display_name"] = identifier
        self.wall_route_points = [
            (anchor["name"], anchor["x"], anchor["y"])
            for anchor in self.coverage_anchors]
        last = self.coverage_anchors[-1]
        self.wall_route_points.append(
            ("coverage_complete", last["x"], last["y"]))
        self.wall_route_yaws = [
            anchor["yaw"] for anchor in self.coverage_anchors] + [last["yaw"]]

    @staticmethod
    def _src2_payload_category(payload):
        category = str(payload.get("src2_category", "") or "").strip().lower()
        if category in SRC2_CATEGORY_WORKSHOP:
            return category
        label = str(payload.get("label", "") or "").strip()
        if label == "电子产品加工车间":
            label = "电子产品生产车间"
        return SRC2_WORKSHOP_CATEGORY.get(label, "")

    def _src2_reset_target_filter(self):
        self._src2_ocr_hits = dict(
            (category, []) for category in SRC2_CATEGORY_WORKSHOP)

    def _src2_push_category(self, observed, now):
        for category in self._src2_ocr_hits:
            self._src2_ocr_hits[category] = [
                stamp for stamp in self._src2_ocr_hits[category]
                if now - stamp <= self.src2_ocr_window_s]
        # Match src2 exactly: a blank frame preserves recent evidence, while a
        # definite competing category resets it.
        if not observed:
            return 0
        for category in self._src2_ocr_hits:
            if category != observed:
                self._src2_ocr_hits[category] = []
        self._src2_ocr_hits[observed].append(now)
        return len(self._src2_ocr_hits[observed])

    def _src2_cache_viewpoint(self, category, payload, hits):
        """Cache a confirmed src2 view for the second delivery only."""
        pose = self.current_pose()
        if pose is None:
            return
        workshop = SRC2_CATEGORY_WORKSHOP[category]
        bbox = payload.get("bbox")
        width = float(payload.get("image_width", 0.0) or 0.0)
        center_error = float("inf")
        if isinstance(bbox, (list, tuple)) and len(bbox) >= 4 and width > 1.0:
            center_error = abs(
                0.5 * (float(bbox[0]) + float(bbox[2])) - 0.5 * width)
        observation = {
            "label": workshop,
            "x": pose[0], "y": pose[1], "yaw": pose[2],
            "anchor_index": int(self.coverage_active_index),
            "votes": int(hits), "bbox": bbox,
            "stamp": time.time(), "monotonic": time.monotonic(),
            "center_error_px": center_error,
            "quality": center_error,
        }
        with self.lock:
            previous = self.workshop_observations.get(workshop)
            if previous is None or center_error <= previous.get(
                    "quality", float("inf")):
                self.workshop_observations[workshop] = observation
            if (self.delivery_stage == "real" and
                    workshop == self.sim_target_warehouse and
                    (self.sim_viewpoint_cache is None or
                     center_error <= self.sim_viewpoint_cache.get(
                         "quality", float("inf")))):
                self.sim_viewpoint_cache = dict(observation)

    def ocr_callback(self, msg):
        """Use the original src2 target rule; bypass every legacy OCR gate."""
        if not self._src2_ocr_state_ready:
            return
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        now = time.monotonic()
        observed = self._src2_payload_category(payload)
        bbox = payload.get("bbox")
        # The adapter only supplies a category for a detector-backed text box,
        # but keep this guard here so a label-only result can never park.
        if not isinstance(bbox, (list, tuple)) or len(bbox) < 4:
            observed = ""
        hits = self._src2_push_category(observed, now)
        if observed and hits >= self.src2_ocr_required_hits:
            self._src2_cache_viewpoint(observed, payload, hits)

        target_category = SRC2_WORKSHOP_CATEGORY.get(
            str(getattr(self, "target_warehouse", "") or ""), "")
        if (not self.room_search_active or not target_category or
                observed != target_category or
                hits < self.src2_ocr_required_hits):
            return

        workshop = SRC2_CATEGORY_WORKSHOP[target_category]
        accepted = dict(payload)
        accepted["label"] = workshop
        accepted["frame_label"] = workshop
        accepted["stable"] = True
        accepted["votes"] = int(hits)
        accepted["received_monotonic"] = now
        pose = self.current_pose()
        with self.lock:
            self.latest_ocr = accepted
            self.parking_target_ocr = dict(accepted)
            self.parking_target_stamp = now
            self.target_snapshot = {
                "ocr": dict(accepted),
                "pose": None if pose is None else tuple(pose),
            }
        self.candidate_event.clear()
        if not self.target_event.is_set():
            self.target_event.set()
            self.publish_state(
                "SRC2_TARGET_LOCKED", target=workshop,
                category=target_category, hits=hits,
                required=self.src2_ocr_required_hits,
                bbox=bbox, legacy_gate=False)

    def hold_ocr_candidate(self, goal_name):
        """src2 has no legacy candidate state: lock or continue scanning."""
        del goal_name
        self.candidate_event.clear()
        return self.target_event.is_set()

    def clear_target_lock_for_second_stage(self):
        super(Src2FullRoomDeliveryV6,
              self).clear_target_lock_for_second_stage()
        self._src2_reset_target_filter()

    def reset_target_and_resume_wall_scan(self, reason):
        resume = super(Src2FullRoomDeliveryV6,
                       self).reset_target_and_resume_wall_scan(reason)
        self._src2_reset_target_filter()
        return resume

    def _release_completed_room_ocr(self):
        """Release camera/NPU after room completion without delaying Gazebo."""
        process = self.ocr_process
        if process is None:
            return
        self.stop_process(process, "completed room OCR")
        if self.ocr_process is process:
            self.ocr_process = None
        self.publish_state("SRC2_ROOM_OCR_RELEASED_FOR_NEXT_STAGE")

    def publish_second_parking_success(self, same_workshop=False):
        # Publish first so UDP simulation handoff starts immediately.  OCR
        # shutdown then overlaps Gazebo execution and is complete long before
        # the post-simulation traffic-light stage needs the NPU.
        super(Src2FullRoomDeliveryV6, self).publish_second_parking_success(
            same_workshop=same_workshop)
        worker = threading.Thread(target=self._release_completed_room_ocr)
        worker.daemon = True
        worker.start()

    def _src2_scan_hold(self, anchor_name, step_index, step_count):
        """Keep zero velocity while OCR consumes stable frames at one yaw."""
        started = time.monotonic()
        deadline = started + self.src2_scan_dwell
        maximum = started + self.src2_scan_max_dwell
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            interrupted = self.motion_interrupted_by_ocr(
                "{}_src2_step_{}/{}".format(
                    anchor_name, step_index, step_count))
            if interrupted is not None:
                if interrupted == "TARGET":
                    self._remember_target_view_yaw()
                return interrupted
            # A candidate may have extended confirmation inside the inherited
            # OCR adapter.  Never exceed src2's bounded maximum dwell.
            deadline = min(max(deadline, time.monotonic()), maximum)
            self.cmd_pub.publish(Twist())
            rate.sleep()
        self.cmd_pub.publish(Twist())
        return "SUCCEEDED"

    @staticmethod
    def _split_scan_angle(total, step):
        values = []
        remaining = max(0.0, float(total))
        while remaining > 1.0e-6:
            value = min(float(step), remaining)
            values.append(value)
            remaining -= value
        return values

    def _src2_rotate_scan_step(self, anchor, step_index, step_count,
                               direction, step_angle):
        """Run one odometry-closed src2 scan step with immediate OCR stop."""
        with self.lock:
            start_yaw = self.odom_yaw
            odom_stamp = self.odom_stamp
        if start_yaw is None or time.monotonic() - odom_stamp > 0.6:
            self.stop_robot(5)
            return "NO_ODOM"

        deadline = (time.monotonic() +
                    step_angle / self.src2_scan_speed +
                    self.src2_scan_timeout_margin)
        maximum_progress = 0.0
        last_fresh = time.monotonic()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            interrupted = self.motion_interrupted_by_ocr(
                "{}_src2_turn_{}/{}".format(
                    anchor["name"], step_index, step_count))
            if interrupted is not None:
                if interrupted == "TARGET":
                    self._remember_target_view_yaw()
                return interrupted
            with self.lock:
                yaw = self.odom_yaw
                stamp = self.odom_stamp
                clearance = self.rotation_clearance
                scan_stamp = self.scan_stamp
            now = time.monotonic()
            if yaw is not None and now - stamp <= 0.6:
                last_fresh = now
                progress = normalize_angle(yaw - start_yaw) * direction
                maximum_progress = max(maximum_progress, progress)
                if maximum_progress >= step_angle:
                    self.stop_robot(4)
                    return "SUCCEEDED"
            elif now - last_fresh >= self.src2_scan_pose_timeout:
                self.stop_robot(5)
                return "NO_ODOM"
            if (now - scan_stamp > self.sweep_sensor_fresh_s or
                    clearance <= self.sweep_rotation_clearance):
                self.stop_robot(5)
                return "BLOCKED"
            command = Twist()
            command.angular.z = direction * self.src2_scan_speed
            self.cmd_pub.publish(command)
            rate.sleep()
        self.stop_robot(5)
        return "TIMEOUT"

    def _src2_inspect_once(self, anchor):
        """Execute the anchor's unchanged arc using src2 stop-and-look steps."""
        self.candidate_event.clear()
        self._coverage_suspect_frames = 0
        self._coverage_scan_active = True
        try:
            initial = self._src2_scan_hold(anchor["name"], 0, 0)
            if initial != "SUCCEEDED":
                return initial
            for sweep_index, (signed_angle, direction_name) in enumerate(
                    anchor["sweeps"], 1):
                direction = 1.0 if signed_angle >= 0.0 else -1.0
                steps = self._split_scan_angle(
                    abs(signed_angle), self.src2_scan_step)
                self.publish_state(
                    "SRC2_STEP_SCAN_START", anchor=anchor["name"],
                    sweep=sweep_index, direction=direction_name,
                    total_angle_deg=math.degrees(abs(signed_angle)),
                    steps=len(steps), speed=self.src2_scan_speed)
                for step_index, step_angle in enumerate(steps, 1):
                    result = self._src2_rotate_scan_step(
                        anchor, step_index, len(steps), direction, step_angle)
                    if result != "SUCCEEDED":
                        return result
                    result = self._src2_scan_hold(
                        anchor["name"], step_index, len(steps))
                    if result != "SUCCEEDED":
                        return result
                self.publish_state(
                    "SRC2_STEP_SCAN_COMPLETE", anchor=anchor["name"],
                    sweep=sweep_index)
            return "SUCCEEDED"
        finally:
            self._coverage_scan_active = False
            self._coverage_suspect_frames = 0
            if not self.target_event.is_set():
                self.candidate_event.clear()

    def _navigate_to_anchor(self, anchor, excluded=None):
        """Prefer the calibrated pose, then use the nearest rotatable pose."""
        if not self.ensure_ocr_running():
            return "OCR_UNAVAILABLE"
        excluded = list(excluded or [])
        result = "BLOCKED"
        attempts = max(
            1 + self.src2_exact_anchor_retry_count,
            self.src2_nearby_rotation_attempts)
        for attempt in range(1, attempts + 1):
            selected = self._nearby_scan_pose(anchor, excluded)
            if selected is None:
                self.publish_state(
                    "SRC2_ROTATABLE_ANCHOR_UNAVAILABLE",
                    anchor=anchor["name"], attempt=attempt)
                break
            distance, x, y, yaw = selected
            self.publish_state(
                "SRC2_ROTATABLE_ANCHOR_NAVIGATION", anchor=anchor["name"],
                attempt=attempt, x=x, y=y, yaw=yaw,
                nominal_x=anchor["x"], nominal_y=anchor["y"],
                fallback_distance=distance,
                substitute_point=distance >= 0.05)
            result = self._src2_navigation_goal(
                x, y, yaw,
                "src2_rotatable_anchor_{}_{}".format(
                    anchor["name"], attempt),
                timeout=self.src2_goal_hard_timeout, watch_target=True)
            if result == "TARGET":
                return result
            if result == "SUCCEEDED":
                rospy.sleep(0.18)
                if self._spin_pose_live_safe():
                    self._src2_last_anchor_pose = (x, y, yaw)
                    self.publish_state(
                        "SRC2_ROTATABLE_ANCHOR_READY",
                        anchor=anchor["name"], x=x, y=y, yaw=yaw,
                        fallback_distance=distance)
                    return result
                result = "ROTATION_CLEARANCE_BLOCKED"
                self.publish_state(
                    "SRC2_ROTATABLE_ANCHOR_REJECTED",
                    anchor=anchor["name"], attempt=attempt,
                    x=x, y=y, reason=result)
            excluded.append((x, y))
            if attempt < attempts:
                self.clear_and_wait(
                    "{} rotatable pose {} {}".format(
                        anchor["name"], attempt, result))
        return result

    def _inspect_anchor(self, anchor):
        """Relocate to the nearest rotatable pose if the sweep is blocked."""
        result = self._src2_inspect_once(anchor)
        recoverable = ("BLOCKED", "TIMEOUT", "NO_ODOM")
        if result not in recoverable:
            return result
        excluded = [(anchor["x"], anchor["y"])]
        if self._src2_last_anchor_pose is not None:
            excluded.append(self._src2_last_anchor_pose[:2])
        for attempt in range(1, self.src2_nearby_rotation_attempts + 1):
            self.publish_state(
                "SRC2_SCAN_RELOCATION_START", anchor=anchor["name"],
                attempt=attempt, reason=result)
            navigation = self._navigate_to_anchor(anchor, excluded=excluded)
            if navigation == "TARGET":
                return navigation
            if navigation == "OCR_UNAVAILABLE":
                return navigation
            if navigation != "SUCCEEDED":
                return "SRC2_SCAN_{}".format(result)
            pose = self.current_pose()
            if pose is not None:
                excluded.append((pose[0], pose[1]))
            result = self._src2_inspect_once(anchor)
            if result not in recoverable:
                return result
        self.publish_state(
            "SRC2_SCAN_RELOCATION_EXHAUSTED", anchor=anchor["name"],
            result=result)
        return "SRC2_SCAN_{}".format(result)

    def run_wall_route(self, start_index=0, approach_start=True):
        """Ordered coverage with calibrated-first rotatable-pose fallback."""
        del approach_start
        count = len(self.coverage_anchors)
        start_index = max(0, min(int(start_index), count))
        for index in range(start_index, count):
            self.coverage_active_index = index
            self.active_wall_segment_index = index
            anchor = self.coverage_anchors[index]
            self.publish_state(
                "SRC2_EXACT_ANCHOR_BEGIN", anchor=anchor["name"],
                anchor_index=index, x=anchor["x"], y=anchor["y"],
                yaw=anchor["yaw"])
            result = self._navigate_to_anchor(anchor)
            if result == "TARGET":
                return result
            if result != "SUCCEEDED":
                self.publish_state(
                    "SRC2_EXACT_ANCHOR_SKIPPED", anchor=anchor["name"],
                    result=result)
                continue
            result = self._inspect_anchor(anchor)
            if result == "TARGET":
                return result
            if result == "OCR_UNAVAILABLE":
                return result
            if result != "SUCCEEDED":
                self.publish_state(
                    "SRC2_EXACT_ANCHOR_PARTIAL", anchor=anchor["name"],
                    result=result)
                continue
            self.publish_state(
                "SRC2_EXACT_ANCHOR_COMPLETE", anchor=anchor["name"])
        return "SUCCEEDED"


if __name__ == "__main__":
    Src2FullRoomDeliveryV6()
    rospy.spin()
