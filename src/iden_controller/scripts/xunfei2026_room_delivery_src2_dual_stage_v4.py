#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Dual-stage room delivery using the ``~/src2`` navigation/parking core.

The calibrated d1..d9 anchors, their order and continuous sweep definitions
remain inherited unchanged.  The real-target parking -> announcement ->
simulation-target parking -> simulation trigger sequence also remains
unchanged.  Anchor travel and both parking operations use src2's watchdog,
staging, wall-fit, three-phase docking and full-footprint validation rules.
"""

import math
import time

import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseGoal

from xunfei2026_room_delivery_dual_stage_v3 import (
    DualStageRoomDeliveryManager,
)
from xunfei2026_src2_navigation_core_v1 import (
    build_quadrilateral_walls,
    docking_within_tolerance,
    fit_wall_line,
    normalize_angle,
    parking_footprint_margins,
    parking_goal_from_wall,
    ray_segment_intersection,
    wall_fit_is_continuous,
    wall_fit_matches_expected,
    wall_frame_docking_command,
)


class Src2DualStageRoomDeliveryManager(DualStageRoomDeliveryManager):
    def __init__(self):
        # shutdown() is registered by the legacy base constructor, so these
        # fields must exist even if that constructor aborts part-way through.
        self._src2_saved_recovery = None
        self._src2_move_base_client = None
        super(Src2DualStageRoomDeliveryManager, self).__init__()

        # src2 measured arena quadrilateral; unlike the old controller this is
        # not reduced to an axis-aligned bounding box when deriving a bay.
        corners = rospy.get_param("~src2_room_corners", [
            [-2.2311, -1.2505], [2.8000, -1.1940],
            [-2.2197, -3.2746], [2.7739, -3.2186],
        ])
        self.src2_walls = build_quadrilateral_walls(corners)

        # Exact src2 coverage watchdog policy.  The current anchor substitute
        # search is intentionally retained because it is part of the requested
        # d1..d9 flow when a random cone occupies the nominal coordinate.
        self.src2_goal_soft_timeout = max(1.0, float(rospy.get_param(
            "~src2_coverage_goal_soft_timeout_s", 25.0)))
        self.src2_goal_hard_timeout = max(
            self.src2_goal_soft_timeout, float(rospy.get_param(
                "~src2_coverage_goal_hard_timeout_s", 40.0)))
        self.src2_progress_window = max(1.0, float(rospy.get_param(
            "~src2_coverage_progress_window_s", 5.0)))
        self.src2_min_progress = max(0.01, float(rospy.get_param(
            "~src2_coverage_min_progress_m", 0.03)))
        self.src2_rotation_window = max(1.0, float(rospy.get_param(
            "~src2_coverage_rotation_window_s", 5.0)))
        self.src2_rotation_limit = math.radians(abs(float(rospy.get_param(
            "~src2_coverage_rotation_limit_deg", 90.0))))
        self.src2_anchor_position_tolerance = max(0.05, float(rospy.get_param(
            "~src2_anchor_position_tolerance_m", 0.12)))
        self.src2_anchor_yaw_tolerance = math.radians(abs(float(
            rospy.get_param("~src2_anchor_yaw_tolerance_deg", 10.0))))
        self.src2_anchor_yaw_timeout = max(2.0, float(rospy.get_param(
            "~src2_anchor_yaw_timeout_s", 12.0)))

        # src2 target centering: bounded odometry-closed steps with automatic
        # one-time steering-sign correction if image error grows.
        self.src2_center_tolerance = abs(float(rospy.get_param(
            "~src2_target_center_tolerance", 0.08)))
        self.src2_center_required_hits = max(2, int(rospy.get_param(
            "~src2_target_center_required_hits", 2)))
        self.src2_center_timeout = max(2.0, float(rospy.get_param(
            "~src2_target_center_timeout_s", 12.0)))
        self.src2_bbox_stale = max(0.2, float(rospy.get_param(
            "~src2_target_bbox_stale_s", 0.8)))
        self.src2_center_steering_sign = (1.0 if float(rospy.get_param(
            "~src2_target_center_steering_sign", -1.0)) >= 0.0 else -1.0)
        self.src2_center_coarse_step = math.radians(abs(float(rospy.get_param(
            "~src2_target_center_coarse_step_deg", 4.0))))
        self.src2_center_fine_step = math.radians(abs(float(rospy.get_param(
            "~src2_target_center_fine_step_deg", 2.0))))
        self.src2_center_fine_threshold = abs(float(rospy.get_param(
            "~src2_target_center_fine_threshold", 0.20)))
        self.src2_center_speed = abs(float(rospy.get_param(
            "~src2_target_center_speed_rps", 0.25)))
        self.src2_center_max_speed = max(self.src2_center_speed, abs(float(
            rospy.get_param("~src2_target_center_max_speed_rps", 0.35))))
        self.src2_center_settle = max(0.0, float(rospy.get_param(
            "~src2_target_center_settle_s", 0.25)))
        self.src2_center_reverse_threshold = abs(float(rospy.get_param(
            "~src2_target_center_reverse_threshold", 0.03)))

        # src2 parking values used by the competition integration.
        self.src2_box_width = abs(float(rospy.get_param(
            "~src2_parking_box_width_m", 0.50)))
        self.src2_box_depth = abs(float(rospy.get_param(
            "~src2_parking_box_depth_m", 0.50)))
        self.src2_goal_offset = abs(float(rospy.get_param(
            "~src2_parking_goal_offset_m", 0.26)))
        self.src2_staging_offset = abs(float(rospy.get_param(
            "~src2_parking_staging_offset_m", 0.55)))
        self.src2_staging_timeout = max(3.0, float(rospy.get_param(
            "~src2_parking_staging_timeout_s", 20.0)))
        self.src2_staging_position_tolerance = abs(float(rospy.get_param(
            "~src2_parking_staging_position_tolerance_m", 0.10)))
        self.src2_staging_yaw_tolerance = abs(float(rospy.get_param(
            "~src2_parking_staging_yaw_tolerance_rad", 0.10)))
        self.src2_staging_watchdog_window = max(0.5, float(rospy.get_param(
            "~src2_parking_staging_watchdog_window_s", 2.0)))
        self.src2_staging_max_rotation = math.radians(abs(float(
            rospy.get_param("~src2_parking_staging_max_rotation_deg", 45.0))))
        self.src2_docking_timeout = max(3.0, float(rospy.get_param(
            "~src2_parking_docking_timeout_s", 15.0)))
        self.src2_dock_max_x = abs(float(rospy.get_param(
            "~src2_parking_dock_max_x_mps", 0.10)))
        self.src2_dock_max_y = abs(float(rospy.get_param(
            "~src2_parking_dock_max_y_mps", 0.06)))
        self.src2_dock_max_yaw = abs(float(rospy.get_param(
            "~src2_parking_dock_max_yaw_rps", 0.15)))
        self.src2_dock_min_yaw = min(self.src2_dock_max_yaw, abs(float(
            rospy.get_param("~src2_parking_dock_min_yaw_rps", 0.15))))
        self.src2_normal_tolerance = abs(float(rospy.get_param(
            "~src2_parking_normal_tolerance_m", 0.015)))
        self.src2_tangent_tolerance = abs(float(rospy.get_param(
            "~src2_parking_tangent_tolerance_m", 0.020)))
        self.src2_yaw_tolerance = abs(float(rospy.get_param(
            "~src2_parking_yaw_tolerance_rad", 0.035)))
        self.src2_stable_s = max(0.2, float(rospy.get_param(
            "~src2_parking_stable_s", 0.50)))
        self.src2_min_wall_distance = abs(float(rospy.get_param(
            "~src2_parking_min_wall_distance_m", 0.19)))
        self.src2_lidar_stop_distance = abs(float(rospy.get_param(
            "~src2_parking_lidar_stop_distance_m", 0.15)))
        self.src2_recenter_wait = max(0.0, float(rospy.get_param(
            "~src2_parking_recenter_initial_wait_s", 1.0)))
        self.src2_recenter_tolerance = abs(float(rospy.get_param(
            "~src2_parking_recenter_tolerance", 0.04)))
        self.src2_wall_fit_half_angle = math.radians(abs(float(rospy.get_param(
            "~src2_parking_wall_fit_half_angle_deg", 35.0))))
        self.src2_wall_fit_min_points = max(4, int(rospy.get_param(
            "~src2_parking_wall_fit_min_points", 12)))
        self.src2_wall_fit_min_span = abs(float(rospy.get_param(
            "~src2_parking_wall_fit_min_span_m", 0.25)))
        self.src2_wall_fit_near_span = abs(float(rospy.get_param(
            "~src2_parking_wall_fit_near_min_span_m", 0.18)))
        self.src2_wall_fit_max_residual = abs(float(rospy.get_param(
            "~src2_parking_wall_fit_max_residual_m", 0.015)))
        self.src2_wall_fit_max_distance_jump = abs(float(rospy.get_param(
            "~src2_parking_wall_fit_max_distance_jump_m", 0.05)))
        self.src2_wall_fit_max_normal_jump = math.radians(abs(float(
            rospy.get_param("~src2_parking_wall_fit_max_normal_jump_deg", 8.0))))
        self.src2_wall_fit_max_normal_error = math.radians(abs(float(
            rospy.get_param("~src2_parking_wall_fit_max_normal_error_deg", 20.0))))
        self.src2_required_margin = max(0.0, float(rospy.get_param(
            "~src2_parking_required_margin_m", 0.02)))

        self.src2_wall_name = None
        self.src2_wall_point = None
        self.src2_inward_normal = None
        self.src2_final_goal = None
        self.src2_staging_goal = None
        self.src2_final_wall_fit = None
        self.src2_final_tangent_error = None
        self.src2_last_wall_fit = None
        self.src2_last_wall_fit_stamp = 0.0
        self.publish_state(
            "SRC2_DUAL_STAGE_V4_READY",
            anchors=[anchor["name"] for anchor in self.coverage_anchors],
            parking_operations=2,
            parking_core="src2_staging_wall_fit_three_phase")

    # ------------------------------------------------------------------
    # src2 move_base ownership and anchor navigation
    # ------------------------------------------------------------------
    def replace_move_base(self):
        result = super(Src2DualStageRoomDeliveryManager, self).replace_move_base()
        try:
            from dynamic_reconfigure.client import Client
            self._src2_move_base_client = Client("/move_base", timeout=3.0)
            current = self._src2_move_base_client.get_configuration(timeout=3.0)
            keys = ("recovery_behavior_enabled", "clearing_rotation_allowed")
            self._src2_saved_recovery = {
                key: bool(current[key]) for key in keys if key in current
            }
            applied = self._src2_move_base_client.update_configuration({
                "recovery_behavior_enabled": False,
                "clearing_rotation_allowed": False,
            })
            self.publish_state(
                "SRC2_COVERAGE_RECOVERY_DISABLED",
                recovery=applied.get("recovery_behavior_enabled"),
                clearing_rotation=applied.get("clearing_rotation_allowed"))
        except Exception as exc:
            self.publish_state(
                "SRC2_COVERAGE_RECOVERY_DISABLE_UNAVAILABLE", reason=str(exc))
        return result

    def _src2_wait_idle(self, timeout=2.0):
        active = (GoalStatus.PENDING, GoalStatus.ACTIVE,
                  GoalStatus.PREEMPTING, GoalStatus.RECALLING)
        deadline = time.monotonic() + max(0.1, float(timeout))
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            state = self.move_base.get_state()
            if state not in active:
                self.cmd_pub.publish(Twist())
                return True
            self.move_base.cancel_goal()
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.04)
        return self.move_base.get_state() not in active

    def _src2_align_yaw(self, target_yaw, watch_target=True):
        if not self._src2_wait_idle():
            return False
        deadline = time.monotonic() + self.src2_anchor_yaw_timeout
        stable_since = None
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if watch_target and self.target_event.is_set():
                self.stop_robot(5)
                return False
            pose = self.current_pose()
            if pose is None:
                self.stop_robot(5)
                return False
            error = normalize_angle(float(target_yaw) - pose[2])
            if abs(error) <= self.src2_anchor_yaw_tolerance:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= 0.35:
                    self.stop_robot(6)
                    return True
            else:
                stable_since = None
                command = Twist()
                command.angular.z = math.copysign(
                    min(0.35, max(0.15, 1.4 * abs(error))), error)
                self.cmd_pub.publish(command)
            rate.sleep()
        self.stop_robot(6)
        return False

    def _src2_navigation_goal(self, x, y, yaw, name, timeout=None,
                              watch_target=True, position_tolerance=None,
                              yaw_tolerance=None, staging=False):
        position_tolerance = (self.src2_anchor_position_tolerance
                              if position_tolerance is None else
                              abs(float(position_tolerance)))
        yaw_tolerance = (self.src2_anchor_yaw_tolerance
                         if yaw_tolerance is None else abs(float(yaw_tolerance)))
        hard_timeout = (self.src2_goal_hard_timeout if timeout is None else
                        max(1.0, float(timeout)))
        soft_timeout = min(self.src2_goal_soft_timeout, hard_timeout)
        goal = MoveBaseGoal()
        goal.target_pose = self.pose_message(x, y, yaw)
        self.move_base.send_goal(goal)
        self.publish_state(
            "SRC2_NAVIGATING", goal=name, x=x, y=y, yaw=yaw,
            soft_timeout=soft_timeout, hard_timeout=hard_timeout,
            staging=bool(staging))
        started = time.monotonic()
        samples = []
        rotation_started = started
        rotation_pose = None
        rotation_last_yaw = None
        rotation_accumulated = 0.0
        close_since = None
        rate = rospy.Rate(12)
        while not rospy.is_shutdown():
            if watch_target and self.target_event.is_set():
                self.move_base.cancel_goal()
                self._src2_wait_idle()
                return "TARGET"
            if watch_target and self.candidate_event.is_set():
                self.move_base.cancel_goal()
                self._src2_wait_idle()
                hold_started = time.monotonic()
                if self.hold_ocr_candidate(name):
                    return "TARGET"
                # A deliberate stationary OCR confirmation is not navigation
                # stagnation.  Excluding it from the soft/hard watchdog keeps
                # a valid anchor goal from timing out after several suspects.
                hold_elapsed = time.monotonic() - hold_started
                started += hold_elapsed
                self.move_base.send_goal(goal)
                samples = []
                rotation_pose = None
                rotation_last_yaw = None
                rotation_accumulated = 0.0
                rotation_started = time.monotonic()

            pose = self.current_pose()
            now = time.monotonic()
            if pose is not None:
                distance = math.hypot(x - pose[0], y - pose[1])
                yaw_error = abs(normalize_angle(yaw - pose[2]))
                samples.append((now, distance))
                cutoff = now - self.src2_progress_window
                samples = [item for item in samples if item[0] >= cutoff]
                window_progress = (0.0 if not samples else
                                   max(0.0, samples[0][1] - distance))
                if distance <= position_tolerance:
                    if yaw_error <= yaw_tolerance:
                        self.move_base.cancel_goal()
                        self._src2_wait_idle()
                        self.publish_state(
                            "SRC2_GOAL_ACCEPTED", goal=name,
                            distance=distance, yaw_error=yaw_error)
                        return "SUCCEEDED"
                    if close_since is None:
                        close_since = now
                    elif now - close_since >= 0.35:
                        self.move_base.cancel_goal()
                        self._src2_wait_idle()
                        if self._src2_align_yaw(yaw, watch_target):
                            return "SUCCEEDED"
                        return "TARGET" if self.target_event.is_set() else "YAW_FAILED"
                else:
                    close_since = None

                if rotation_pose is None:
                    rotation_pose = pose
                    rotation_last_yaw = pose[2]
                    rotation_started = now
                else:
                    rotation_accumulated += abs(normalize_angle(
                        pose[2] - rotation_last_yaw))
                    rotation_last_yaw = pose[2]
                rotation_window = (self.src2_staging_watchdog_window
                                   if staging else self.src2_rotation_window)
                rotation_limit = (self.src2_staging_max_rotation
                                  if staging else self.src2_rotation_limit)
                if now - rotation_started >= rotation_window:
                    moved = math.hypot(pose[0] - rotation_pose[0],
                                       pose[1] - rotation_pose[1])
                    if moved < self.src2_min_progress and \
                            rotation_accumulated > rotation_limit:
                        self.move_base.cancel_goal()
                        self._src2_wait_idle()
                        self.publish_state(
                            "SRC2_ROTATION_STALL", goal=name, moved=moved,
                            accumulated_yaw_deg=math.degrees(rotation_accumulated))
                        return "ROTATION_STALL"
                    rotation_pose = pose
                    rotation_last_yaw = pose[2]
                    rotation_accumulated = 0.0
                    rotation_started = now

                elapsed = now - started
                if elapsed >= hard_timeout:
                    self.move_base.cancel_goal()
                    self._src2_wait_idle()
                    return "HARD_TIMEOUT"
                if elapsed >= soft_timeout and window_progress < self.src2_min_progress:
                    self.move_base.cancel_goal()
                    self._src2_wait_idle()
                    return "SOFT_TIMEOUT"

            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                return "SUCCEEDED"
            if state not in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                return "FAILED_{}".format(state)
            rate.sleep()
        return "SHUTDOWN"

    def _navigate_to_anchor(self, anchor):
        if not self.ensure_ocr_running():
            return "OCR_UNAVAILABLE"
        selected = self._nearest_scan_pose(anchor)
        if selected is None:
            self.publish_state(
                "SRC2_ANCHOR_BLOCKED", anchor=anchor["name"],
                policy="nearest_reachable_unavailable_skip")
            return "BLOCKED"
        self.publish_state(
            "SRC2_ANCHOR_SELECTED", anchor=anchor["name"],
            nominal_x=anchor["x"], nominal_y=anchor["y"],
            selected_x=selected[0], selected_y=selected[1],
            selected_yaw=selected[2])
        previous = self._coverage_scan_active
        self._coverage_scan_active = True
        try:
            return self._src2_navigation_goal(
                selected[0], selected[1], selected[2],
                "src2_anchor_{}".format(anchor["name"]),
                timeout=self.src2_goal_hard_timeout, watch_target=True)
        finally:
            self._coverage_scan_active = previous

    def _send_anchor_goal_fast(self, x, y, yaw, name, timeout):
        return self._src2_navigation_goal(
            x, y, yaw, name, timeout=timeout, watch_target=True)

    def _send_parking_recovery_goal_fast(self, x, y, yaw, name, timeout):
        return self._src2_navigation_goal(
            x, y, yaw, name, timeout=timeout, watch_target=False)

    # ------------------------------------------------------------------
    # src2 closed-loop image centering and wall target lock
    # ------------------------------------------------------------------
    def _src2_target_sample(self):
        with self.lock:
            payload = None if self.latest_ocr is None else dict(self.latest_ocr)
            target = self.target_warehouse
        if not payload or time.monotonic() - float(payload.get(
                "received_monotonic", 0.0)) > self.src2_bbox_stale:
            return None
        if (not bool(payload.get("stable", False)) or
                str(payload.get("label", "")) != target or
                str(payload.get("frame_label", "")) != target):
            return None
        bbox = payload.get("bbox")
        width = float(payload.get("image_width", 0.0) or 0.0)
        if not isinstance(bbox, (list, tuple)) or len(bbox) < 4 or width <= 1.0:
            return None
        center = 0.5 * (float(bbox[0]) + float(bbox[2]))
        return ((center - 0.5 * width) / (0.5 * width),
                float(payload.get("received_monotonic", 0.0)), payload)

    def _src2_rotate_step(self, direction, target_angle):
        with self.lock:
            start_yaw = self.odom_yaw
            stamp = self.odom_stamp
        if start_yaw is None or time.monotonic() - stamp > 0.6:
            return False
        speed = min(self.src2_center_max_speed,
                    max(self.src2_center_speed, 0.20))
        deadline = time.monotonic() + target_angle / max(speed, 0.05) + 1.5
        maximum_progress = 0.0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                yaw = self.odom_yaw
                odom_stamp = self.odom_stamp
                clearance = self.rotation_clearance
            if yaw is None or time.monotonic() - odom_stamp > 0.6:
                self.stop_robot(5)
                return False
            progress = normalize_angle(yaw - start_yaw) * direction
            maximum_progress = max(maximum_progress, progress)
            if maximum_progress >= target_angle:
                self.stop_robot(5)
                return True
            if clearance <= self.sweep_rotation_clearance:
                self.stop_robot(5)
                self.publish_state(
                    "SRC2_CENTER_ROTATION_BLOCKED", clearance=clearance)
                return False
            command = Twist()
            command.angular.z = speed * direction
            self.cmd_pub.publish(command)
            rate.sleep()
        self.stop_robot(5)
        return False

    def align_target_for_parking(self):
        self.move_base.cancel_all_goals()
        if not self._src2_wait_idle():
            return False
        self.publish_state("SRC2_TARGET_CENTERING_START")
        deadline = time.monotonic() + self.src2_center_timeout
        steering_sign = self.src2_center_steering_sign
        stable = 0
        reversed_once = False
        must_improve_after_reverse = False
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.parking_wrong_event.is_set():
                return False
            sample = self._src2_target_sample()
            if sample is None:
                self.stop_robot(3)
                rospy.sleep(0.04)
                continue
            before_error, before_stamp, _payload = sample
            if abs(before_error) <= self.src2_center_tolerance:
                stable += 1
                self.stop_robot(2)
                if stable >= self.src2_center_required_hits:
                    self.publish_state(
                        "SRC2_TARGET_CENTERED", normalized_error=before_error,
                        hits=stable)
                    return True
                rospy.sleep(0.05)
                continue
            stable = 0
            step = (self.src2_center_fine_step
                    if abs(before_error) <= self.src2_center_fine_threshold
                    else self.src2_center_coarse_step)
            direction = steering_sign * math.copysign(1.0, before_error)
            self.publish_state(
                "SRC2_TARGET_CENTER_STEP", error=before_error,
                step_deg=math.degrees(step), direction=direction)
            if not self._src2_rotate_step(direction, step):
                return False
            self.stop_robot(3)
            rospy.sleep(self.src2_center_settle)
            fresh_deadline = min(deadline, time.monotonic() + self.src2_bbox_stale)
            after = None
            while not rospy.is_shutdown() and time.monotonic() < fresh_deadline:
                candidate = self._src2_target_sample()
                if candidate is not None and candidate[1] > before_stamp:
                    after = candidate
                    break
                rospy.sleep(0.03)
            if after is None:
                return False
            improvement = abs(before_error) - abs(after[0])
            if must_improve_after_reverse:
                if improvement <= 0.0:
                    return False
                must_improve_after_reverse = False
            elif improvement < -self.src2_center_reverse_threshold:
                if reversed_once:
                    return False
                steering_sign *= -1.0
                reversed_once = True
                must_improve_after_reverse = True
                self.publish_state(
                    "SRC2_TARGET_CENTER_DIRECTION_REVERSED",
                    steering_sign=steering_sign)
        self.stop_robot(5)
        self.publish_state("SRC2_TARGET_CENTERING_TIMEOUT")
        return False

    def _src2_compute_wall_goals(self):
        pose = self.current_pose()
        if pose is None:
            return False
        direction = (math.cos(pose[2]), math.sin(pose[2]))
        best = None
        for name, start, end, normal in self.src2_walls:
            distance = ray_segment_intersection(
                (pose[0], pose[1]), direction, start, end)
            if distance is not None and (best is None or distance < best[0]):
                best = (distance, name, normal,
                        (pose[0] + distance * direction[0],
                         pose[1] + distance * direction[1]))
        if best is None:
            self.publish_state("SRC2_WALL_RAY_FAILED")
            return False
        _distance, name, normal, point = best
        self.src2_wall_name = name
        self.src2_wall_point = point
        self.src2_inward_normal = normal
        self.src2_final_goal = parking_goal_from_wall(
            point, normal, self.src2_goal_offset)
        self.src2_staging_goal = parking_goal_from_wall(
            point, normal, self.src2_staging_offset)
        self.parking_target_wall_yaw = self.src2_final_goal[2]
        self.publish_state(
            "SRC2_WALL_TARGET_LOCKED", wall=name,
            wall_x=point[0], wall_y=point[1],
            inward_x=normal[0], inward_y=normal[1],
            staging_x=self.src2_staging_goal[0],
            staging_y=self.src2_staging_goal[1],
            final_x=self.src2_final_goal[0],
            final_y=self.src2_final_goal[1],
            final_yaw=self.src2_final_goal[2])
        return True

    def refresh_centered_target_wall(self):
        return self._src2_compute_wall_goals()

    def approach_target(self):
        if self.src2_staging_goal is None:
            return False
        x, y, yaw = self.src2_staging_goal
        result = self._src2_navigation_goal(
            x, y, yaw, "src2_parking_staging",
            timeout=self.src2_staging_timeout, watch_target=False,
            position_tolerance=self.src2_staging_position_tolerance,
            yaw_tolerance=self.src2_staging_yaw_tolerance,
            staging=True)
        self.publish_state("SRC2_PARKING_STAGING_RESULT", result=result)
        return result == "SUCCEEDED"

    def handoff_directly_to_parking(self):
        self.move_base.cancel_all_goals()
        idle = self._src2_wait_idle()
        self.stop_robot(5)
        self.publish_state("SRC2_DIRECT_DOCK_HANDOFF", move_base_idle=idle)
        return idle

    # ------------------------------------------------------------------
    # src2 robust wall fit, three-phase docking and footprint validation
    # ------------------------------------------------------------------
    def _src2_wall_points(self):
        samples, stamp = self.scan_snapshot()
        if time.monotonic() - stamp > 0.6:
            return [], None
        points = [(sample[2], sample[3]) for sample in samples
                  if abs(sample[0]) <= self.src2_wall_fit_half_angle]
        front = [sample[1] for sample in samples
                 if abs(sample[0]) <= math.radians(15.0)]
        return points, (None if not front else min(front))

    def _src2_fit_wall(self, expected_base_angle):
        points, front = self._src2_wall_points()
        fit = fit_wall_line(
            points, self.src2_wall_fit_min_points,
            self.src2_wall_fit_min_span,
            self.src2_wall_fit_max_residual)
        if fit and wall_fit_matches_expected(
                fit, expected_base_angle, self.src2_wall_fit_max_normal_error):
            self.src2_last_wall_fit = fit
            self.src2_last_wall_fit_stamp = time.monotonic()
            return fit, front
        if (self.src2_last_wall_fit is not None and
                time.monotonic() - self.src2_last_wall_fit_stamp <= 0.6):
            near = fit_wall_line(
                points, self.src2_wall_fit_min_points,
                self.src2_wall_fit_near_span,
                self.src2_wall_fit_max_residual)
            if (near and wall_fit_matches_expected(
                    near, expected_base_angle,
                    self.src2_wall_fit_max_normal_error) and
                    wall_fit_is_continuous(
                        near, self.src2_last_wall_fit,
                        self.src2_wall_fit_max_distance_jump,
                        self.src2_wall_fit_max_normal_jump)):
                self.src2_last_wall_fit = near
                self.src2_last_wall_fit_stamp = time.monotonic()
                return near, front
            return self.src2_last_wall_fit, front
        return None, front

    def _src2_map_target_in_odom(self):
        map_pose = self.current_pose()
        with self.lock:
            odom_pose = None if self.odom_pose is None else tuple(self.odom_pose)
            odom_stamp = self.odom_stamp
        if (map_pose is None or odom_pose is None or
                time.monotonic() - odom_stamp > 0.6 or
                self.src2_final_goal is None or
                self.src2_inward_normal is None):
            return None
        map_from_odom_yaw = normalize_angle(map_pose[2] - odom_pose[2])
        cosine = math.cos(map_from_odom_yaw)
        sine = math.sin(map_from_odom_yaw)

        def map_vector_to_odom(dx, dy):
            return (cosine * dx + sine * dy,
                    -sine * dx + cosine * dy)

        delta = map_vector_to_odom(
            self.src2_final_goal[0] - map_pose[0],
            self.src2_final_goal[1] - map_pose[1])
        target_odom = (odom_pose[0] + delta[0], odom_pose[1] + delta[1])
        inward_odom = map_vector_to_odom(
            self.src2_inward_normal[0], self.src2_inward_normal[1])
        length = math.hypot(inward_odom[0], inward_odom[1])
        if length <= 1.0e-6:
            return None
        inward_odom = (inward_odom[0] / length, inward_odom[1] / length)
        return target_odom, inward_odom

    def _src2_dock(self):
        transformed = self._src2_map_target_in_odom()
        if transformed is None or not self._src2_wait_idle():
            return False
        target, inward = transformed
        outward = (-inward[0], -inward[1])
        tangent = (-outward[1], outward[0])
        desired_distance = max(
            self.src2_min_wall_distance, self.src2_goal_offset)
        self.src2_last_wall_fit = None
        self.src2_last_wall_fit_stamp = 0.0
        self.src2_final_wall_fit = None
        self.src2_final_tangent_error = None
        stable_since = None
        rotation_start = None
        rotation_yaw = None
        deadline = time.monotonic() + self.src2_docking_timeout
        fit_wait_started = time.monotonic()
        rate = rospy.Rate(20)
        self.parking_active = True
        self.publish_state(
            "SRC2_PARKING_DOCKING", wall=self.src2_wall_name,
            desired_wall_distance=desired_distance,
            control_order="rotate_then_tangent_then_forward")
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                if self.parking_wrong_event.is_set():
                    return False
                with self.lock:
                    pose = None if self.odom_pose is None else tuple(self.odom_pose)
                    odom_stamp = self.odom_stamp
                if pose is None or time.monotonic() - odom_stamp > 0.6:
                    return False
                expected = normalize_angle(
                    math.atan2(outward[1], outward[0]) - pose[2])
                fit, front = self._src2_fit_wall(expected)
                if fit is None:
                    self.cmd_pub.publish(Twist())
                    if time.monotonic() - fit_wait_started > 0.6:
                        self.publish_state("SRC2_PARKING_WALL_FIT_FAILED")
                        return False
                    rate.sleep()
                    continue
                fit_wait_started = time.monotonic()
                tangent_error = ((target[0] - pose[0]) * tangent[0] +
                                 (target[1] - pose[1]) * tangent[1])
                normal_error = fit["distance"] - desired_distance
                yaw_error = normalize_angle(fit["normal_angle"])
                errors = (normal_error, tangent_error, yaw_error)
                if docking_within_tolerance(
                        errors, self.src2_normal_tolerance,
                        self.src2_tangent_tolerance,
                        self.src2_yaw_tolerance):
                    self.cmd_pub.publish(Twist())
                    if stable_since is None:
                        stable_since = time.monotonic()
                    elif time.monotonic() - stable_since >= self.src2_stable_s:
                        self.src2_final_wall_fit = dict(fit)
                        self.src2_final_tangent_error = tangent_error
                        self.publish_state(
                            "SRC2_PARKING_CONVERGED",
                            normal_error=normal_error,
                            tangent_error=tangent_error,
                            yaw_error=yaw_error,
                            stable_s=self.src2_stable_s)
                        return True
                    rate.sleep()
                    continue
                stable_since = None
                if fit["distance"] < self.src2_min_wall_distance:
                    self.publish_state(
                        "SRC2_PARKING_HARD_WALL_STOP",
                        distance=fit["distance"],
                        limit=self.src2_min_wall_distance)
                    return False
                if (front is None or front < self.src2_lidar_stop_distance or
                        front < fit["distance"] - 0.08):
                    self.publish_state(
                        "SRC2_PARKING_NEAR_OBSTACLE_STOP", front=front,
                        fitted_wall_distance=fit["distance"])
                    return False
                command = wall_frame_docking_command(
                    normal_error, tangent_error, yaw_error,
                    self.src2_normal_tolerance,
                    self.src2_tangent_tolerance,
                    self.src2_yaw_tolerance,
                    self.src2_dock_max_x, self.src2_dock_max_y,
                    self.src2_dock_max_yaw, self.src2_dock_min_yaw)
                if abs(command[2]) > 0.0:
                    if rotation_yaw is None:
                        rotation_yaw = pose[2]
                        rotation_start = time.monotonic()
                    elif time.monotonic() - rotation_start >= 0.6:
                        if abs(normalize_angle(pose[2] - rotation_yaw)) < \
                                math.radians(0.5):
                            self.publish_state(
                                "SRC2_PARKING_ROTATION_NO_PROGRESS")
                            return False
                        rotation_yaw = pose[2]
                        rotation_start = time.monotonic()
                else:
                    rotation_yaw = None
                    rotation_start = None
                twist = Twist()
                twist.linear.x = command[0]
                twist.linear.y = command[1]
                twist.angular.z = command[2]
                self.cmd_pub.publish(twist)
                rospy.logwarn_throttle(
                    0.5,
                    "SRC2_DOCK errors=(%+.3f,%+.3f,%+.3f) cmd=(%+.3f,%+.3f,%+.3f) fit=(d=%.3f span=%.3f rms=%.4f n=%d)",
                    normal_error, tangent_error, yaw_error,
                    command[0], command[1], command[2], fit["distance"],
                    fit["span"], fit["residual"], fit["inliers"])
                rate.sleep()
        finally:
            self.parking_active = False
            self.cmd_pub.publish(Twist())
        self.publish_state("SRC2_PARKING_DOCKING_TIMEOUT")
        return False

    def _src2_validate_parking(self):
        if self.src2_final_wall_fit is None or \
                self.src2_final_tangent_error is None:
            return False
        fit = self.src2_final_wall_fit
        local_pose = (
            float(fit["distance"]),
            -float(self.src2_final_tangent_error),
            math.pi - float(fit["normal_angle"]),
        )
        diagnostics = parking_footprint_margins(
            local_pose, (0.0, 0.0), (1.0, 0.0),
            self.src2_box_width, self.src2_box_depth,
            self.robot_half_length, self.robot_half_width, 0.0)
        minimum_margin = min(
            float(diagnostics.get("near_margin", float("-inf"))),
            float(diagnostics.get("far_margin", float("-inf"))),
            float(diagnostics.get("side_margin", float("-inf"))))
        valid = (bool(diagnostics.get("inside")) and
                 minimum_margin >= self.src2_required_margin)
        self.publish_state(
            "SRC2_PARKING_FOOTPRINT_VALIDATION", valid=valid,
            wall_distance=fit["distance"],
            tangent_error=self.src2_final_tangent_error,
            yaw_error=fit["normal_angle"],
            near_margin=diagnostics.get("near_margin"),
            far_margin=diagnostics.get("far_margin"),
            side_margin=diagnostics.get("side_margin"),
            minimum_margin=minimum_margin,
            required_margin=self.src2_required_margin)
        return valid

    def park(self):
        # src2 optionally refreshes the image bearing at the 0.55 m staging
        # pose.  No fresh box means preserve the first physical wall lock.
        recenter_deadline = time.monotonic() + self.src2_recenter_wait
        fresh = None
        while not rospy.is_shutdown() and time.monotonic() < recenter_deadline:
            fresh = self._src2_target_sample()
            if fresh is not None:
                break
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.04)
        if fresh is not None:
            original_tolerance = self.src2_center_tolerance
            self.src2_center_tolerance = self.src2_recenter_tolerance
            try:
                if not self.align_target_for_parking():
                    return "REACQUIRE"
                if not self._src2_compute_wall_goals():
                    return "REACQUIRE"
            finally:
                self.src2_center_tolerance = original_tolerance
        else:
            self.publish_state(
                "SRC2_PARKING_RECENTER_SKIPPED",
                wait_s=self.src2_recenter_wait,
                policy="preserve_first_wall_lock")
        if not self._src2_dock():
            return "REACQUIRE"
        if not self._src2_validate_parking():
            return "REACQUIRE"
        self.stop_robot(10)
        return "SUCCEEDED"

    def shutdown(self):
        if self._src2_saved_recovery and self._src2_move_base_client is not None:
            try:
                self._src2_move_base_client.update_configuration(
                    self._src2_saved_recovery)
            except Exception:
                pass
        super(Src2DualStageRoomDeliveryManager, self).shutdown()


if __name__ == "__main__":
    Src2DualStageRoomDeliveryManager()
    rospy.spin()
