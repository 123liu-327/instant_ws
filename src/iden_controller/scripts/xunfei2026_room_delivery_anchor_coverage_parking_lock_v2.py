#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Anchor coverage with a latched, verified parking target.

The coverage-v1 scanner remains untouched.  Once that scanner has confirmed
the requested sign, centred it, and locked the matching lidar wall, an
edge/adjacent sign entering the camera must not invalidate the physical wall
being parked against.  Before that lock exists, all original negative OCR
rejection rules remain active.
"""

import json
import math

import rospy

from xunfei2026_room_delivery_anchor_coverage_v1 import (
    AnchorCoverageRoomDeliveryManager,
)
from xunfei2026_room_delivery_manager_v1 import canonical_workshop


class ParkingLockAnchorCoverageManager(AnchorCoverageRoomDeliveryManager):
    """Keep a fully verified target immutable during final geometric parking."""

    def __init__(self):
        self._continuous_ocr_announced = False
        self._anchor_navigation_limited = False
        super(ParkingLockAnchorCoverageManager, self).__init__()

        # The coverage-v1 defaults were tuned for fastest possible traversal:
        # only 0.08 s stationary at an anchor and a 3 s replan watchdog.  This
        # variant deliberately gives the camera/OCR a stable half-second view
        # and does not abandon a temporarily constrained TEB path too early.
        self.coverage_dwell_s = max(0.50, float(rospy.get_param(
            "~parking_lock_anchor_dwell_s", 0.50)))
        self.coverage_fast_stuck_s = max(4.50, float(rospy.get_param(
            "~parking_lock_anchor_stuck_s", 4.50)))

        self.anchor_max_vel_x = max(0.20, float(rospy.get_param(
            "~parking_lock_anchor_max_vel_x", 0.48)))
        self.anchor_max_vel_y = max(0.20, float(rospy.get_param(
            "~parking_lock_anchor_max_vel_y", 0.48)))
        self.anchor_max_vel_backwards = max(0.15, float(rospy.get_param(
            "~parking_lock_anchor_max_vel_backwards", 0.38)))
        self.anchor_max_vel_theta = max(0.45, float(rospy.get_param(
            "~parking_lock_anchor_max_vel_theta", 1.00)))
        self.anchor_acc_lim_x = max(0.20, float(rospy.get_param(
            "~parking_lock_anchor_acc_lim_x", 0.38)))
        self.anchor_acc_lim_y = max(0.20, float(rospy.get_param(
            "~parking_lock_anchor_acc_lim_y", 0.38)))
        self.anchor_acc_lim_theta = max(0.45, float(rospy.get_param(
            "~parking_lock_anchor_acc_lim_theta", 0.90)))

    def replace_move_base(self):
        """Keep the same planner, with calmer limits for inspection travel."""
        result = super(
            ParkingLockAnchorCoverageManager, self).replace_move_base()
        try:
            from dynamic_reconfigure.client import Client
            client = Client("/move_base/TebLocalPlannerROS", timeout=2.0)
            applied = client.update_configuration({
                "max_vel_x": self.anchor_max_vel_x,
                "max_vel_x_backwards": self.anchor_max_vel_backwards,
                "max_vel_y": self.anchor_max_vel_y,
                "max_vel_theta": self.anchor_max_vel_theta,
                "acc_lim_x": self.anchor_acc_lim_x,
                "acc_lim_y": self.anchor_acc_lim_y,
                "acc_lim_theta": self.anchor_acc_lim_theta,
            })
            self._anchor_navigation_limited = True
            self.publish_state(
                "ANCHOR_NAVIGATION_CALM_LIMITS_ACTIVE",
                max_vel_x=applied.get("max_vel_x", self.anchor_max_vel_x),
                max_vel_y=applied.get("max_vel_y", self.anchor_max_vel_y),
                max_vel_theta=applied.get(
                    "max_vel_theta", self.anchor_max_vel_theta),
                anchor_dwell_s=self.coverage_dwell_s,
                stuck_s=self.coverage_fast_stuck_s)
        except Exception as exc:
            # Navigation remains usable with its loaded TEB profile; failure to
            # lower a limit is visible rather than aborting the mission.
            self.publish_state(
                "ANCHOR_NAVIGATION_CALM_LIMITS_UNAVAILABLE", reason=str(exc),
                anchor_dwell_s=self.coverage_dwell_s)
        return result

    def _navigate_to_anchor(self, anchor, excluded=None):
        """Verify OCR health without toggling it or clearing accumulated votes."""
        if not self.ensure_ocr_running():
            self.publish_state(
                "OCR_CONTINUOUS_NAVIGATION_UNAVAILABLE",
                anchor=anchor.get("name", ""))
            return "OCR_UNAVAILABLE"
        if not self._continuous_ocr_announced:
            # start_ocr() already sent reset+enable.  Re-publishing enable at
            # every anchor would clear the OCR vote window, so continuous OCR
            # means intentionally sending no further enable/disable command.
            self._continuous_ocr_announced = True
            self.publish_state(
                "OCR_CONTINUOUS_NAVIGATION_ACTIVE",
                policy="no_toggle_between_anchors",
                anchor_dwell_s=self.coverage_dwell_s)
        return super(
            ParkingLockAnchorCoverageManager, self)._navigate_to_anchor(
                anchor, excluded=excluded)

    def _observation_dwell(self, anchor_name, phase):
        self.publish_state(
            "SCAN_ANCHOR_STABILIZING", anchor=anchor_name, phase=phase,
            dwell_s=self.coverage_dwell_s, ocr="continuous")
        return super(
            ParkingLockAnchorCoverageManager, self)._observation_dwell(
                anchor_name, phase)

    def ocr_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return super(ParkingLockAnchorCoverageManager, self).ocr_callback(
                msg)

        stable = bool(payload.get("stable", False))
        votes = int(payload.get("votes", 0) or 0)
        label = canonical_workshop(payload.get("label", ""))
        frame_label = canonical_workshop(payload.get("frame_label", ""))
        with self.lock:
            target = self.target_warehouse
            parking_active = self.parking_active
            target_snapshot_locked = self.target_snapshot is not None
            wall_yaw_locked = self.parking_target_wall_yaw is not None
            target_ocr_locked = self.parking_target_ocr is not None

        contradictory = (
            stable and target and label and label != target and
            frame_label == label and votes >= self.required_ocr_votes)
        verified_parking_lock = (
            parking_active and target_snapshot_locked and wall_yaw_locked and
            target_ocr_locked)

        if verified_parking_lock and contradictory:
            # Do not pass this observation to the base callback: it would set
            # parking_wrong_event immediately and discard a target that has
            # already passed stationary OCR, visual centring and lidar-wall
            # validation.  The parking controller continues using its locked
            # target centre and live lidar geometry.
            rospy.logwarn_throttle(
                0.35,
                "OCR_PARKING_ADJACENT_SIGN_IGNORED label=%s target=%s "
                "votes=%d verified_target_lock=true parking_continues=true",
                label, target, votes)
            return

        return super(ParkingLockAnchorCoverageManager, self).ocr_callback(msg)

    def refresh_centered_target_wall(self):
        """Retry a failed lidar wall lock from the nearest cardinal yaw.

        A visually centred and stably verified workshop sign is not discarded
        merely because the first oblique lidar fit has too few wall points.
        Normal lidar fitting remains authoritative; this fallback only turns
        to the closest 0/90/180/-90 degree heading and runs the same fit again.
        """
        if super(ParkingLockAnchorCoverageManager,
                 self).refresh_centered_target_wall():
            return True

        pose = self.current_pose()
        with self.lock:
            live = (None if self.latest_ocr is None else
                    dict(self.latest_ocr))
            locked = (None if self.parking_target_ocr is None else
                      dict(self.parking_target_ocr))
            target = self.target_warehouse

        def valid_target_payload(value):
            return (
                isinstance(value, dict) and
                bool(value.get("stable", False)) and
                canonical_workshop(value.get("label", "")) == target and
                canonical_workshop(value.get("frame_label", "")) == target)

        # Use the same source priority as the base wall-lock routine.  The
        # stationary five-frame confirmation can promote latest_ocr after the
        # earlier parking snapshot was captured, so checking only the older
        # snapshot incorrectly rejected a genuinely verified target.
        payload = live if valid_target_payload(live) else locked
        verified_target = pose is not None and valid_target_payload(payload)
        if not verified_target:
            self.publish_state(
                "CARDINAL_WALL_FALLBACK_REJECTED",
                reason="target_not_stably_locked")
            return False

        quarter_turn = math.pi / 2.0
        target_yaw = self._normalize_cardinal_yaw(
            round(pose[2] / quarter_turn) * quarter_turn)
        self.publish_state(
            "CARDINAL_WALL_FALLBACK_START",
            target_yaw=target_yaw,
            target_yaw_deg=int(round(math.degrees(target_yaw))))
        result = self.turn_to_wall(
            "centered_target_cardinal_fallback", target_yaw,
            watch_ocr=False)
        if result != "SUCCEEDED":
            self.publish_state(
                "CARDINAL_WALL_FALLBACK_TURN_FAILED",
                result=result, target_yaw=target_yaw)
            return False

        locked = super(ParkingLockAnchorCoverageManager,
                       self).refresh_centered_target_wall()
        self.publish_state(
            "CARDINAL_WALL_FALLBACK_LOCKED" if locked else
            "CARDINAL_WALL_FALLBACK_NO_WALL",
            target_yaw=target_yaw,
            target_yaw_deg=int(round(math.degrees(target_yaw))))
        return locked

    @staticmethod
    def _normalize_cardinal_yaw(yaw):
        """Return an equivalent cardinal yaw in [-pi, pi]."""
        while yaw > math.pi:
            yaw -= 2.0 * math.pi
        while yaw < -math.pi:
            yaw += 2.0 * math.pi
        return yaw


if __name__ == "__main__":
    ParkingLockAnchorCoverageManager()
    rospy.spin()
