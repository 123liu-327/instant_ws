#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""v6 parking: lock the real floor-frame centre before src2 lidar docking."""

import math
import statistics
import time

import rospy
from sensor_msgs.msg import Image

from factory_room_vision_core import decode_ros_image
from factory_room_vision_parking_v4 import CenterlineParkingDetector
from xunfei2026_room_delivery_src2_dual_stage_v5 import (
    Src2DualStageRoomDeliveryV5,
)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class Src2FrameCenterRoomDeliveryV6(Src2DualStageRoomDeliveryV5):
    def __init__(self):
        # The subscriber is created only after the inherited lock exists, but
        # shutdown/error paths may still inspect these fields during init.
        self.src2_frame_image = None
        self.src2_frame_image_stamp = 0.0
        self.src2_frame_locked = False
        self.src2_frame_lock = None
        super(Src2FrameCenterRoomDeliveryV6, self).__init__()

        self.src2_frame_detector = CenterlineParkingDetector()
        self.src2_frame_topic = str(rospy.get_param(
            "~src2_parking_frame_image_topic", "/ucar_camera/image_raw"))
        self.src2_frame_timeout = max(0.8, float(rospy.get_param(
            "~src2_parking_frame_acquire_timeout_s", 2.8)))
        self.src2_frame_required = max(3, int(rospy.get_param(
            "~src2_parking_frame_required_frames", 4)))
        self.src2_frame_stale = max(0.15, float(rospy.get_param(
            "~src2_parking_frame_image_stale_s", 0.55)))
        self.src2_frame_min_width = abs(float(rospy.get_param(
            "~src2_parking_frame_min_width_m", 0.36)))
        self.src2_frame_max_width = abs(float(rospy.get_param(
            "~src2_parking_frame_max_width_m", 0.64)))
        self.src2_frame_max_lateral = abs(float(rospy.get_param(
            "~src2_parking_frame_max_lateral_m", 0.28)))
        self.src2_frame_max_heading = math.radians(abs(float(rospy.get_param(
            "~src2_parking_frame_max_heading_deg", 18.0))))
        self.src2_frame_lateral_coherence = abs(float(rospy.get_param(
            "~src2_parking_frame_lateral_coherence_m", 0.055)))
        self.src2_frame_heading_coherence = math.radians(abs(float(
            rospy.get_param(
                "~src2_parking_frame_heading_coherence_deg", 6.0))))
        self.src2_frame_correction_limit = abs(float(rospy.get_param(
            "~src2_parking_frame_correction_limit_m", 0.22)))
        self.src2_frame_lateral_sign = (1.0 if float(rospy.get_param(
            "~src2_parking_frame_lateral_to_dock_tangent_sign", -1.0
        )) >= 0.0 else -1.0)
        self.src2_frame_required_for_success = bool(rospy.get_param(
            "~src2_parking_frame_required_for_success", True))
        self.src2_frame_subscriber = rospy.Subscriber(
            self.src2_frame_topic, Image, self._src2_frame_image_callback,
            queue_size=1, buff_size=4 * 1024 * 1024)
        self.publish_state(
            "SRC2_FRAME_CENTER_V6_READY",
            parking_center_source="floor_rails_and_crossbar",
            fallback_center_source="reacquire_not_ocr_projection",
            required_frames=self.src2_frame_required,
            frame_required=self.src2_frame_required_for_success)

    def _src2_frame_image_callback(self, message):
        image = decode_ros_image(message)
        if image is None:
            return
        with self.lock:
            self.src2_frame_image = image
            self.src2_frame_image_stamp = time.monotonic()

    def _src2_frame_snapshot(self):
        with self.lock:
            image = (None if self.src2_frame_image is None else
                     self.src2_frame_image.copy())
            stamp = self.src2_frame_image_stamp
        if image is None or time.monotonic() - stamp > self.src2_frame_stale:
            return None, stamp
        return image, stamp

    def _src2_frame_result_valid(self, result):
        if not result or not bool(result.get("found", False)):
            return False
        width = result.get("rail_width_m")
        lateral = result.get("lateral_error_m")
        heading = result.get("heading_error_rad")
        if width is None or lateral is None or heading is None:
            return False
        return (
            self.src2_frame_min_width <= float(width) <=
            self.src2_frame_max_width and
            abs(float(lateral)) <= self.src2_frame_max_lateral and
            abs(float(heading)) <= self.src2_frame_max_heading and
            int(result.get("vertical_count", 0)) >= 2 and
            int(result.get("horizontal_count", 0)) >= 1)

    def _src2_acquire_floor_frame(self):
        """Require a coherent physical bay centre, never a single-frame line."""
        deadline = time.monotonic() + self.src2_frame_timeout
        samples = []
        last_stamp = 0.0
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            image, stamp = self._src2_frame_snapshot()
            if image is None or stamp <= last_stamp:
                self.cmd_pub.publish(self._zero_twist())
                rate.sleep()
                continue
            last_stamp = stamp
            result = self.src2_frame_detector.detect(image)
            if not self._src2_frame_result_valid(result):
                samples = []
                rospy.logwarn_throttle(
                    0.45,
                    "SRC2_FRAME_CENTER_WAIT found=%s rails=%s bars=%s width=%s",
                    str(bool(result and result.get("found"))),
                    str(result.get("vertical_count") if result else None),
                    str(result.get("horizontal_count") if result else None),
                    str(result.get("rail_width_m") if result else None))
                rate.sleep()
                continue
            lateral = float(result["lateral_error_m"])
            heading = float(result["heading_error_rad"])
            if samples and (
                    abs(lateral - samples[-1][0]) >
                    self.src2_frame_lateral_coherence or
                    abs(heading - samples[-1][1]) >
                    self.src2_frame_heading_coherence):
                samples = []
            samples.append((lateral, heading, float(result["rail_width_m"]),
                            float(result.get("confidence", 0.0))))
            self.publish_state(
                "SRC2_PARKING_FRAME_CONFIRMING",
                frames=len(samples), required=self.src2_frame_required,
                lateral_error_m=lateral,
                heading_error_deg=math.degrees(heading),
                rail_width_m=float(result["rail_width_m"]),
                confidence=float(result.get("confidence", 0.0)))
            if len(samples) >= self.src2_frame_required:
                window = samples[-self.src2_frame_required:]
                locked = {
                    "lateral_error_m": statistics.median(
                        value[0] for value in window),
                    "heading_error_rad": statistics.median(
                        value[1] for value in window),
                    "rail_width_m": statistics.median(
                        value[2] for value in window),
                    "confidence": statistics.median(
                        value[3] for value in window),
                    "frames": len(window),
                }
                self.publish_state(
                    "SRC2_PARKING_FRAME_LOCKED",
                    lateral_error_m=locked["lateral_error_m"],
                    heading_error_deg=math.degrees(
                        locked["heading_error_rad"]),
                    rail_width_m=locked["rail_width_m"],
                    confidence=locked["confidence"],
                    frames=locked["frames"])
                return locked
            rate.sleep()
        self.publish_state(
            "SRC2_PARKING_FRAME_NOT_LOCKED",
            timeout_s=self.src2_frame_timeout,
            policy=("reacquire" if self.src2_frame_required_for_success else
                    "ocr_projection_fallback"))
        return None

    @staticmethod
    def _zero_twist():
        from geometry_msgs.msg import Twist
        return Twist()

    def _src2_apply_floor_frame_center(self, frame):
        pose = self.current_pose()
        if (pose is None or self.src2_final_goal is None or
                self.src2_wall_point is None or
                self.src2_inward_normal is None):
            return False
        inward_x, inward_y = self.src2_inward_normal
        # This is the robot's left axis when it faces outward toward the wall,
        # and is also the tangent axis used by _src2_dock().
        dock_tangent = (inward_y, -inward_x)
        old_goal = self.src2_final_goal
        current_tangent = pose[0] * dock_tangent[0] + \
            pose[1] * dock_tangent[1]
        old_tangent = old_goal[0] * dock_tangent[0] + \
            old_goal[1] * dock_tangent[1]
        desired_tangent = (
            current_tangent + self.src2_frame_lateral_sign *
            float(frame["lateral_error_m"]))
        correction = desired_tangent - old_tangent
        if abs(correction) > self.src2_frame_correction_limit:
            self.publish_state(
                "SRC2_PARKING_FRAME_CORRECTION_REJECTED",
                requested_correction_m=correction,
                limit_m=self.src2_frame_correction_limit,
                policy="reacquire")
            return False
        dx = dock_tangent[0] * correction
        dy = dock_tangent[1] * correction
        self.src2_final_goal = (
            old_goal[0] + dx, old_goal[1] + dy, old_goal[2])
        self.src2_wall_point = (
            self.src2_wall_point[0] + dx,
            self.src2_wall_point[1] + dy)
        if self.src2_staging_goal is not None:
            self.src2_staging_goal = (
                self.src2_staging_goal[0] + dx,
                self.src2_staging_goal[1] + dy,
                self.src2_staging_goal[2])
        self.src2_frame_locked = True
        self.src2_frame_lock = dict(frame)
        self.publish_state(
            "SRC2_PARKING_CENTER_REPLACED_BY_FLOOR_FRAME",
            old_final_x=old_goal[0], old_final_y=old_goal[1],
            final_x=self.src2_final_goal[0],
            final_y=self.src2_final_goal[1],
            tangent_correction_m=correction,
            camera_lateral_error_m=frame["lateral_error_m"],
            rail_width_m=frame["rail_width_m"],
            ocr_recenter="disabled")
        return True

    def park(self):
        # At 0.55 m staging the complete 50 cm floor frame is visible.  Freeze
        # its centre once, then let src2's wall fit own yaw and forward range.
        self.src2_frame_locked = False
        self.src2_frame_lock = None
        frame = self._src2_acquire_floor_frame()
        if frame is None:
            if self.src2_frame_required_for_success:
                return "REACQUIRE"
        elif not self._src2_apply_floor_frame_center(frame):
            return "REACQUIRE"
        self.publish_state(
            "SRC2_CLOSE_OCR_RECENTER_DISABLED",
            reason="floor_frame_center_is_authoritative")
        if not self._src2_dock():
            return "REACQUIRE"
        if not self._src2_validate_parking():
            return "REACQUIRE"
        self.stop_robot(10)
        return "SUCCEEDED"

    def _src2_validate_parking(self):
        if self.src2_frame_required_for_success and not self.src2_frame_locked:
            self.publish_state(
                "SRC2_PARKING_VALIDATION_REJECTED_NO_FRAME_LOCK")
            return False
        valid = super(Src2FrameCenterRoomDeliveryV6,
                      self)._src2_validate_parking()
        if valid and self.src2_frame_lock is not None:
            self.publish_state(
                "SRC2_PARKING_FRAME_AND_FOOTPRINT_VALIDATED",
                frame_lateral_error_m=
                self.src2_frame_lock["lateral_error_m"],
                frame_heading_error_deg=math.degrees(
                    self.src2_frame_lock["heading_error_rad"]),
                rail_width_m=self.src2_frame_lock["rail_width_m"])
        return valid


if __name__ == "__main__":
    Src2FrameCenterRoomDeliveryV6()
    rospy.spin()
