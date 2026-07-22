#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Exact src2 QR step-scan motion, adapted only to the existing flow topics."""

import math
import time

import rospy
from geometry_msgs.msg import Twist

from xunfei2026_continuous_qr_hybrid_v1 import ContinuousQRHybrid


def normalize_angle(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


class DirectedYawAccumulator(object):
    """Verbatim behavior of src2's odometry yaw step accumulator."""

    def __init__(self, direction=1.0):
        self.direction = 1.0 if float(direction) >= 0.0 else -1.0
        self.start_yaw = None
        self.last_yaw = None
        self.progress = 0.0

    def reset(self, yaw):
        self.start_yaw = float(yaw)
        self.last_yaw = float(yaw)
        self.progress = 0.0

    def update(self, yaw):
        yaw = float(yaw)
        if self.last_yaw is None:
            self.reset(yaw)
            return self.progress
        self.last_yaw = yaw
        net_progress = normalize_angle(yaw - self.start_yaw) * self.direction
        if net_progress > self.progress:
            self.progress = net_progress
        return self.progress


class Src2ExactQRStage(ContinuousQRHybrid):
    def __init__(self):
        super(Src2ExactQRStage, self).__init__()
        self.src2_speed = abs(float(rospy.get_param(
            "~qr_scan_angular_speed", 0.20)))
        self.src2_step_angle = abs(float(rospy.get_param(
            "~qr_scan_step_angle_rad", math.radians(20.0))))
        self.src2_settle_s = max(0.0, float(rospy.get_param(
            "~qr_scan_settle_sec", 0.6)))
        self.src2_scan_timeout_s = max(5.0, float(rospy.get_param(
            "~qr_scan_timeout_sec", 60.0)))
        self.src2_step_margin_s = max(0.1, float(rospy.get_param(
            "~qr_scan_step_timeout_margin_sec", 2.0)))
        self.src2_odom_stale_s = max(0.1, float(rospy.get_param(
            "~qr_odom_stale_sec", 0.5)))
        self.src2_odom_wait_s = max(0.2, float(rospy.get_param(
            "~qr_odom_wait_sec", 2.0)))

    def fresh_yaw(self):
        with self.lock:
            yaw = self.yaw
            stamp = self.odom_stamp
        if (yaw is None or stamp == rospy.Time(0) or
                (rospy.Time.now() - stamp).to_sec() >
                self.src2_odom_stale_s):
            return None
        return yaw

    def wait_for_fresh_yaw(self):
        deadline = time.monotonic() + self.src2_odom_wait_s
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            yaw = self.fresh_yaw()
            if yaw is not None:
                return yaw
            rospy.sleep(0.05)
        return None

    def settle_for_qr(self, scan_deadline):
        self.publish_zero(3)
        deadline = min(
            scan_deadline, time.monotonic() + self.src2_settle_s)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.complete.is_set():
                return True
            if self.fresh_yaw() is None:
                self.publish_zero(3)
                return False
            rospy.sleep(0.05)
        return self.complete.is_set()

    def scan_qr_at_current_pose(self, label):
        """Port of src2 CompetitionFlow.scan_qr_at_current_pose()."""
        if self.src2_speed <= 0.0 or self.src2_step_angle <= 0.0:
            self.publish_status("FAILED_SRC2_SCAN_PARAMETERS")
            return False
        total_steps = int(math.ceil(
            (2.0 * math.pi) / self.src2_step_angle))
        scan_deadline = time.monotonic() + self.src2_scan_timeout_s
        with self.lock:
            self.scan_active = True
            # Keeps the inherited timer completely out of cmd_vel ownership.
            self.motion_mode = "SRC2_EXACT_STEP_SCAN"
        self.publish_status(
            "{}_START count={}/{} steps={}".format(
                label, len(self.results), self.target_count, total_steps))
        if self.wait_for_fresh_yaw() is None:
            self.publish_status("FAILED_SRC2_ODOMETRY_TIMEOUT")
            return False
        if self.settle_for_qr(scan_deadline):
            return True

        command = Twist()
        command.angular.z = self.src2_speed
        tracker = DirectedYawAccumulator(direction=1.0)
        for step_index in range(total_steps):
            if self.complete.is_set():
                self.publish_zero(3)
                return True
            if time.monotonic() >= scan_deadline:
                self.publish_zero(3)
                return False
            start_yaw = self.fresh_yaw()
            if start_yaw is None:
                self.publish_status("FAILED_SRC2_ODOMETRY_STALE")
                return False
            tracker.reset(start_yaw)
            step_deadline = (
                time.monotonic() + self.src2_step_angle / self.src2_speed +
                self.src2_step_margin_s)
            while tracker.progress < self.src2_step_angle:
                if rospy.is_shutdown() or self.complete.is_set():
                    self.publish_zero(3)
                    return self.complete.is_set()
                if (time.monotonic() >= scan_deadline or
                        time.monotonic() >= step_deadline):
                    self.publish_zero(3)
                    self.publish_status(
                        "FAILED_SRC2_STEP_TIMEOUT_{}".format(step_index + 1))
                    return False
                yaw = self.fresh_yaw()
                if yaw is None:
                    self.publish_zero(3)
                    self.publish_status("FAILED_SRC2_ODOMETRY_STALE")
                    return False
                if tracker.update(yaw) >= self.src2_step_angle:
                    break
                self.cmd_pub.publish(command)
                rospy.sleep(0.05)
            if self.settle_for_qr(scan_deadline):
                return True
        self.publish_zero(3)
        return self.complete.is_set()

    def run(self):
        self.publish_status("WAITING_FIRST_STAGE_NAVIGATION")
        while not rospy.is_shutdown():
            if self.nav_finished.wait(0.1):
                break
            if self.nav_failed.is_set():
                self.publish_status(
                    "FAILED_FIRST_STAGE_{}".format(self.nav_status))
                self.publish_zero()
                return 2
        if rospy.is_shutdown():
            return 1

        self.publish_status("NAVIGATION_SUCCEEDED_SETTLING")
        self.publish_zero()
        rospy.sleep(self.nav_settle_s)
        if not self.start_camera_if_needed():
            self.publish_status("FAILED_CAMERA_START")
            return 3
        self.enable_camera_low_exposure()
        if not self.wait_for_camera():
            self.publish_status("FAILED_CAMERA_TIMEOUT")
            return 3

        completed = self.scan_qr_at_current_pose("SRC2_PRIMARY_SCAN")
        with self.lock:
            self.scan_active = False
            self.motion_mode = "IDLE"
        if (not completed and self.fallback_enabled and
                not rospy.is_shutdown()):
            self.publish_status(
                "SRC2_PRIMARY_INCOMPLETE_{}/{}_REPOSITION".format(
                    len(self.results), self.target_count))
            if self.navigate_to_fallback_viewpoint():
                completed = self.scan_qr_at_current_pose(
                    "SRC2_FALLBACK_SCAN")

        with self.lock:
            self.scan_active = False
            self.motion_mode = "IDLE"
        self.publish_zero(12)
        self.publish_summary()
        if not completed:
            self.publish_status(
                "FAILED_SRC2_TWO_POSES_COUNT_{}/{}".format(
                    len(self.results), self.target_count))
            return 4
        self.publish_status("COMPLETE_3_UNIQUE_QR_STOPPED")
        rospy.sleep(2.0)
        return 0


def main():
    rospy.init_node("xunfei2026_persistent_qr_src2_exact")
    node = Src2ExactQRStage()
    try:
        result = node.run()
        rospy.logwarn("XUNFEI2026_SRC2_EXACT_QR result=%d", result)
        if result == 0 and not rospy.is_shutdown():
            rospy.spin()
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()
