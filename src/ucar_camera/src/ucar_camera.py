#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""USB camera publisher with a reversible low-exposure V4L2 profile."""

import re
import shlex
import subprocess
import threading

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_srvs.srv import SetBool, SetBoolResponse


class UcarCamera:
    """Publish camera images and expose a low-exposure profile service."""

    CONTROL_NAMES = (
        "exposure_auto",
        "exposure_absolute",
        "exposure_auto_priority",
    )

    def __init__(self):
        # The fixed name keeps the private service path stable:
        # /ucar_camera/set_exposure_profile.
        rospy.init_node("ucar_camera", anonymous=False)

        self.img_width = int(rospy.get_param("~image_width", 1280))
        self.img_height = int(rospy.get_param("~image_height", 720))
        self.camera_topic_name = rospy.get_param("~cam_topic_name", "/ucar_camera/image_raw")
        self.device_path = rospy.get_param("~device_path", "/dev/video0")
        self.v4l2_ctl_command = rospy.get_param("~v4l2_ctl_command", "v4l2-ctl")
        self.low_exposure_absolute = int(rospy.get_param("~low_exposure_absolute", 150))
        self.manual_exposure_auto = int(rospy.get_param("~manual_exposure_auto", 1))
        self.low_exposure_auto_priority = int(
            rospy.get_param("~low_exposure_auto_priority", 0)
        )
        self.cam_pub_rate = int(rospy.get_param("~rate", 15))

        self.control_lock = threading.RLock()
        self.original_controls = None
        self.low_exposure_active = False
        self.closed = False

        self.cam_pub = rospy.Publisher(self.camera_topic_name, Image, queue_size=1)
        self.exposure_service = rospy.Service(
            "~set_exposure_profile", SetBool, self.set_exposure_profile_callback
        )

        self.cap = cv2.VideoCapture(self.device_path)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_height)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("I", "4", "2", "0"))
        if not self.cap.isOpened():
            raise RuntimeError("failed to open camera device {}".format(self.device_path))

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "ucar_camera ready: device=%s image=%s exposure_service=%s low_exposure=%d",
            self.device_path,
            self.camera_topic_name,
            rospy.resolve_name("~set_exposure_profile"),
            self.low_exposure_absolute,
        )
        self.publish_loop()

    def publish_loop(self):
        rate = rospy.Rate(self.cam_pub_rate)
        while not rospy.is_shutdown():
            with self.control_lock:
                ret, frame = self.cap.read()
            if not ret or frame is None:
                rospy.logwarn_throttle(1.0, "camera read failed: %s", self.device_path)
                rate.sleep()
                continue

            frame = cv2.flip(frame, 1)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            message = Image()
            message.header = Header(stamp=rospy.Time.now(), frame_id="opencv")
            message.height, message.width = rgb_frame.shape[:2]
            message.encoding = "rgb8"
            message.is_bigendian = False
            message.step = message.width * 3
            message.data = np.ascontiguousarray(rgb_frame).tobytes()
            self.cam_pub.publish(message)
            rate.sleep()

    def set_exposure_profile_callback(self, request):
        with self.control_lock:
            try:
                if request.data:
                    self.enable_low_exposure()
                    return SetBoolResponse(True, "low exposure enabled")
                self.restore_exposure()
                return SetBoolResponse(True, "original exposure restored")
            except RuntimeError as exc:
                rospy.logerr("exposure profile change failed: %s", exc)
                return SetBoolResponse(False, str(exc))

    def enable_low_exposure(self):
        if self.low_exposure_active:
            return
        snapshot = self.get_control_values()
        try:
            self.set_control("exposure_auto", self.manual_exposure_auto)
            self.set_control("exposure_absolute", self.low_exposure_absolute)
            self.set_control("exposure_auto_priority", self.low_exposure_auto_priority)
            self.verify_control_values(
                {
                    "exposure_auto": self.manual_exposure_auto,
                    "exposure_absolute": self.low_exposure_absolute,
                    "exposure_auto_priority": self.low_exposure_auto_priority,
                }
            )
        except RuntimeError:
            self.restore_control_values(snapshot, suppress_errors=True)
            raise

        self.original_controls = snapshot
        self.low_exposure_active = True
        rospy.loginfo("camera low-exposure profile enabled: %s", self.low_exposure_absolute)

    def restore_exposure(self):
        if not self.low_exposure_active:
            return
        if self.original_controls is None:
            raise RuntimeError("missing original exposure profile")

        self.restore_control_values(self.original_controls, suppress_errors=False)
        self.low_exposure_active = False
        rospy.loginfo("camera original exposure profile restored: %s", self.original_controls)

    def restore_control_values(self, values, suppress_errors):
        try:
            # exposure_absolute is writable only in manual mode for this UVC camera.
            self.set_control("exposure_auto", self.manual_exposure_auto)
            self.set_control("exposure_absolute", values["exposure_absolute"])
            self.set_control("exposure_auto_priority", values["exposure_auto_priority"])
            self.set_control("exposure_auto", values["exposure_auto"])
            self.verify_control_values(
                {
                    "exposure_auto": values["exposure_auto"],
                    "exposure_auto_priority": values["exposure_auto_priority"],
                }
            )
        except RuntimeError:
            if suppress_errors:
                rospy.logerr("failed to roll back partial exposure change", exc_info=True)
                return
            raise

    def get_control_values(self):
        output = self.run_v4l2("--get-ctrl=" + ",".join(self.CONTROL_NAMES))
        values = {}
        for line in output.splitlines():
            match = re.match(r"\s*([a-z_]+).*?\bvalue=(-?\d+)", line)
            if match and match.group(1) in self.CONTROL_NAMES:
                values[match.group(1)] = int(match.group(2))
        missing = [name for name in self.CONTROL_NAMES if name not in values]
        if missing:
            raise RuntimeError("v4l2 controls unavailable: {}".format(", ".join(missing)))
        return values

    def verify_control_values(self, expected):
        actual = self.get_control_values()
        mismatches = [
            "{}={} (expected {})".format(name, actual[name], value)
            for name, value in expected.items()
            if actual[name] != value
        ]
        if mismatches:
            raise RuntimeError("v4l2 control verification failed: {}".format(", ".join(mismatches)))

    def set_control(self, name, value):
        self.run_v4l2("--set-ctrl={}".format("{}={}".format(name, int(value))))

    def run_v4l2(self, argument):
        try:
            command = shlex.split(self.v4l2_ctl_command) + ["-d", self.device_path, argument]
            completed = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                check=False,
                timeout=3.0,
            )
        except OSError as exc:
            raise RuntimeError("cannot execute {}: {}".format(self.v4l2_ctl_command, exc))
        except subprocess.TimeoutExpired:
            raise RuntimeError("v4l2 control command timed out")

        if completed.returncode != 0:
            detail = completed.stderr.strip() or completed.stdout.strip()
            raise RuntimeError("v4l2 control command failed: {}".format(detail))
        return completed.stdout

    def shutdown(self):
        with self.control_lock:
            if self.closed:
                return
            self.closed = True
            try:
                self.restore_exposure()
            except RuntimeError as exc:
                rospy.logerr("failed to restore camera exposure during shutdown: %s", exc)
            if getattr(self, "cap", None) is not None:
                self.cap.release()
                self.cap = None


if __name__ == "__main__":
    try:
        UcarCamera()
    except rospy.ROSInterruptException:
        pass
