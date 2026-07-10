#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS wrapper for the four-state YOLOv5s/RKNN traffic-light detector."""

import os
import logging
import sys
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

# catkin relay scripts and direct source execution both need to find this module.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rknn_backend import (  # noqa: E402
    CLASS_NAMES,
    TemporalVoter,
    YoloV5RknnBackend,
    select_primary_detection,
)


def restore_logging_level_names():
    """Undo RKNN/third-party logging aliases that break rosgraph logging."""
    logging.addLevelName(logging.DEBUG, "DEBUG")
    logging.addLevelName(logging.INFO, "INFO")
    logging.addLevelName(logging.WARNING, "WARNING")
    logging.addLevelName(logging.ERROR, "ERROR")
    logging.addLevelName(logging.CRITICAL, "CRITICAL")


class TrafficLightDetectorNode:
    COMMANDS = {
        "red": "Stop",
        "straight": "Middle",
        "left": "Left",
        "right": "Right",
    }
    COLORS = {
        "red": (0, 0, 255),
        "straight": (0, 255, 0),
        "left": (0, 255, 0),
        "right": (0, 255, 0),
    }

    def __init__(self):
        rospy.init_node("traffic_light_detector", anonymous=False)

        self.image_topic = rospy.get_param("~image_topic", "/ucar_camera/image_raw")
        self.enable_topic = rospy.get_param("~enable_topic", "/traffic_light/enable")
        self.state_topic = rospy.get_param("~state_topic", "/traffic_light/state")
        self.command_topic = rospy.get_param("~command_topic", "/follow_begin")
        self.decision_topic = rospy.get_param("~decision_topic")
        self.debug_topic = rospy.get_param("~debug_topic", "/traffic_light/debug_image")
        self.camera_profile_service = rospy.get_param(
            "~camera_profile_service", "/ucar_camera/set_exposure_profile"
        )
        self.require_camera_profile = self.get_bool_param("~require_camera_profile", True)
        self.camera_profile_timeout_seconds = float(
            rospy.get_param("~camera_profile_timeout_seconds", 2.0)
        )
        self.model_path = os.path.abspath(os.path.expanduser(rospy.get_param("~model_path")))

        self.publish_debug = self.get_bool_param("~publish_debug", True)
        self.publish_commands = self.get_bool_param("~publish_commands", True)
        self.publish_decisions = self.get_bool_param("~publish_decisions", True)
        self.armed = False
        self.command_sent = False
        self.camera_profile_active = False
        self.last_state = "unknown"
        self.bridge = CvBridge()

        class_names = tuple(rospy.get_param("~class_names", list(CLASS_NAMES)))
        anchors = rospy.get_param(
            "~anchors",
            [
                [10, 13], [16, 30], [33, 23],
                [30, 61], [62, 45], [59, 119],
                [116, 90], [156, 198], [373, 326],
            ],
        )
        self.voter = TemporalVoter(
            window_size=int(rospy.get_param("~vote_window", 7)),
            direction_min_votes=int(rospy.get_param("~direction_min_votes", 5)),
            direction_min_average_confidence=float(
                rospy.get_param("~direction_min_avg_confidence", 0.65)
            ),
            red_confirm_frames=int(rospy.get_param("~red_confirm_frames", 2)),
        )

        if not os.path.isfile(self.model_path):
            raise RuntimeError("RKNN model does not exist: {}".format(self.model_path))

        self.backend = YoloV5RknnBackend(
            model_path=self.model_path,
            input_size=int(rospy.get_param("~input_size", 640)),
            confidence_threshold=float(rospy.get_param("~confidence_threshold", 0.55)),
            nms_threshold=float(rospy.get_param("~nms_threshold", 0.45)),
            class_names=class_names,
            anchors=anchors,
            apply_sigmoid=self.get_bool_param("~apply_sigmoid", False),
            use_all_npu_cores=self.get_bool_param("~use_all_npu_cores", True),
        )
        restore_logging_level_names()

        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=1, latch=True)
        self.command_pub = rospy.Publisher(self.command_topic, String, queue_size=1)
        self.decision_pub = rospy.Publisher(self.decision_topic, String, queue_size=1)
        self.camera_profile_client = rospy.ServiceProxy(
            self.camera_profile_service, SetBool, persistent=False
        )
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.enable_sub = rospy.Subscriber(
            self.enable_topic, Bool, self.enable_callback, queue_size=1
        )
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        rospy.on_shutdown(self.shutdown)
        self.publish_state("unknown")

        if self.get_bool_param("~enabled_at_start", False):
            self.arm()

        rospy.loginfo(
            "traffic_light_detector ready: model=%s image=%s enable=%s state=%s command=%s decision=%s camera_profile=%s",
            self.model_path,
            self.image_topic,
            self.enable_topic,
            self.state_topic,
            self.command_topic,
            self.decision_topic,
            self.camera_profile_service,
        )

    def enable_callback(self, message):
        if message.data:
            if not self.armed:
                self.arm()
        elif self.armed:
            self.disarm(publish_unknown=True)

    def arm(self):
        self.voter.reset()
        self.armed = False
        self.command_sent = False
        self.publish_state("unknown")
        self.publish_command("Stop")
        if not self.set_camera_profile(True):
            rospy.logerr(
                "traffic-light detection remains disarmed because low-exposure camera profile failed"
            )
            return
        self.armed = True
        rospy.logwarn("traffic-light detection armed; vehicle held at Stop")

    def disarm(self, publish_unknown=False):
        self.armed = False
        self.command_sent = False
        self.voter.reset()
        self.set_camera_profile(False)
        if publish_unknown:
            self.publish_state("unknown")
        rospy.loginfo("traffic-light detection disarmed")

    def image_callback(self, message):
        if not self.armed:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logerr_throttle(1.0, "cv_bridge error: %s", exc)
            return

        start_time = time.monotonic()
        try:
            detections = self.backend.infer(frame)
        except Exception as exc:
            rospy.logfatal("RKNN inference failed: %s", exc)
            self.publish_command("Stop")
            self.set_camera_profile(False)
            rospy.signal_shutdown("RKNN inference failure")
            return

        primary = select_primary_detection(detections)
        raw_state = primary.state if primary is not None else "unknown"
        confidence = primary.confidence if primary is not None else 0.0
        stable_state = self.voter.update(raw_state, confidence)
        previous_state = self.last_state
        self.publish_state(stable_state)

        if stable_state == "red":
            # Stop was already sent when armed; repeat only on the red transition.
            if previous_state != "red":
                self.publish_command("Stop")
                self.publish_decision("red")
                self.set_camera_profile(False)
        elif stable_state in ("straight", "left", "right") and not self.command_sent:
            self.publish_command(self.COMMANDS[stable_state])
            self.publish_decision(stable_state)
            self.set_camera_profile(False)
            self.command_sent = True
            self.armed = False
            self.voter.reset()
            rospy.logwarn(
                "traffic light confirmed: state=%s command=%s; detector auto-disarmed",
                stable_state,
                self.COMMANDS[stable_state],
            )

        elapsed_ms = (time.monotonic() - start_time) * 1000.0
        if self.publish_debug:
            self.publish_debug_image(
                message, frame, detections, raw_state, stable_state, elapsed_ms
            )
        rospy.loginfo_throttle(
            1.0,
            "traffic raw=%s confidence=%.3f stable=%s detections=%d latency=%.1fms",
            raw_state,
            confidence,
            stable_state,
            len(detections),
            elapsed_ms,
        )
        self.last_state = stable_state

    def publish_state(self, state):
        self.last_state = state
        self.state_pub.publish(String(data=state))

    def publish_command(self, command):
        if not self.publish_commands:
            rospy.logwarn("command publishing disabled; would publish %s", command)
            return
        self.command_pub.publish(String(data=command))
        rospy.logwarn("published traffic command %s to %s", command, self.command_topic)

    def publish_decision(self, decision):
        """Publish one confirmed traffic-light decision for presentation layers."""
        if not self.publish_decisions:
            rospy.loginfo("decision publishing disabled; would publish %s", decision)
            return
        self.decision_pub.publish(String(data=decision))
        rospy.loginfo("published traffic decision %s to %s", decision, self.decision_topic)

    def set_camera_profile(self, enable_low_exposure):
        """Switch camera exposure through its parameterized SetBool service."""
        if not self.require_camera_profile:
            return True
        try:
            rospy.wait_for_service(
                self.camera_profile_service, timeout=self.camera_profile_timeout_seconds
            )
            response = self.camera_profile_client(bool(enable_low_exposure))
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logerr(
                "camera exposure service %s unavailable: %s", self.camera_profile_service, exc
            )
            return False

        if not response.success:
            rospy.logerr("camera exposure service rejected request: %s", response.message)
            return False
        self.camera_profile_active = bool(enable_low_exposure)
        rospy.loginfo(
            "camera exposure profile %s via %s",
            "enabled" if enable_low_exposure else "restored",
            self.camera_profile_service,
        )
        return True

    def publish_debug_image(
        self, source_message, frame, detections, raw_state, stable_state, elapsed_ms
    ):
        debug = frame.copy()
        for detection in detections:
            x1, y1, x2, y2 = (int(round(value)) for value in detection.box)
            color = self.COLORS.get(detection.state, (0, 255, 255))
            cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
            label = "{} {:.2f}".format(detection.state, detection.confidence)
            cv2.putText(
                debug,
                label,
                (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )
        status = "raw={} stable={} {:.1f}ms".format(raw_state, stable_state, elapsed_ms)
        cv2.putText(
            debug,
            status,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 255),
            2,
        )
        try:
            debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            debug_message.header = source_message.header
            self.debug_pub.publish(debug_message)
        except Exception as exc:
            rospy.logwarn_throttle(1.0, "debug image publish failed: %s", exc)

    def shutdown(self):
        if getattr(self, "camera_profile_active", False):
            self.set_camera_profile(False)
        if getattr(self, "backend", None) is not None:
            self.backend.release()
            self.backend = None

    @staticmethod
    def get_bool_param(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)


if __name__ == "__main__":
    try:
        TrafficLightDetectorNode()
        rospy.spin()
    except (rospy.ROSInterruptException, RuntimeError, ValueError) as exc:
        restore_logging_level_names()
        rospy.logfatal("traffic_light_detector startup failed: %s", exc)
