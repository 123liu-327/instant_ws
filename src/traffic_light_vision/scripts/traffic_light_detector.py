#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS wrapper for the four-state YOLOv5s/RKNN traffic-light detector."""

import os
import sys
import time

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

# catkin relay scripts and direct source execution both need to find this module.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rknn_backend import (  # noqa: E402
    CLASS_NAMES,
    TemporalVoter,
    YoloV5RknnBackend,
    select_primary_detection,
)


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
        self.debug_topic = rospy.get_param("~debug_topic", "/traffic_light/debug_image")
        self.model_path = os.path.abspath(os.path.expanduser(rospy.get_param("~model_path")))

        self.publish_debug = self.get_bool_param("~publish_debug", True)
        self.publish_commands = self.get_bool_param("~publish_commands", True)
        self.armed = False
        self.command_sent = False
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

        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=1, latch=True)
        self.command_pub = rospy.Publisher(self.command_topic, String, queue_size=1)
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
            "traffic_light_detector ready: model=%s image=%s enable=%s state=%s command=%s",
            self.model_path,
            self.image_topic,
            self.enable_topic,
            self.state_topic,
            self.command_topic,
        )

    def enable_callback(self, message):
        if message.data:
            if not self.armed:
                self.arm()
        elif self.armed:
            self.disarm(publish_unknown=True)

    def arm(self):
        self.voter.reset()
        self.armed = True
        self.command_sent = False
        self.publish_state("unknown")
        self.publish_command("Stop")
        rospy.logwarn("traffic-light detection armed; vehicle held at Stop")

    def disarm(self, publish_unknown=False):
        self.armed = False
        self.command_sent = False
        self.voter.reset()
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
        elif stable_state in ("straight", "left", "right") and not self.command_sent:
            self.publish_command(self.COMMANDS[stable_state])
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
        rospy.logfatal("traffic_light_detector startup failed: %s", exc)
