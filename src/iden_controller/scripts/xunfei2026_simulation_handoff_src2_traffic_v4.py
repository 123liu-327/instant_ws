#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Second-parking simulation handoff with native src2 traffic gating."""

import json
import subprocess
import threading
import time

import rospy
from std_msgs.msg import String

from xunfei2026_simulation_handoff_second_parking_v3 import (
    SecondParkingSimulationHandoff,
)


TRAFFIC_TO_FOLLOW = {
    "green_left": "left",
    "green_right": "right",
    "green_straight": "middle",
}


class Src2TrafficSimulationHandoff(SecondParkingSimulationHandoff):
    """Hold at the line on red and start the selected follower on green."""

    def __init__(self):
        self.traffic_process = None
        self.traffic_event = threading.Event()
        self.traffic_class = ""
        super(Src2TrafficSimulationHandoff, self).__init__()
        self.traffic_launch_file = rospy.get_param(
            "~src2_traffic_launch_file",
            "xunfei2026_src2_traffic_light_stage_v1.launch")
        self.traffic_topic = rospy.get_param(
            "~src2_traffic_topic", "/traffic_light_rknn_test/detections")
        self.traffic_timeout = max(5.0, float(rospy.get_param(
            "~src2_traffic_timeout_s", 180.0)))
        rospy.Subscriber(
            self.traffic_topic, String, self.traffic_callback, queue_size=20)

    def traffic_callback(self, message):
        try:
            payload = json.loads(message.data)
            consensus = payload.get("consensus", {})
            if not bool(consensus.get("active", False)):
                return
            class_name = str(consensus.get("class_name", "")).strip()
        except Exception:
            return
        if class_name == "red_light":
            self.stop_robot(6)
            self.publish_state("SRC2_TRAFFIC_RED_HOLD")
            return
        if class_name not in TRAFFIC_TO_FOLLOW:
            return
        with self.lock:
            self.traffic_class = class_name
        self.traffic_event.set()

    def wait_for_src2_traffic(self):
        """Run the native src2 classifier only during the traffic stage."""
        self.stop_robot(12)
        self.traffic_event.clear()
        with self.lock:
            self.traffic_class = ""
        self.publish_state("SRC2_TRAFFIC_WAITING")
        self.traffic_process = subprocess.Popen([
            "roslaunch", "iden_controller", self.traffic_launch_file])
        deadline = time.monotonic() + self.traffic_timeout
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                if self.traffic_event.wait(0.10):
                    with self.lock:
                        class_name = self.traffic_class
                    mode = TRAFFIC_TO_FOLLOW.get(class_name)
                    if mode:
                        self.follow_mode = mode
                        self.publish_state(
                            "SRC2_TRAFFIC_GREEN_CONFIRMED",
                            traffic_class=class_name, follow_mode=mode)
                        return
                if (self.traffic_process is not None and
                        self.traffic_process.poll() is not None):
                    raise RuntimeError("src2 traffic detector exited")
            raise RuntimeError("src2 traffic recognition timed out")
        finally:
            self.stop_process(self.traffic_process)
            self.traffic_process = None
            self.stop_robot(12)

    def post_simulation_route(self):
        try:
            self.navigate_to_stop_line_observation()
            self.run_stop_line_parking()
            if self.follow_after_stop_line:
                self.wait_for_src2_traffic()
                self.advance_before_line_follow()
                self.start_line_following()
        except Exception as exc:
            self.stop_robot(20)
            self.publish_state("POST_SIM_STOP_LINE_FAILED", reason=str(exc))

    def shutdown(self):
        self.stop_process(self.traffic_process)
        self.traffic_process = None
        super(Src2TrafficSimulationHandoff, self).shutdown()


if __name__ == "__main__":
    Src2TrafficSimulationHandoff()
    rospy.spin()
