#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String


class FollowTtsBridge:
    def __init__(self):
        self.begin_topic = rospy.get_param("~begin_topic", "/follow_begin")
        self.end_topic = rospy.get_param("~end_topic", "/follow_end")
        self.tts_topic = rospy.get_param("~tts_topic", "/factory/tts_text")
        self.traffic_decision_topic = rospy.get_param("~traffic_decision_topic", "")
        self.speak_begin_commands = self.get_bool_param("~speak_begin_commands", True)
        self.debounce_seconds = float(rospy.get_param("~debounce_seconds", 2.0))

        self.begin_text_map = {
            "Left": "巡线开始，左线",
            "Right": "巡线开始，右线",
            "Middle": "巡线开始，中线",
            "stop": "巡线已停止",
            "pause": "巡线已停止",
        }
        self.end_text_map = {
            "STOP": "巡线完成",
            "finished": "巡线完成",
            "finish": "巡线完成",
            "done": "巡线完成",
        }
        self.traffic_text_map = {
            "red": "红灯，请停止",
            "straight": "绿灯，请直行",
            "right": "绿灯，请右转",
            "left": "绿灯，请左转",
        }

        self.last_spoken = {}
        self.tts_pub = rospy.Publisher(self.tts_topic, String, queue_size=10)
        self.begin_sub = rospy.Subscriber(self.begin_topic, String, self.begin_callback)
        self.end_sub = rospy.Subscriber(self.end_topic, String, self.end_callback)
        self.traffic_decision_sub = None
        if self.traffic_decision_topic:
            self.traffic_decision_sub = rospy.Subscriber(
                self.traffic_decision_topic, String, self.traffic_decision_callback
            )

        rospy.loginfo(
            "follow_tts_bridge ready: begin=%s end=%s traffic_decision=%s tts=%s",
            self.begin_topic,
            self.end_topic,
            self.traffic_decision_topic or "disabled",
            self.tts_topic,
        )

    def begin_callback(self, msg):
        if not self.speak_begin_commands:
            return
        command = msg.data.strip()
        key = command.lower()
        text = self.begin_text_map.get(key)
        if text is None:
            text = "巡线开始"
        self.speak_once("begin:" + key, text)

    def end_callback(self, msg):
        status = msg.data.strip()
        key = status.lower()
        text = self.end_text_map.get(key)
        if text is None:
            text = "巡线完成"
        self.speak_once("end:" + key, text)

    def traffic_decision_callback(self, msg):
        decision = msg.data.strip().lower()
        text = self.traffic_text_map.get(decision)
        if text is None:
            rospy.logwarn("ignored unknown traffic decision: %s", msg.data)
            return
        self.speak_once("traffic:" + decision, text)

    def speak_once(self, event_key, text):
        now = rospy.Time.now()
        last_time = self.last_spoken.get(event_key)
        if last_time is not None and (now - last_time).to_sec() < self.debounce_seconds:
            return

        self.last_spoken[event_key] = now
        self.tts_pub.publish(String(data=text))
        rospy.loginfo("TTS published: %s", text)

    @staticmethod
    def get_bool_param(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)


if __name__ == "__main__":
    rospy.init_node("follow_tts_bridge")
    FollowTtsBridge()
    rospy.spin()
