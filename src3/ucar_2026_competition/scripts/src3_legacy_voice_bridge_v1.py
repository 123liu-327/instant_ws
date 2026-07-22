#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Adapt the existing cloud_asr_test2 topics to the src3 voice contract."""

import re
import threading
import time

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


def normalize_text(value):
    text = str(value or "").strip()
    return re.sub(r"[\s，。！？、,.!?;；:：]+", "", text)


class LegacyVoiceBridge:
    def __init__(self):
        self.lock = threading.RLock()
        self.listening = False
        self.accepted = False
        self.pending_question = False
        self.wake_pending = False
        self.last_wake_at = 0.0
        self.last_question = ""
        self.last_question_at = 0.0
        self.wake_debounce_s = float(rospy.get_param("~wake_debounce_s", 8.0))
        self.question_debounce_s = float(
            rospy.get_param("~question_debounce_s", 1.0)
        )
        self.wake_words = tuple(
            normalize_text(x)
            for x in rospy.get_param("~wake_words", ["小飞小飞"])
            if normalize_text(x)
        )
        self.echo_phrases = {
            normalize_text(x)
            for x in rospy.get_param(
                "~echo_phrases",
                ["我在", "我在请说明需要领取的货品大类", "好的", "请再说一次"],
            )
        }

        self.wakeup_pub = rospy.Publisher("/wakeup", String, queue_size=5)
        self.question_pub = rospy.Publisher("/question", String, queue_size=10)
        self.tts_pub = rospy.Publisher("/factory/tts_text", String, queue_size=10)
        self.accepted_pub = rospy.Publisher(
            "/factory/voice_command_accepted", Bool, queue_size=1, latch=True
        )

        rospy.Subscriber(
            "/factory/voice_raw_text", String, self.asr_callback, queue_size=20
        )
        rospy.Subscriber("/speak", String, self.speak_callback, queue_size=20)
        rospy.Service(
            "/speech_command_node/start_listening", Trigger, self.start_listening
        )
        rospy.Service(
            "/speech_command_node/stop_listening", Trigger, self.stop_listening
        )
        rospy.logwarn(
            "SRC3_LEGACY_VOICE_BRIDGE_READY asr=/factory/voice_raw_text "
            "tts=/factory/tts_text"
        )

    def is_wake_word(self, text):
        return any(word and word in text for word in self.wake_words)

    def asr_callback(self, msg):
        raw = str(msg.data or "").strip()
        text = normalize_text(raw)
        if not text:
            return

        with self.lock:
            if self.accepted:
                return
            if self.is_wake_word(text):
                if self.listening:
                    rospy.loginfo("SRC3_VOICE_WAKE_IGNORED during command window")
                    return
                now = time.monotonic()
                if self.wake_pending and now - self.last_wake_at < self.wake_debounce_s:
                    rospy.loginfo("SRC3_VOICE_WAKE_IGNORED pending handshake")
                    return
                self.wake_pending = True
                self.last_wake_at = now
                rospy.logwarn("SRC3_VOICE_WAKE_ACCEPTED text=%s", raw)
                self.wakeup_pub.publish(String(data=raw))
                return
            if not self.listening:
                rospy.loginfo("SRC3_VOICE_IGNORED outside command window: %s", raw)
                return
            if text in self.echo_phrases:
                rospy.loginfo("SRC3_VOICE_ECHO_IGNORED text=%s", raw)
                return

            now = time.monotonic()
            if (
                text == self.last_question
                and now - self.last_question_at < self.question_debounce_s
            ):
                return
            self.last_question = text
            self.last_question_at = now
            self.pending_question = True

        rospy.logwarn("SRC3_VOICE_QUESTION_FORWARDED text=%s", raw)
        self.question_pub.publish(String(data=raw))

    def speak_callback(self, msg):
        text = str(msg.data or "").strip()
        if text:
            rospy.loginfo("SRC3_TTS_TO_LEGACY text=%s", text)
            self.tts_pub.publish(String(data=text))

    def start_listening(self, _request):
        with self.lock:
            if self.accepted:
                return TriggerResponse(False, "voice command already accepted")
            self.wake_pending = False
            self.listening = True
            self.pending_question = False
            self.last_question = ""
            self.last_question_at = 0.0
        rospy.logwarn("SRC3_LEGACY_ASR_COMMAND_WINDOW_OPEN")
        return TriggerResponse(True, "legacy ASR command window opened")

    def stop_listening(self, _request):
        with self.lock:
            had_question = self.pending_question
            self.wake_pending = False
            self.listening = False
            self.pending_question = False
            if had_question:
                self.accepted = True
        if had_question:
            self.accepted_pub.publish(Bool(data=True))
            rospy.logwarn("SRC3_LEGACY_ASR_COMMAND_ACCEPTED")
        else:
            rospy.loginfo("SRC3_LEGACY_ASR_COMMAND_WINDOW_CLOSED without acceptance")
        return TriggerResponse(True, "legacy ASR command window closed")


def main():
    rospy.init_node("src3_legacy_voice_bridge")
    LegacyVoiceBridge()
    rospy.spin()


if __name__ == "__main__":
    main()
