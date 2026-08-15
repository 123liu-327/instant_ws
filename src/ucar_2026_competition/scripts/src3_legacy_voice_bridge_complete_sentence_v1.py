#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Adapt cloud_asr_test2 while withholding incomplete dual-category orders."""

import re
import threading
import time

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse


def normalize_text(value):
    text = str(value or "").strip()
    return re.sub(r"[\s，。！？、,.!?;；:：]+", "", text)


def category_from_text(text):
    compact = normalize_text(text).lower()
    if "日用品" in compact or "daily" in compact:
        return "daily"
    if "电子产品" in compact or "electronics" in compact or "electronic" in compact:
        return "electronics"
    if "食品" in compact or "food" in compact:
        return "food"
    return None


def dual_categories(text):
    compact = normalize_text(text).lower()
    marker_position = -1
    for marker in ("仿真环境", "仿真", "模拟环境", "虚拟环境"):
        position = compact.find(marker)
        if position >= 0 and (marker_position < 0 or position < marker_position):
            marker_position = position
    if marker_position < 0:
        return None, None
    physical = category_from_text(compact[:marker_position])
    simulation = category_from_text(compact[marker_position:])
    if physical and simulation and physical != simulation:
        return physical, simulation
    return None, None


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
        self.command_fragments = []
        self.fragment_started_at = 0.0
        self.fragment_last_at = 0.0
        self.fragment_reset_s = max(
            2.0, float(rospy.get_param("~fragment_reset_s", 8.0))
        )
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
        rospy.Service(
            "/speech_command_node/start_listening", Trigger, self.start_listening
        )
        rospy.Service(
            "/speech_command_node/stop_listening", Trigger, self.stop_listening
        )
        rospy.logwarn(
            "SRC3_COMPLETE_SENTENCE_VOICE_READY asr=/factory/voice_raw_text "
            "requires=physical+simulation"
        )

    def is_wake_word(self, text):
        return any(word and word in text for word in self.wake_words)

    def split_wake_command(self, text):
        matches = []
        for word in self.wake_words:
            position = text.find(word)
            if position >= 0:
                matches.append((position, -len(word), word))
        if not matches:
            return False, text
        position, _negative_length, word = min(matches)
        return True, text[position + len(word):]

    def merge_command_fragment(self, raw, text):
        if not self.command_fragments:
            self.command_fragments = [text]
            return text

        combined = "".join(self.command_fragments)
        compact_combined = normalize_text(combined)
        if text == compact_combined or text in compact_combined:
            return compact_combined
        if compact_combined and text.startswith(compact_combined):
            self.command_fragments = [text]
            return text

        overlap = 0
        max_overlap = min(len(compact_combined), len(text))
        for size in range(max_overlap, 0, -1):
            if compact_combined.endswith(text[:size]):
                overlap = size
                break
        merged = compact_combined + text[overlap:]
        self.command_fragments = [merged]
        return merged

    def asr_callback(self, msg):
        raw = str(msg.data or "").strip()
        text = normalize_text(raw)
        if not text:
            return

        with self.lock:
            if self.accepted:
                return
            has_wake, command_text = self.split_wake_command(text)
            if has_wake and not self.listening:
                now = time.monotonic()
                if self.wake_pending and now - self.last_wake_at < self.wake_debounce_s:
                    rospy.loginfo("SRC3_VOICE_WAKE_IGNORED pending handshake")
                    return
                self.wake_pending = True
                self.last_wake_at = now
                if command_text:
                    self.command_fragments = [command_text]
                    self.fragment_started_at = now
                    self.fragment_last_at = now
                    rospy.logwarn(
                        "SRC3_INLINE_COMMAND_BUFFERED text=%s", command_text)
                rospy.logwarn("SRC3_VOICE_WAKE_ACCEPTED text=%s", raw)
                self.wakeup_pub.publish(String(data=raw))
                return
            if has_wake:
                if not command_text:
                    rospy.loginfo("SRC3_VOICE_WAKE_IGNORED during command window")
                    return
                text = command_text
                raw = command_text
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

            if (self.command_fragments and
                    now - self.fragment_last_at > self.fragment_reset_s):
                rospy.logwarn("SRC3_VOICE_FRAGMENT_WINDOW_RESET after long silence")
                self.command_fragments = []
                self.fragment_started_at = 0.0

            if not self.command_fragments:
                self.fragment_started_at = now
            self.fragment_last_at = now

            physical, simulation = dual_categories(raw)
            if physical and simulation:
                self.command_fragments = [raw]
                complete_question = raw
            else:
                complete_question = self.merge_command_fragment(raw, text)
                physical, simulation = dual_categories(complete_question)

            if not physical or not simulation:
                rospy.logwarn(
                    "SRC3_VOICE_FRAGMENT_RETAINED elapsed=%.1fs text=%s",
                    now - self.fragment_started_at,
                    complete_question,
                )
                return
            self.pending_question = True

        rospy.logwarn(
            "SRC3_VOICE_COMPLETE_ORDER physical=%s simulation=%s text=%s",
            physical,
            simulation,
            complete_question,
        )
        self.question_pub.publish(String(data=complete_question))

    def start_listening(self, _request):
        complete = None
        with self.lock:
            if self.accepted:
                return TriggerResponse(False, "voice command already accepted")
            buffered = "".join(self.command_fragments)
            self.wake_pending = False
            self.listening = True
            self.pending_question = False
            self.last_question = ""
            self.last_question_at = 0.0
            if buffered:
                physical, simulation = dual_categories(buffered)
                if physical and simulation:
                    self.pending_question = True
                    complete = (buffered, physical, simulation)
            else:
                self.command_fragments = []
                self.fragment_started_at = 0.0
                self.fragment_last_at = 0.0
        if complete is not None:
            question, physical, simulation = complete
            rospy.logwarn(
                "SRC3_INLINE_COMPLETE_ORDER physical=%s simulation=%s text=%s",
                physical,
                simulation,
                question,
            )
            self.question_pub.publish(String(data=question))
        else:
            rospy.logwarn(
                "SRC3_COMPLETE_SENTENCE_COMMAND_WINDOW_OPEN buffered=%s",
                buffered or "none",
            )
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
