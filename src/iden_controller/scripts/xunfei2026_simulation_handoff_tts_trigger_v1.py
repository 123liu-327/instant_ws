#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Start simulation immediately when the real completion voice is published.

The existing handoff owns the UDP protocol, completion announcement and
post-simulation route.  This adapter changes only its start gate: the matching
``已将...放入...`` TTS message directly starts simulation, independently of
the parking result.  Earlier wake/order/QR announcements cannot start it.
"""

import json
import threading

import rospy
from std_msgs.msg import String

from xunfei2026_simulation_handoff_v1 import SimulationHandoff


class TtsTriggeredSimulationHandoff(SimulationHandoff):
    def __init__(self):
        # These fields must exist before the parent registers callbacks.
        self.real_completion_voice_seen = False
        self.real_selected_item = ""
        self.real_target_warehouse = ""
        self.last_completion_voice = ""
        super(TtsTriggeredSimulationHandoff, self).__init__()
        rospy.Subscriber(
            self.tts_topic, String, self.tts_callback, queue_size=10)
        self.publish_state("WAITING_FINAL_VOICE_TRIGGER",
                           trigger_topic=self.tts_topic,
                           parking_result_required=False)

    def result_callback(self, msg):
        super(TtsTriggeredSimulationHandoff, self).result_callback(msg)
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if str(payload.get("status", "")).lower() != "success":
            return
        with self.lock:
            self.real_selected_item = str(
                payload.get("selected_item", "")).strip()
            self.real_target_warehouse = str(
                payload.get("target_warehouse", "")).strip()
        self.try_start_voice_gated_handoff()

    def completion_voice_matches(self, text):
        normalized = "".join(str(text).split()).rstrip("。！!")
        if not (normalized.startswith("已将") and "放入" in normalized):
            return False
        with self.lock:
            item = self.real_selected_item
            warehouse = self.real_target_warehouse
        return bool(item and warehouse and item in normalized and
                    warehouse in normalized)

    def tts_callback(self, msg):
        text = str(msg.data).strip()
        if not self.completion_voice_matches(text):
            return
        with self.lock:
            if self.real_completion_voice_seen:
                return
            self.real_completion_voice_seen = True
            self.last_completion_voice = text
        self.publish_state(
            "REAL_COMPLETION_VOICE_TRIGGERED", announcement=text)
        self.try_start_voice_gated_handoff()

    def real_status_callback(self, _msg):
        # Deliberately ignore parking status.  The matching final TTS message
        # is the sole mission trigger requested for this workflow.
        return

    def try_start_voice_gated_handoff(self):
        with self.lock:
            if self.handoff_started:
                return
            required_order = (
                self.sim_target_key, self.sim_selected_item,
                self.sim_target_category, self.sim_target_warehouse,
                self.sim_mission_id)
            if not self.real_completion_voice_seen:
                return
            if not all(required_order):
                self.publish_state(
                    "SIMULATION_TRIGGER_WAITING_ORDER",
                    reason="simulation order is incomplete")
                return
            self.handoff_started = True
            announcement = self.last_completion_voice
        self.publish_state(
            "SIMULATION_TRIGGERED_BY_FINAL_VOICE",
            announcement=announcement, parking_result_required=False)
        worker = threading.Thread(target=self.start_simulation_handoff)
        worker.daemon = True
        worker.start()


if __name__ == "__main__":
    TtsTriggeredSimulationHandoff()
    rospy.spin()
