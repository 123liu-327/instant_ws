#!/usr/bin/env python3
"""Two-target voice/QR fusion for the real-to-simulation collaboration task."""

import json
import re
import threading
import time

import rospy
from std_msgs.msg import String

from subtask1_real_order_fusion_route import (
    STATE_NAV_AND_SCAN,
    STATE_REASONING,
    STATE_WAITING_VOICE,
    Subtask1RealOrderFusion,
    WAREHOUSE_BY_CATEGORY,
    normalize_text,
)


CATEGORY_SPEECH = {
    "食品加工类": "食品大类",
    "日用品类": "日用品大类",
    "电子产品类": "电子产品大类",
}

SIM_TARGET_KEY = {
    "食品加工类": "food",
    "日用品类": "daily",
    "电子产品类": "electronics",
}


class Xunfei2026DualOrderFusion(Subtask1RealOrderFusion):
    """Accept one real target and one simulation target in the second utterance."""

    def __init__(self):
        self.sim_target_category = ""
        # cloud_asr_test2 closes its AIUI agent after one utterance.  The base
        # rearm thread waits for "我在" to finish, restarts that node, and only
        # then opens this event for the second utterance.
        self.command_session_ready = threading.Event()
        super().__init__()
        rospy.loginfo("dual real/simulation order fusion ready")

    def rearm_asr_thread(self):
        super().rearm_asr_thread()
        if not rospy.is_shutdown() and not self.voice_text:
            self.command_session_ready.set()
            rospy.loginfo("DUAL_ORDER_COMMAND_SESSION_READY")

    @staticmethod
    def _without_wake_words(text):
        compact = normalize_text(text).replace(" ", "")
        for word in ("小飞小飞", "小飞", "小辉小辉", "小辉"):
            compact = compact.replace(word, "")
        return compact.strip("，。！？,.!?：:")

    def extract_dual_categories(self, text):
        parts = re.split(r"仿真环境中|仿真环境", normalize_text(text), maxsplit=1)
        if len(parts) != 2:
            return "", ""
        real_category = self.extract_target_category(parts[0])
        sim_category = self.extract_target_category(parts[1])
        return real_category, sim_category

    def accept_voice(self, text):
        text = normalize_text(text)
        if not text:
            return

        # The wake utterance is deliberately separate from the order.  This
        # produces exactly "我在" and opens the command window only afterward.
        if self.has_wake_word(text) and not self._without_wake_words(text):
            if not self.awakened:
                self.awakened = True
                self.command_session_ready.clear()
                self.publish_state(STATE_WAITING_VOICE)
                self.speak("我在")
                rospy.loginfo("dual-order wake accepted; waiting full command")
                self.rearm_asr_if_needed()
            return

        if self.require_wake_before_order and not self.awakened:
            rospy.logwarn("dual order ignored before wake word: %s", text)
            return
        if not self.command_session_ready.is_set():
            # Ignore loudspeaker echo and any tail fragment captured by the
            # first one-shot AIUI session.  A fresh cloud_asr_test2 process is
            # the only process allowed to supply the command.
            rospy.loginfo_throttle(
                1.0, "waiting fresh ASR session; voice fragment ignored")
            return

        real_category, sim_category = self.extract_dual_categories(text)
        if not real_category or not sim_category:
            # cloud_asr_test2 publishes evolving ASR fragments.  Do not speak
            # or restart ASR on a partial sentence; its later fragment contains
            # the complete real + simulation order.
            rospy.loginfo_throttle(
                0.5, "waiting complete dual order real=%s simulation=%s text=%s",
                real_category, sim_category, text)
            return

        with self.lock:
            if self.voice_text:
                rospy.logwarn("dual order already accepted; ignoring: %s", text)
                return
            self.voice_text = text
            self.target_category = real_category
            self.sim_target_category = sim_category
            order_event = getattr(self, "order_accepted_event", None)
            if order_event is not None:
                order_event.set()

        rospy.loginfo("dual order accepted real=%s simulation=%s",
                      real_category, sim_category)
        self.speak("已接收任务，开始前往物品领取区。")
        self.start_nav_if_needed()
        self.try_decide()

    def decide_thread(self, voice_text, category, items, evidence=None):
        evidence = evidence or []
        sim_category = self.sim_target_category
        if not category or not sim_category:
            self.publish_error("真实环境或仿真环境目标大类缺失")
            return
        if self.require_spark_decision and not self.spark_ready():
            self.publish_error("星火大模型未配置，不能进行双目标货品筛选")
            return

        real_item = self.select_item_with_spark(category, items, evidence)
        if not real_item:
            self.publish_error("星火大模型未能选出真实环境目标货品")
            return
        sim_item = self.select_item_with_spark(sim_category, items, evidence)
        if not sim_item:
            self.publish_error("星火大模型未能选出仿真环境目标货品")
            return
        self.finish_dual_success(
            voice_text, category, real_item,
            sim_category, sim_item, items)

    def finish_success(self, voice_text, category, selected_item, items):
        """Compatibility entry point if a parent decision path calls it."""
        sim_category = self.sim_target_category
        if not sim_category:
            self.publish_error("仿真环境目标大类缺失")
            return
        sim_item = self.select_item_with_spark(sim_category, items, self.qr_evidence)
        if not sim_item:
            self.publish_error("星火大模型未能选出仿真环境目标货品")
            return
        self.finish_dual_success(
            voice_text, category, selected_item,
            sim_category, sim_item, items)

    def finish_dual_success(self, voice_text, real_category, real_item,
                            sim_category, sim_item, items):
        real_warehouse = WAREHOUSE_BY_CATEGORY.get(real_category, "未知车间")
        sim_warehouse = WAREHOUSE_BY_CATEGORY.get(sim_category, "未知车间")
        real_spoken_category = CATEGORY_SPEECH.get(real_category, real_category)
        sim_spoken_category = CATEGORY_SPEECH.get(sim_category, sim_category)
        text = (
            "取得{}属于{}应放置在{}，"
            "仿真环境中取得{}属于{}应放置在{}。"
        ).format(
            real_item, real_spoken_category, real_warehouse,
            sim_item, sim_spoken_category, sim_warehouse)
        payload = {
            "status": "success",
            "voice_text": voice_text,
            "target_category": real_category,
            "selected_item": real_item,
            "target_warehouse": real_warehouse,
            "sim_target_key": SIM_TARGET_KEY[sim_category],
            "sim_target_category": sim_category,
            "sim_selected_item": sim_item,
            "sim_target_warehouse": sim_warehouse,
            "scanned_items": items,
            "broadcast_text": text,
            "sim_task_ignored": False,
            "stamp": time.time(),
        }
        rospy.loginfo("dual subtask decision: %s",
                      json.dumps(payload, ensure_ascii=False))
        self.result_pub.publish(String(
            data=json.dumps(payload, ensure_ascii=False)))
        self.compat_pub.publish(String(data=json.dumps({
                "real_item": real_item,
                "real_category": real_category,
                "real_warehouse": real_warehouse,
                "sim_item": sim_item,
                "sim_category": sim_category,
                "sim_warehouse": sim_warehouse,
                "sim_target_key": SIM_TARGET_KEY[sim_category],
            }, ensure_ascii=False)))
        self.speak(text)
        if self.post_route_enabled:
            self.start_post_route_after_tts()


if __name__ == "__main__":
    Xunfei2026DualOrderFusion()
    rospy.spin()
