#!/usr/bin/env python3
"""Two-target voice/QR fusion for the real-to-simulation collaboration task."""

import json
import re
import threading
import time

import rospy
from std_msgs.msg import Bool, String

from subtask1_real_order_fusion_route import (
    STATE_NAV_AND_SCAN,
    STATE_REASONING,
    STATE_WAITING_VOICE,
    Subtask1RealOrderFusion,
    WAREHOUSE_BY_CATEGORY,
    first_json_object,
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
        # The command window opens only after the wake response has finished.
        # cloud_asr_test2 itself stays alive and keeps listening until this
        # class explicitly confirms a complete competition command.
        self.command_session_ready = threading.Event()
        super().__init__()
        self.voice_command_accepted_pub = rospy.Publisher(
            "/factory/voice_command_accepted", Bool,
            queue_size=1, latch=True)
        rospy.loginfo("dual real/simulation order fusion ready")

    def rearm_asr_thread(self):
        """Open the command window without killing the persistent ASR node."""
        try:
            self.wait_for_prompt_playback()
            if not rospy.is_shutdown() and not self.order_accepted_event.is_set():
                self.command_session_ready.set()
                rospy.loginfo("VOICE_READY waiting competition command")
        finally:
            with self.asr_rearm_lock:
                self.asr_rearm_in_progress = False

    @staticmethod
    def _is_exact_wake(text):
        compact = re.sub(
            r"[\s，。！？,.!?：:]", "", normalize_text(text))
        return compact == "小飞小飞"

    @staticmethod
    def _without_wake_words(text):
        compact = normalize_text(text).replace(" ", "")
        for word in ("小飞小飞",):
            compact = compact.replace(word, "")
        return compact.strip("，。！？,.!?：:")

    def extract_dual_categories(self, text):
        parts = re.split(r"仿真环境中|仿真环境", normalize_text(text), maxsplit=1)
        if len(parts) != 2:
            return "", ""
        real_category = self.extract_target_category(parts[0])
        sim_category = self.extract_target_category(parts[1])
        return real_category, sim_category

    @staticmethod
    def is_complete_competition_command(text):
        compact = normalize_text(text).replace(" ", "")
        parts = re.split(r"仿真环境中|仿真环境", compact, maxsplit=1)
        if len(parts) != 2:
            return False
        pickup_words = ("取得", "领取", "拿取")
        # Cloud ASR can validly end at the second category and omit the spoken
        # placement tail.  Requiring an acquisition verb independently in the
        # real and simulation clauses preserves a strong competition-command
        # signature without rejecting that clipped final transcript.
        return all((
            "物品领取区" in parts[0],
            any(word in parts[0] for word in pickup_words),
            any(word in parts[1] for word in pickup_words),
        ))

    def accept_voice(self, text):
        text = normalize_text(text)
        if not text:
            return

        # The wake utterance is deliberately separate from the order.  This
        # produces exactly "我在" and opens the command window only afterward.
        if self._is_exact_wake(text):
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
            # Ignore loudspeaker echo and wake-utterance tail fragments while
            # "我在" is playing.  The same persistent ASR process continues
            # supplying audio as soon as the command window opens.
            rospy.loginfo_throttle(
                1.0, "waiting wake response playback; voice fragment ignored")
            return

        real_category, sim_category = self.extract_dual_categories(text)
        complete_command = self.is_complete_competition_command(text)
        if not complete_command or not real_category or not sim_category:
            # cloud_asr_test2 publishes evolving ASR fragments.  Do not speak
            # or restart ASR on a partial sentence; its later fragment contains
            # the complete real + simulation order.
            rospy.loginfo_throttle(
                0.5,
                "waiting competition command format=%s real=%s simulation=%s",
                complete_command, real_category, sim_category)
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

        # This is the only signal that permits the persistent ASR backend to
        # stop recording.  Partial and unrelated utterances never publish it.
        self.voice_command_accepted_pub.publish(Bool(data=True))
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

        # One authenticated request selects both independent targets.  The old
        # implementation serialized two full HTTP calls, leaving the vehicle
        # stationary for roughly twice one Spark latency after QR completion.
        real_item, sim_item = self.select_dual_items_with_spark(
            category, sim_category, items, evidence)
        if not real_item or not sim_item:
            # Preserve the proven parser as a compatibility fallback only;
            # normal successful runs use the single combined request above.
            rospy.logwarn("DUAL_SPARK_COMBINED_FALLBACK sequential=true")
            real_item = self.select_item_with_spark(category, items, evidence)
            sim_item = self.select_item_with_spark(
                sim_category, items, evidence) if real_item else ""
        if not real_item:
            self.publish_error("星火大模型未能选出真实环境目标货品")
            return
        if not sim_item:
            self.publish_error("星火大模型未能选出仿真环境目标货品")
            return
        self.finish_dual_success(
            voice_text, category, real_item,
            sim_category, sim_item, items)

    def select_dual_items_with_spark(self, real_category, sim_category,
                                     items, evidence):
        candidates = []
        for item in items:
            cleaned = self.clean_candidate_item(item)
            if cleaned and cleaned not in candidates:
                candidates.append(cleaned)
        payload = {
            "real_target_category": real_category,
            "simulation_target_category": sim_category,
            "candidate_items": candidates,
            "qr_evidence": evidence or [],
            "rules": [
                "Each selected item must come from candidate_items.",
                "QR URL/category_hint is authoritative: food=食品加工类, daily=日用品类, electronic=电子产品类.",
                "Select the real and simulation items independently.",
                "Do not invent an item and do not exchange the two targets.",
                "Return strict JSON only.",
            ],
            "required_output": {
                "real_item": "真实环境目标货品原名",
                "sim_item": "仿真环境目标货品原名",
            },
        }
        prompt = (
            "你是智慧工厂双目标货品筛选节点。根据两个目标大类和二维码证据，"
            "一次性选出真实环境与仿真环境货品。只返回严格JSON："
            "{\"real_item\":\"名称\",\"sim_item\":\"名称\"}。\n" +
            json.dumps(payload, ensure_ascii=False))
        for attempt in range(2):
            data = self.call_spark(prompt, max_tokens=160)
            content = self.extract_spark_content(data)
            parsed = first_json_object(content)
            if isinstance(parsed, dict):
                real_item = self.clean_candidate_item(
                    parsed.get("real_item", ""))
                sim_item = self.clean_candidate_item(
                    parsed.get("sim_item", ""))
                if real_item not in candidates or sim_item not in candidates:
                    real_item, sim_item = "", ""
                if real_item and self.selected_item_conflicts_with_evidence(
                        real_category, real_item, evidence):
                    real_item = ""
                if sim_item and self.selected_item_conflicts_with_evidence(
                        sim_category, sim_item, evidence):
                    sim_item = ""
                if real_item and sim_item:
                    rospy.loginfo(
                        "DUAL_SPARK_COMBINED_OK real=%s simulation=%s",
                        real_item, sim_item)
                    return real_item, sim_item
            rospy.logwarn(
                "DUAL_SPARK_COMBINED_PARSE_RETRY attempt=%d content=%s",
                attempt + 1, content[:200])
            prompt += "\n上一次格式不正确。只返回包含real_item和sim_item的JSON。"
        return "", ""

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
