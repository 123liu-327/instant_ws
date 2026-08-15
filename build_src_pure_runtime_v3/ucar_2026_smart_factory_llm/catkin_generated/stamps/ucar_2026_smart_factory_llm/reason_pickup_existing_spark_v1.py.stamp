#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Expose the src3 reasoning service through the user's existing Spark keys."""

import json
import os
import re
import threading
import time
import unicodedata

import requests
import rospy
from ucar_2026_smart_factory_llm.srv import (
    ReasonPickupOrder,
    ReasonPickupOrderResponse,
)


SYSTEM_PROMPT = """你是第21届全国大学生智能汽车竞赛讯飞智慧工厂赛项的调度推理模块。
输入包含现场三个二维码得到的货品名称，以及完整语音指令。你必须依靠星火模型完成归类和选择，不使用本地货品词典。

大类与车间全名：
- 食品、食品加工、生鲜、食材相关 -> 食品大类，食品加工车间
- 日用品、日化、纺织、清洁用品相关 -> 日用品大类，日用品加工车间
- 电子产品、数码、电器相关 -> 电子产品大类，电子产品生产车间

从语音中解析现实环境目标大类和仿真环境目标大类，再分别从三个货品中选择唯一匹配项。不得创造输入中不存在的货品。
只输出一个 JSON 对象，不要 Markdown，不要解释，键必须齐全：
{
  "pickup_item":"字符串或null",
  "pickup_major":"食品大类|日用品大类|电子产品大类之一或null",
  "pickup_workshop":"食品加工车间|日用品加工车间|电子产品生产车间之一或null",
  "sim_item":"字符串或null",
  "sim_major":"食品大类|日用品大类|电子产品大类之一或null",
  "sim_workshop":"食品加工车间|日用品加工车间|电子产品生产车间之一或null",
  "announcement_physical":"取得X属于Y应放置在Z",
  "announcement_simulation":"仿真环境中取得X属于Y应放置在Z",
  "err_hint":"无问题时为空字符串"
}
"""


WORKSHOP_FOR_MAJOR = {
    "食品大类": "食品加工车间",
    "日用品大类": "日用品加工车间",
    "电子产品大类": "电子产品生产车间",
}


def compact_text(value):
    text = unicodedata.normalize("NFKC", str(value or "")).strip().lower()
    return re.sub(r"[\s，。！？、,.!?;；:：\"'“”‘’]+", "", text)


def canonical_major(value):
    text = compact_text(value)
    if "日用品" in text or "daily" in text:
        return "日用品大类"
    if "电子产品" in text or "electronics" in text or "electronic" in text:
        return "电子产品大类"
    if "食品" in text or "food" in text:
        return "食品大类"
    return ""


def canonical_item(value, items):
    token = compact_text(value)
    matches = [item for item in items if compact_text(item) == token]
    return matches[0] if len(matches) == 1 else ""


def expected_majors_from_voice(voice):
    text = compact_text(voice)
    split_at = -1
    for marker in ("仿真环境", "模拟环境", "虚拟环境", "仿真"):
        pos = text.find(marker)
        if pos >= 0 and (split_at < 0 or pos < split_at):
            split_at = pos
    if split_at < 0:
        return canonical_major(text), ""
    return canonical_major(text[:split_at]), canonical_major(text[split_at:])


def parse_json_object(content):
    text = str(content or "").strip()
    fenced = re.search(r"```(?:json)?\s*([\s\S]*?)```", text, re.IGNORECASE)
    if fenced:
        text = fenced.group(1).strip()
    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end <= start:
        raise ValueError("模型输出中没有 JSON 对象")
    return json.loads(text[start : end + 1])


def response_template():
    res = ReasonPickupOrderResponse()
    res.success = False
    res.error_message = ""
    res.announcement_physical = ""
    res.announcement_simulation = ""
    res.announcement_full = ""
    res.pickup_item = ""
    res.pickup_major = ""
    res.pickup_workshop = ""
    res.sim_item = ""
    res.sim_major = ""
    res.sim_workshop = ""
    res.raw_model_reply = ""
    return res


class ExistingSparkReasoningServer:
    def __init__(self):
        self.api_password = str(
            rospy.get_param(
                "~spark_api_password", os.environ.get("SPARK_API_PASSWORD", "")
            )
        ).strip()
        self.api_key = str(
            rospy.get_param("~spark_api_key", os.environ.get("SPARK_API_KEY", ""))
        ).strip()
        self.api_secret = str(
            rospy.get_param(
                "~spark_api_secret", os.environ.get("SPARK_API_SECRET", "")
            )
        ).strip()
        if self.api_password:
            self.authorization_token = self.api_password
            self.authorization_mode = "SPARK_API_PASSWORD"
        elif self.api_key and self.api_secret:
            self.authorization_token = "{}:{}".format(
                self.api_key, self.api_secret
            )
            self.authorization_mode = "SPARK_API_KEY:SPARK_API_SECRET"
        else:
            self.authorization_token = ""
            self.authorization_mode = "missing"
        self.url = str(
            rospy.get_param(
                "~spark_url",
                "https://spark-api-open.xf-yun.com/x2/chat/completions",
            )
        ).strip()
        self.model = str(rospy.get_param("~spark_model", "spark-x")).strip()
        self.timeout_s = float(rospy.get_param("~spark_timeout_s", 25.0))
        self.max_attempts = max(1, int(rospy.get_param("~max_attempts", 2)))
        self.session = requests.Session()
        self.http_lock = threading.Lock()
        rospy.Service("~reason_pickup_order", ReasonPickupOrder, self.handle)
        if not self.authorization_token:
            rospy.logerr("SRC3_EXISTING_SPARK_MISSING_CREDENTIALS")
        else:
            rospy.logwarn(
                "SRC3_EXISTING_SPARK_READY model=%s auth=%s",
                self.model,
                self.authorization_mode,
            )

    def call_spark(self, items, voice, correction=""):
        prompt = (
            "三个二维码货品：1) {} 2) {} 3) {}\n完整语音指令：{}\n{}"
        ).format(items[0], items[1], items[2], voice, correction)
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": prompt},
            ],
            "temperature": 0.05,
            "max_tokens": 520,
            "stream": False,
        }
        headers = {
            "Authorization": "Bearer {}".format(self.authorization_token),
            "Content-Type": "application/json",
        }
        started = time.monotonic()
        rospy.logwarn(
            "SRC3_SPARK_CALL_START model=%s items=%s voice_chars=%d",
            self.model,
            items,
            len(voice),
        )
        with self.http_lock:
            response = self.session.post(
                self.url, headers=headers, json=payload, timeout=self.timeout_s
            )
        elapsed = time.monotonic() - started
        if response.status_code != 200:
            raise RuntimeError(
                "Spark HTTP {} after {:.2f}s: {}".format(
                    response.status_code, elapsed, response.text[:300]
                )
            )
        body = response.json()
        choices = body.get("choices") or []
        if not choices:
            raise ValueError("Spark 响应缺少 choices")
        content = str((choices[0].get("message") or {}).get("content") or "")
        if not content:
            raise ValueError("Spark 响应 content 为空")
        rospy.logwarn(
            "SRC3_SPARK_HTTP_OK elapsed=%.2fs response_chars=%d", elapsed, len(content)
        )
        return content

    @staticmethod
    def fill_result(res, data):
        fields = (
            "pickup_item",
            "pickup_major",
            "pickup_workshop",
            "sim_item",
            "sim_major",
            "sim_workshop",
        )
        for field in fields:
            setattr(res, field, str(data.get(field) or "").strip())

        res.announcement_physical = str(
            data.get("announcement_physical") or ""
        ).strip()
        res.announcement_simulation = str(
            data.get("announcement_simulation") or ""
        ).strip()
        if not res.announcement_physical and res.pickup_item:
            res.announcement_physical = "取得{}属于{}应放置在{}".format(
                res.pickup_item, res.pickup_major, res.pickup_workshop
            )
        if not res.announcement_simulation and res.sim_item:
            res.announcement_simulation = "仿真环境中取得{}属于{}应放置在{}".format(
                res.sim_item, res.sim_major, res.sim_workshop
            )
        res.announcement_full = "，".join(
            x
            for x in (res.announcement_physical, res.announcement_simulation)
            if x
        )

    def validate(self, res, items, voice):
        required = (
            res.pickup_item,
            res.pickup_major,
            res.pickup_workshop,
            res.sim_item,
            res.sim_major,
            res.sim_workshop,
        )
        if not all(required):
            return "模型返回字段不完整"

        pickup_item = canonical_item(res.pickup_item, items)
        sim_item = canonical_item(res.sim_item, items)
        if not pickup_item or not sim_item:
            return "模型选择了二维码之外的货品"
        if pickup_item == sim_item:
            return "现实任务和仿真任务不能选择同一个二维码货品"

        pickup_major = canonical_major(res.pickup_major)
        sim_major = canonical_major(res.sim_major)
        if not pickup_major or not sim_major:
            return "模型返回了无法识别的货品大类"
        if pickup_major == sim_major:
            return "现实任务和仿真任务的大类必须不同"

        expected_pickup, expected_sim = expected_majors_from_voice(voice)
        if expected_pickup and pickup_major != expected_pickup:
            return "现实任务大类与语音指令不一致"
        if expected_sim and sim_major != expected_sim:
            return "仿真任务大类与语音指令不一致"

        pickup_workshop = WORKSHOP_FOR_MAJOR[pickup_major]
        sim_workshop = WORKSHOP_FOR_MAJOR[sim_major]
        if canonical_major(res.pickup_workshop) != pickup_major:
            return "现实任务车间与大类不一致"
        if canonical_major(res.sim_workshop) != sim_major:
            return "仿真任务车间与大类不一致"

        res.pickup_item = pickup_item
        res.sim_item = sim_item
        res.pickup_major = pickup_major
        res.sim_major = sim_major
        res.pickup_workshop = pickup_workshop
        res.sim_workshop = sim_workshop
        res.announcement_physical = "取得{}属于{}应放置在{}".format(
            pickup_item, pickup_major, pickup_workshop
        )
        res.announcement_simulation = "仿真环境中取得{}属于{}应放置在{}".format(
            sim_item, sim_major, sim_workshop
        )
        res.announcement_full = "，".join(
            (res.announcement_physical, res.announcement_simulation)
        )
        return ""

    def handle(self, req):
        res = response_template()
        items = [req.item_a.strip(), req.item_b.strip(), req.item_c.strip()]
        voice = str(req.voice_instruction or "").strip()
        if not all(items) or not voice:
            res.error_message = "三个二维码货品和语音指令均不能为空"
            return res
        if not self.authorization_token:
            res.error_message = (
                "SPARK_API_PASSWORD 或 SPARK_API_KEY/SPARK_API_SECRET 未配置"
            )
            return res

        correction = ""
        last_error = ""
        for attempt in range(1, self.max_attempts + 1):
            try:
                content = self.call_spark(items, voice, correction)
                res.raw_model_reply = content
                data = parse_json_object(content)
                self.fill_result(res, data)
                last_error = self.validate(res, items, voice)
                if not last_error:
                    res.success = True
                    rospy.logwarn(
                        "SRC3_SPARK_DECISION_OK pickup=%s/%s sim=%s/%s",
                        res.pickup_item,
                        res.pickup_workshop,
                        res.sim_item,
                        res.sim_workshop,
                    )
                    return res
            except Exception as exc:
                last_error = str(exc)
            rospy.logwarn(
                "SRC3_SPARK_ATTEMPT_FAILED attempt=%d/%d reason=%s",
                attempt,
                self.max_attempts,
                last_error,
            )
            correction = (
                "上一次输出不可用：{}。请重新检查，严格只输出完整 JSON；"
                "pickup_item 和 sim_item 必须逐字选自三个二维码货品。"
            ).format(last_error)

        res.error_message = "星火大模型推理失败：{}".format(last_error)
        return res


def main():
    rospy.init_node("smart_factory_llm")
    ExistingSparkReasoningServer()
    rospy.spin()


if __name__ == "__main__":
    main()
