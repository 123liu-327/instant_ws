#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Manual-positioned real-car QR rotate/scan/analyse/speak acceptance test."""

from __future__ import annotations

import json
import math
import os
import random
import sys
import threading
import time
from collections import OrderedDict
from typing import Any, Dict, Iterable, List, Optional, Tuple

import rosgraph
import rospkg
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


_PACKAGE_SCRIPTS = os.path.join(
    rospkg.RosPack().get_path("ucar_2026_qr_speak_test"), "scripts")
if _PACKAGE_SCRIPTS not in sys.path:
    sys.path.insert(0, _PACKAGE_SCRIPTS)
from qr_speak_test_node import SparkX2Client, _parse_llm_json  # noqa: E402


TARGETS = {
    "food": {
        "category_name": "食品大类",
        "workshop": "食品加工车间",
    },
    "daily": {
        "category_name": "日用品大类",
        "workshop": "日用品加工车间",
    },
    "electronics": {
        "category_name": "电子产品大类",
        "workshop": "电子产品生产车间",
    },
}

TARGET_ALIASES = {
    "food": "food",
    "食品": "food",
    "食品大类": "food",
    "daily": "daily",
    "日用品": "daily",
    "日用品大类": "daily",
    "electronics": "electronics",
    "electronic": "electronics",
    "电子产品": "electronics",
    "电子产品大类": "electronics",
}

SINGLE_TARGET_SYSTEM_PROMPT = """你是智慧工厂真车专项测试的商品分类模块。
用户会给出三个现场二维码解析出的商品名称，以及本次唯一目标商品大类。
请从三个商品中选择一个属于目标大类的商品。不得编造列表外的商品。
只输出JSON对象，不要Markdown。格式：
{"selected_item":"列表中的商品或null","selected_category":"food|daily|electronics或null","confidence":0到1,"error":"无错误时为空字符串"}
若没有匹配商品，selected_item和selected_category必须为null，并在error中说明。"""


class TestStopped(RuntimeError):
    pass


class TestFailure(RuntimeError):
    pass


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def scan_parameters_from_degrees(first_deg: float, second_deg: float,
                                 offset_deg: float,
                                 keyframe_wait_sec: float):
    first_deg = float(first_deg)
    second_deg = float(second_deg)
    offset_deg = float(offset_deg)
    keyframe_wait_sec = float(keyframe_wait_sec)
    if first_deg <= 0.0 or second_deg <= 0.0:
        raise ValueError("两圈扫码步进角度必须大于0度")
    if offset_deg <= 0.0 or offset_deg > min(first_deg, second_deg):
        raise ValueError("第二圈偏移必须大于0度且不得超过任一圈步进角度")
    if keyframe_wait_sec <= 0.0:
        raise ValueError("关键帧等待时间必须大于0秒")
    return (
        math.radians(first_deg),
        math.radians(second_deg),
        math.radians(offset_deg),
        keyframe_wait_sec,
    )


def choose_target_category(requested: str = "", seed: int = -1) -> Dict[str, str]:
    requested = str(requested or "").strip().lower()
    if requested:
        key = TARGET_ALIASES.get(requested)
        if key is None:
            raise ValueError("unsupported target_category: {}".format(requested))
    else:
        generator = random.SystemRandom() if int(seed) < 0 else random.Random(int(seed))
        key = generator.choice(sorted(TARGETS))
    result = dict(TARGETS[key])
    result["key"] = key
    return result


def extract_qr_items(raw_text: str) -> Iterable[Tuple[str, str]]:
    try:
        payload = json.loads(str(raw_text or "").strip())
    except Exception:
        return []
    entries = payload.get("items") if isinstance(payload, dict) else None
    if not isinstance(entries, list):
        entries = [payload] if isinstance(payload, dict) else []
    output = []
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        raw = str(entry.get("raw") or "").strip()
        result = str(entry.get("result") or "").strip()
        api = entry.get("api")
        if not result and isinstance(api, dict):
            result = str(api.get("result") or "").strip()
        if not result and raw and not raw.startswith(("http://", "https://")):
            result = raw
        if raw and result:
            output.append((raw, result))
    return output


def build_analysis_prompt(items: List[str], target: Dict[str, str]) -> str:
    return (
        "本次唯一目标大类：{category}（键：{key}）\n"
        "对应生产车间：{workshop}\n"
        "现场商品：\n1) {a}\n2) {b}\n3) {c}"
    ).format(
        category=target["category_name"],
        key=target["key"],
        workshop=target["workshop"],
        a=items[0], b=items[1], c=items[2],
    )


def validate_analysis(data: Dict[str, Any], items: List[str],
                      target: Dict[str, str]) -> Dict[str, str]:
    error = str(data.get("error") or "").strip()
    selected_item = str(data.get("selected_item") or "").strip()
    selected_category = TARGET_ALIASES.get(
        str(data.get("selected_category") or "").strip().lower())
    if error:
        raise ValueError(error)
    if selected_item not in items:
        raise ValueError("Spark selected an item outside the QR list")
    if selected_category != target["key"]:
        raise ValueError("Spark category does not match the random target")
    announcement = "取得{}属于{}应放置在{}".format(
        selected_item, target["category_name"], target["workshop"])
    return {
        "selected_item": selected_item,
        "selected_category": target["key"],
        "selected_category_name": target["category_name"],
        "selected_workshop": target["workshop"],
        "announcement": announcement,
    }


class DirectedYawAccumulator:
    def __init__(self, direction: float) -> None:
        self.direction = 1.0 if direction >= 0.0 else -1.0
        self.last_yaw: Optional[float] = None
        self.progress = 0.0

    def reset(self, yaw: float) -> None:
        self.last_yaw = float(yaw)
        self.progress = 0.0

    def update(self, yaw: float) -> float:
        if self.last_yaw is None:
            self.reset(yaw)
            return self.progress
        delta = normalize_angle(float(yaw) - self.last_yaw) * self.direction
        self.last_yaw = float(yaw)
        if delta > 0.0:
            self.progress += delta
        return self.progress


class QRRotateScanSpeakTest:
    def __init__(self) -> None:
        self.camera_topic = rospy.get_param("~camera_topic", "/usb_cam/image_raw")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.qr_topic = rospy.get_param("~qr_topic", "/qr_code_data")
        self.decoder_status_topic = rospy.get_param(
            "~decoder_status_topic", "/qr_decoder/status")
        self.tts_topic = rospy.get_param("~tts_topic", "/factory/tts_text")
        self.expected_count = int(rospy.get_param("~expected_count", 3))
        self.target_category = rospy.get_param("~target_category", "")
        self.random_seed = int(rospy.get_param("~random_seed", -1))
        (self.step_angle,
         self.second_round_step_angle,
         self.second_round_offset,
         self.settle_sec) = scan_parameters_from_degrees(
            rospy.get_param("~first_round_step_angle_deg", 20.0),
            rospy.get_param("~second_round_step_angle_deg", 10.0),
            rospy.get_param("~second_round_offset_deg", 5.0),
            rospy.get_param("~keyframe_wait_sec", 0.30),
        )
        self.scan_speed = abs(float(rospy.get_param("~scan_speed", 0.60)))
        self.scan_direction = 1.0 if float(rospy.get_param(
            "~scan_direction", 1.0)) >= 0.0 else -1.0
        # First confirm odometry is stationary, then wait the configured
        # keyframe interval before the decoder is allowed to use an image.
        self.stationary_hold = max(
            0.05, float(rospy.get_param("~stationary_hold", 0.15)))
        self.stop_timeout = max(
            self.stationary_hold,
            float(rospy.get_param("~stop_timeout", 1.50)))
        self.linear_tolerance = max(
            0.0, float(rospy.get_param("~linear_tolerance", 0.025)))
        self.angular_tolerance = max(
            0.0, float(rospy.get_param("~angular_tolerance", 0.05)))
        self.odom_stale_sec = max(
            0.05, float(rospy.get_param("~odom_stale_sec", 0.50)))
        self.step_timeout_margin = max(
            0.2, float(rospy.get_param("~step_timeout_margin", 2.0)))
        self.scan_timeout = max(
            1.0, float(rospy.get_param("~scan_timeout", 90.0)))
        self.network_idle_sec = max(
            0.1, float(rospy.get_param("~network_idle_sec", 0.50)))
        self.api_password = rospy.get_param(
            "~api_password", os.environ.get("XF_SPARK_API_PASSWORD", ""))
        self.spark_base_url = rospy.get_param(
            "~spark_base_url",
            "https://spark-api-open.xf-yun.com/x2/chat/completions")
        self.spark_model = rospy.get_param("~spark_model", "spark-x")
        self.request_timeout_sec = float(rospy.get_param(
            "~request_timeout_sec", 90.0))

        self.lock = threading.RLock()
        self.abort_event = threading.Event()
        self.worker: Optional[threading.Thread] = None
        self.running = False
        self.collecting = False
        self.run_id = 0
        self.target: Optional[Dict[str, str]] = None
        self.qr_items: "OrderedDict[str, str]" = OrderedDict()
        self.odom_yaw: Optional[float] = None
        self.odom_received_at = 0.0
        self.base_twist: Optional[Tuple[float, float, float]] = None
        self.decoder_state = ""
        self.decoder_pending = 0
        self.decoder_status_at = 0.0

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=2)
        self.tts_pub = rospy.Publisher(self.tts_topic, String, queue_size=2)
        self.status_pub = rospy.Publisher(
            "/qr_rotate_scan_speak/status", String, queue_size=10, latch=True)
        self.result_pub = rospy.Publisher(
            "/qr_rotate_scan_speak/result", String, queue_size=2, latch=True)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber(self.qr_topic, String, self.qr_cb, queue_size=50)
        rospy.Subscriber(
            self.decoder_status_topic, String, self.decoder_status_cb,
            queue_size=10)
        self.start_service = rospy.Service(
            "/qr_rotate_scan_speak/start", Trigger, self.start_cb)
        self.stop_service = rospy.Service(
            "/qr_rotate_scan_speak/stop", Trigger, self.stop_cb)
        rospy.on_shutdown(self.on_shutdown)
        self.safe_stop()
        self.publish_status("waiting_for_start", "车辆保持停止，等待开始服务")

    def odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        twist = msg.twist.twist
        with self.lock:
            self.odom_yaw = yaw
            self.odom_received_at = time.monotonic()
            self.base_twist = (
                float(twist.linear.x), float(twist.linear.y),
                float(twist.angular.z))

    def decoder_status_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            state = str(payload.get("state") or "").strip().lower()
            pending = max(0, int(payload.get("pending_count", 0)))
        except Exception:
            return
        with self.lock:
            self.decoder_state = state
            self.decoder_pending = pending
            self.decoder_status_at = time.monotonic()

    def qr_cb(self, msg: String) -> None:
        with self.lock:
            if not self.collecting:
                return
        accepted = []
        for key, value in extract_qr_items(msg.data):
            with self.lock:
                if key in self.qr_items:
                    continue
                self.qr_items[key] = value
                accepted.append((len(self.qr_items), value))
        for count, value in accepted:
            rospy.loginfo("QR_ROTATE_TEST accepted %d/%d: %s",
                          count, self.expected_count, value)
            self.publish_status(
                "scanning", "已识别{}/{}：{}".format(
                    count, self.expected_count, value))

    def start_cb(self, _request: Trigger) -> TriggerResponse:
        with self.lock:
            if self.running:
                return TriggerResponse(False, "专项测试已经在运行")
        try:
            self.assert_safe_to_start()
            target = choose_target_category(
                self.target_category, self.random_seed)
        except Exception as exc:
            self.safe_stop()
            return TriggerResponse(False, str(exc))
        with self.lock:
            self.run_id += 1
            run_id = self.run_id
            self.target = target
            self.qr_items.clear()
            self.running = True
            self.collecting = True
            self.abort_event.clear()
            self.worker = threading.Thread(
                target=self.run_test, args=(run_id,),
                name="qr_rotate_scan_speak_test", daemon=True)
            self.worker.start()
        return TriggerResponse(
            True, "已开始，随机目标：{}".format(target["workshop"]))

    def stop_cb(self, _request: Trigger) -> TriggerResponse:
        with self.lock:
            was_running = self.running
        self.abort_event.set()
        self.safe_stop()
        return TriggerResponse(
            True, "已请求停止" if was_running else "当前没有运行中的测试")

    def assert_safe_to_start(self) -> None:
        now = time.monotonic()
        with self.lock:
            odom_ready = (
                self.odom_yaw is not None and
                now - self.odom_received_at <= self.odom_stale_sec)
            decoder_ready = self.decoder_state in ("ready", "idle", "fetching")
        if not odom_ready:
            raise TestFailure("/odom缺失或超过{:.2f}秒未更新".format(
                self.odom_stale_sec))
        if not decoder_ready:
            raise TestFailure("二维码节点尚未就绪")
        try:
            rospy.wait_for_message(self.camera_topic, Image, timeout=1.0)
        except rospy.ROSException:
            raise TestFailure("相机话题没有图像：{}".format(self.camera_topic))

        publishers, subscribers, services = rosgraph.Master(
            rospy.get_name()).getSystemState()
        all_nodes = set()
        for _topic, nodes in publishers + subscribers + services:
            all_nodes.update(nodes)
        unsafe_tokens = ("move_base", "simple_navigator", "competition_flow",
                         "strict_mission")
        unsafe_nodes = sorted(
            node for node in all_nodes
            if any(token in node for token in unsafe_tokens))
        if unsafe_nodes:
            raise TestFailure("检测到可能争用底盘的节点：{}".format(
                ", ".join(unsafe_nodes)))
        cmd_publishers = []
        for topic, nodes in publishers:
            if topic == self.cmd_topic:
                cmd_publishers.extend(node for node in nodes
                                      if node != rospy.get_name())
        if cmd_publishers:
            raise TestFailure("检测到其他{}发布者：{}".format(
                self.cmd_topic, ", ".join(sorted(set(cmd_publishers)))))

    def run_test(self, run_id: int) -> None:
        started_at = time.monotonic()
        result: Dict[str, Any] = {
            "run_id": run_id,
            "success": False,
            "started_at": time.time(),
        }
        try:
            target = dict(self.target or {})
            result.update({
                "target_category": target["key"],
                "target_category_name": target["category_name"],
                "target_workshop": target["workshop"],
            })
            self.publish_status(
                "target_selected",
                "随机目标：{}，调用扫码流程".format(target["workshop"]))
            self.publish_status(
                "scan_configuration",
                "扫码参数：第一圈{:.1f}度，偏移{:.1f}度，第二圈{:.1f}度，"
                "静止后等待{:.2f}秒".format(
                    math.degrees(self.step_angle),
                    math.degrees(self.second_round_offset),
                    math.degrees(self.second_round_step_angle),
                    self.settle_sec,
                ))
            self.speak("本次随机目标为{}，开始旋转扫码。".format(
                target["workshop"]))
            self.wait_zero(1.0)
            deadline = time.monotonic() + self.scan_timeout
            first_step_count = max(
                1, int(round(2.0 * math.pi / self.step_angle)))
            second_step_count = max(
                1, int(round(
                    2.0 * math.pi / self.second_round_step_angle)))
            self.run_step_round(
                1, self.step_angle, first_step_count, deadline)
            if self.qr_count() < self.expected_count:
                self.wait_network_idle(deadline, 1.5)
            if self.qr_count() < self.expected_count:
                self.rotate_step(self.second_round_offset, deadline)
                self.settle(deadline, "第二圈偏移")
                self.run_step_round(
                    2, self.second_round_step_angle,
                    second_step_count, deadline)
            if self.qr_count() < self.expected_count:
                self.wait_network_idle(deadline, 1.5)
            self.safe_stop()
            if self.qr_count() < self.expected_count:
                raise TestFailure("扫码结束，仅取得{}/{}个商品".format(
                    self.qr_count(), self.expected_count))
            with self.lock:
                self.collecting = False
                items = list(self.qr_items.values())[:self.expected_count]
            result["qr_items"] = items
            self.publish_status("analysing", "二维码齐全，正在分析随机目标")
            analysis = self.analyse(items, target)
            result.update(analysis)
            result["success"] = True
            result["elapsed_sec"] = time.monotonic() - started_at
            self.result_pub.publish(String(data=json.dumps(
                result, ensure_ascii=False, separators=(",", ":"))))
            self.publish_status("completed", analysis["announcement"])
            self.speak(analysis["announcement"] + "。")
        except TestStopped as exc:
            result["error"] = str(exc)
            result["elapsed_sec"] = time.monotonic() - started_at
            self.result_pub.publish(String(data=json.dumps(
                result, ensure_ascii=False, separators=(",", ":"))))
            self.publish_status("stopped", str(exc))
        except Exception as exc:
            result["error"] = str(exc)
            result["elapsed_sec"] = time.monotonic() - started_at
            self.result_pub.publish(String(data=json.dumps(
                result, ensure_ascii=False, separators=(",", ":"))))
            self.publish_status("failed", str(exc))
            self.speak("二维码专项测试失败，{}。".format(str(exc)))
            rospy.logerr("QR rotate/scan/speak test failed: %s", exc)
        finally:
            self.safe_stop()
            with self.lock:
                self.collecting = False
                self.running = False

    def run_step_round(self, round_index: int, step_angle: float,
                       step_count: int, deadline: float) -> None:
        self.publish_status(
            "scanning_round_{}".format(round_index),
            "开始第{}圈，每次旋转{:.1f}度".format(
                round_index, math.degrees(step_angle)))
        for step in range(1, step_count + 1):
            if self.qr_count() >= self.expected_count:
                return
            self.check_deadline(deadline)
            self.rotate_step(step_angle, deadline)
            self.settle(deadline, "第{}圈第{}步".format(round_index, step))

    def rotate_step(self, angle: float, deadline: float) -> None:
        if angle <= 0.0:
            return
        tracker = DirectedYawAccumulator(self.scan_direction)
        tracker.reset(self.fresh_yaw())
        step_deadline = min(
            deadline,
            time.monotonic() + angle / self.scan_speed +
            self.step_timeout_margin)
        rate = rospy.Rate(20)
        command = Twist()
        command.angular.z = self.scan_direction * self.scan_speed
        while tracker.progress < angle:
            self.check_abort()
            if self.qr_count() >= self.expected_count:
                break
            if time.monotonic() >= step_deadline:
                raise TestFailure("小角度旋转超时")
            tracker.update(self.fresh_yaw())
            if tracker.progress >= angle:
                break
            self.cmd_pub.publish(command)
            rate.sleep()
        self.cmd_pub.publish(Twist())

    def settle(self, deadline: float, label: str) -> None:
        local_deadline = min(
            deadline,
            time.monotonic() + self.stop_timeout +
            self.stationary_hold + self.settle_sec)
        stable_since = None
        rate = rospy.Rate(20)
        while time.monotonic() < local_deadline:
            self.check_abort()
            self.cmd_pub.publish(Twist())
            if self.qr_count() >= self.expected_count:
                return
            with self.lock:
                twist = self.base_twist
                age = time.monotonic() - self.odom_received_at
            stopped = (
                twist is not None and age <= self.odom_stale_sec and
                abs(twist[0]) <= self.linear_tolerance and
                abs(twist[1]) <= self.linear_tolerance and
                abs(twist[2]) <= self.angular_tolerance)
            if stopped:
                if stable_since is None:
                    stable_since = time.monotonic()
                if (time.monotonic() - stable_since >=
                        self.stationary_hold + self.settle_sec):
                    return
            else:
                stable_since = None
                if age > self.odom_stale_sec:
                    raise TestFailure("{}期间/odom超时".format(label))
            rate.sleep()
        raise TestFailure("{}未能稳定停止".format(label))

    def wait_network_idle(self, deadline: float, max_wait: float) -> None:
        wait_deadline = min(deadline, time.monotonic() + max_wait)
        idle_since = None
        rate = rospy.Rate(20)
        while time.monotonic() < wait_deadline:
            self.check_abort()
            self.cmd_pub.publish(Twist())
            if self.qr_count() >= self.expected_count:
                return
            with self.lock:
                pending = self.decoder_pending
            if pending <= 0:
                if idle_since is None:
                    idle_since = time.monotonic()
                elif time.monotonic() - idle_since >= self.network_idle_sec:
                    return
            else:
                idle_since = None
            rate.sleep()

    def analyse(self, items: List[str], target: Dict[str, str]) -> Dict[str, str]:
        if len(items) != self.expected_count:
            raise TestFailure("商品数量不完整")
        if not str(self.api_password or "").strip():
            raise TestFailure("XF_SPARK_API_PASSWORD为空")
        client = SparkX2Client(
            self.api_password, self.spark_base_url,
            self.spark_model, self.request_timeout_sec)
        content = client.chat(
            SINGLE_TARGET_SYSTEM_PROMPT,
            build_analysis_prompt(items, target))
        data = _parse_llm_json(content)
        try:
            return validate_analysis(data, items, target)
        except ValueError as exc:
            raise TestFailure(str(exc))

    def fresh_yaw(self) -> float:
        with self.lock:
            yaw = self.odom_yaw
            age = time.monotonic() - self.odom_received_at
        if yaw is None or age > self.odom_stale_sec:
            raise TestFailure("/odom缺失或超时")
        return yaw

    def qr_count(self) -> int:
        with self.lock:
            return len(self.qr_items)

    def check_abort(self) -> None:
        if rospy.is_shutdown() or self.abort_event.is_set():
            raise TestStopped("用户停止了专项测试")

    def check_deadline(self, deadline: float) -> None:
        self.check_abort()
        if time.monotonic() >= deadline:
            raise TestFailure("扫码总时间超过{:.1f}秒".format(
                self.scan_timeout))

    def speak(self, text: str) -> None:
        text = str(text or "").strip()
        if not text:
            return
        deadline = time.monotonic() + 2.0
        while (self.tts_pub.get_num_connections() <= 0 and
               time.monotonic() < deadline and not rospy.is_shutdown()):
            rospy.sleep(0.05)
        self.tts_pub.publish(String(data=text))
        rospy.loginfo("QR_ROTATE_TEST TTS: %s", text)

    def wait_zero(self, duration: float) -> None:
        deadline = time.monotonic() + max(0.0, duration)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.05)

    def safe_stop(self) -> None:
        zero = Twist()
        for _ in range(5):
            try:
                self.cmd_pub.publish(zero)
                rospy.sleep(0.02)
            except rospy.ROSInterruptException:
                break

    def publish_status(self, state: str, message: str) -> None:
        payload = {
            "stamp": time.time(),
            "state": str(state),
            "message": str(message),
            "qr_count": self.qr_count(),
            "expected_count": self.expected_count,
        }
        self.status_pub.publish(String(data=json.dumps(
            payload, ensure_ascii=False, separators=(",", ":"))))

    def on_shutdown(self) -> None:
        self.abort_event.set()
        self.safe_stop()


def main() -> None:
    rospy.init_node("qr_rotate_scan_speak_test")
    QRRotateScanSpeakTest()
    rospy.spin()


if __name__ == "__main__":
    main()
