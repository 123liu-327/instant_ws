#!/usr/bin/env python3

import json
import math
import os
import sys
import threading
import time
import unittest
from collections import OrderedDict


SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from qr_rotate_scan_speak_test_node import (  # noqa: E402
    DirectedYawAccumulator,
    QRRotateScanSpeakTest,
    TARGETS,
    TestFailure,
    TestStopped,
    build_analysis_prompt,
    choose_target_category,
    extract_qr_items,
    scan_parameters_from_degrees,
    validate_analysis,
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class QRRotateScanSpeakLogicTest(unittest.TestCase):
    def test_degree_scan_parameters_and_default_step_counts(self):
        first, second, offset, wait = scan_parameters_from_degrees(
            20.0, 10.0, 5.0, 0.30)
        self.assertAlmostEqual(first, math.radians(20.0))
        self.assertAlmostEqual(second, math.radians(10.0))
        self.assertAlmostEqual(offset, math.radians(5.0))
        self.assertAlmostEqual(wait, 0.30)
        self.assertEqual(18, int(round(2.0 * math.pi / first)))
        self.assertEqual(36, int(round(2.0 * math.pi / second)))

    def test_custom_degree_scan_parameters(self):
        first, second, offset, wait = scan_parameters_from_degrees(
            30.0, 12.0, 3.0, 0.45)
        self.assertEqual(12, int(round(2.0 * math.pi / first)))
        self.assertEqual(30, int(round(2.0 * math.pi / second)))
        self.assertAlmostEqual(offset, math.radians(3.0))
        self.assertAlmostEqual(wait, 0.45)

    def test_invalid_degree_scan_parameters_rejected(self):
        for values in (
                (0.0, 10.0, 5.0, 0.30),
                (20.0, -1.0, 5.0, 0.30),
                (20.0, 10.0, 11.0, 0.30),
                (20.0, 10.0, 5.0, 0.0)):
            with self.assertRaises(ValueError):
                scan_parameters_from_degrees(*values)

    def test_fixed_target_and_alias(self):
        self.assertEqual(choose_target_category("food")["key"], "food")
        self.assertEqual(choose_target_category("日用品")["key"], "daily")
        self.assertEqual(
            choose_target_category("电子产品大类")["key"], "electronics")

    def test_random_target_is_seeded_and_valid(self):
        first = choose_target_category("", 2026)
        second = choose_target_category("", 2026)
        self.assertEqual(first, second)
        self.assertIn(first["key"], TARGETS)

    def test_invalid_target_rejected(self):
        with self.assertRaises(ValueError):
            choose_target_category("simulation")

    def test_extract_and_deduplicate_qr_results(self):
        payload = json.dumps({"items": [
            {"raw": "https://example/a", "result": "饼干"},
            {"raw": "https://example/a", "result": "饼干"},
            {"raw": "https://example/b", "api": {"result": "纸巾"}},
        ]}, ensure_ascii=False)
        deduped = dict(extract_qr_items(payload))
        self.assertEqual(deduped, {
            "https://example/a": "饼干",
            "https://example/b": "纸巾",
        })

    def test_analysis_prompt_is_physical_only(self):
        target = choose_target_category("food")
        prompt = build_analysis_prompt(["饼干", "纸巾", "芯片"], target)
        self.assertIn("食品加工车间", prompt)
        self.assertNotIn("仿真", prompt)

    def test_validate_analysis(self):
        target = choose_target_category("daily")
        result = validate_analysis({
            "selected_item": "纸巾",
            "selected_category": "daily",
            "confidence": 0.99,
            "error": "",
        }, ["饼干", "纸巾", "芯片"], target)
        self.assertEqual(result["selected_workshop"], "日用品加工车间")
        self.assertNotIn("仿真", result["announcement"])

    def test_validate_rejects_outside_item_and_wrong_category(self):
        target = choose_target_category("food")
        with self.assertRaises(ValueError):
            validate_analysis({
                "selected_item": "牛奶",
                "selected_category": "food",
                "error": "",
            }, ["饼干", "纸巾", "芯片"], target)
        with self.assertRaises(ValueError):
            validate_analysis({
                "selected_item": "饼干",
                "selected_category": "daily",
                "error": "",
            }, ["饼干", "纸巾", "芯片"], target)

    def test_directed_yaw_accumulates_across_wrap(self):
        tracker = DirectedYawAccumulator(1.0)
        tracker.reset(math.radians(179.0))
        progress = tracker.update(math.radians(-176.0))
        self.assertAlmostEqual(progress, math.radians(5.0), places=5)
        tracker.update(math.radians(-171.0))
        self.assertAlmostEqual(tracker.progress, math.radians(10.0), places=5)

    def test_run_uses_offset_second_round_then_completes(self):
        node = QRRotateScanSpeakTest.__new__(QRRotateScanSpeakTest)
        node.target = choose_target_category("food")
        node.expected_count = 3
        node.step_angle = math.radians(20.0)
        node.second_round_step_angle = math.radians(10.0)
        node.second_round_offset = math.radians(5.0)
        node.settle_sec = 0.30
        node.scan_timeout = 90.0
        node.lock = threading.RLock()
        node.qr_items = OrderedDict()
        node.collecting = True
        node.running = True
        node.result_pub = FakePublisher()
        node.events = []
        node.publish_status = lambda state, message: node.events.append((state, message))
        node.speak = lambda text: node.events.append(("tts", text))
        node.safe_stop = lambda: node.events.append(("stop", ""))
        node.wait_zero = lambda duration: None
        node.wait_network_idle = lambda deadline, wait: None
        node.rotate_step = lambda angle, deadline: node.events.append(("offset", angle))
        node.settle = lambda deadline, label: node.events.append(("settle", label))
        node.analyse = lambda items, target: {
            "selected_item": "饼干",
            "selected_category": "food",
            "selected_category_name": "食品大类",
            "selected_workshop": "食品加工车间",
            "announcement": "取得饼干属于食品大类应放置在食品加工车间",
        }

        def fake_round(round_index, step_angle, step_count, deadline):
            node.events.append(("round", round_index, step_angle, step_count))
            if round_index == 2:
                node.qr_items.update([
                    ("url-a", "饼干"),
                    ("url-b", "纸巾"),
                    ("url-c", "芯片"),
                ])

        node.run_step_round = fake_round
        node.run_test(7)
        self.assertIn(("round", 1, node.step_angle, 18), node.events)
        self.assertIn(
            ("round", 2, node.second_round_step_angle, 36), node.events)
        self.assertIn(("offset", node.second_round_offset), node.events)
        self.assertNotIn(("slow", ""), node.events)
        result = json.loads(node.result_pub.messages[-1].data)
        self.assertTrue(result["success"])
        self.assertNotIn("仿真", json.dumps(result, ensure_ascii=False))

    def test_two_incomplete_rounds_stop_and_fail_without_continuous_scan(self):
        node = QRRotateScanSpeakTest.__new__(QRRotateScanSpeakTest)
        node.target = choose_target_category("food")
        node.expected_count = 3
        node.step_angle = math.radians(20.0)
        node.second_round_step_angle = math.radians(10.0)
        node.second_round_offset = math.radians(5.0)
        node.settle_sec = 0.30
        node.scan_timeout = 90.0
        node.lock = threading.RLock()
        node.qr_items = OrderedDict()
        node.collecting = True
        node.running = True
        node.result_pub = FakePublisher()
        node.events = []
        node.publish_status = lambda state, message: node.events.append((state, message))
        node.speak = lambda text: node.events.append(("tts", text))
        node.safe_stop = lambda: node.events.append(("stop", ""))
        node.wait_zero = lambda duration: None
        node.wait_network_idle = lambda deadline, wait: None
        node.rotate_step = lambda angle, deadline: node.events.append(("offset", angle))
        node.settle = lambda deadline, label: node.events.append(("settle", label))
        node.run_step_round = lambda round_index, step_angle, step_count, deadline: (
            node.events.append(("round", round_index, step_angle, step_count)))
        node.analyse = lambda items, target: self.fail(
            "incomplete scans must never enter analysis")

        node.run_test(8)

        self.assertIn(("round", 1, node.step_angle, 18), node.events)
        self.assertIn(
            ("round", 2, node.second_round_step_angle, 36), node.events)
        self.assertIn(("offset", node.second_round_offset), node.events)
        self.assertGreaterEqual(node.events.count(("stop", "")), 2)
        result = json.loads(node.result_pub.messages[-1].data)
        self.assertFalse(result["success"])
        self.assertIn("仅取得0/3个商品", result["error"])

    def test_stop_and_timeout_guards(self):
        node = QRRotateScanSpeakTest.__new__(QRRotateScanSpeakTest)
        node.abort_event = threading.Event()
        node.scan_timeout = 1.0
        node.abort_event.set()
        with self.assertRaises(TestStopped):
            node.check_abort()
        node.abort_event.clear()
        with self.assertRaises(TestFailure):
            node.check_deadline(time.monotonic() - 0.01)


if __name__ == "__main__":
    unittest.main()
