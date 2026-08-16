#!/usr/bin/env python3
import os
import unittest

from ucar_qr_decoder.logic import (
    ReplaceableJobBuffer,
    legacy_url_payload,
    rank_candidates,
    run_decode_pipeline,
)


class DecoderLogicTest(unittest.TestCase):
    def test_rank_and_top_three_probe_order(self):
        candidates = [{"id": i, "score": score} for i, score in enumerate(
            [0.1, 0.9, 0.4, 0.8, 0.7, 0.2])]
        self.assertEqual([1, 3, 4], [item["id"] for item in rank_candidates(candidates)])
        order = []

        def raw(candidate):
            order.append(candidate["id"])
            return [], 1.0, "zbar_none", False

        run_decode_pipeline(candidates, "zbar_only", raw,
                            lambda item: ([], 2.0, "zbar_none", False),
                            lambda item: ([], 3.0, "opencv_none"))
        self.assertEqual([1, 3, 4], order)

    def test_zbar_hit_skips_opencv(self):
        called = []
        result, selected, metadata = run_decode_pipeline(
            [{"id": 1, "score": 1.0}], "hybrid",
            lambda item: (["ok"], 4.0, "zbar_raw", False),
            lambda item: self.fail("enhanced must not run"),
            lambda item: self.fail("opencv must not run"),
        )
        self.assertEqual(["ok"], result)
        self.assertEqual(1, selected["id"])
        self.assertEqual("zbar_raw", metadata["hit_stage"])
        self.assertEqual([], called)

    def test_timeout_and_crash_fall_back_to_opencv(self):
        result, _, metadata = run_decode_pipeline(
            [{"id": 1, "score": 1.0}], "hybrid",
            lambda item: ([], 60.0, "zbar_timeout", True),
            lambda item: (_ for _ in ()).throw(RuntimeError("backend crashed")),
            lambda item: (["fallback"], 12.0, "opencv_raw"),
        )
        self.assertEqual(["fallback"], result)
        self.assertEqual("opencv", metadata["backend"])
        self.assertEqual(1, metadata["zbar_timeouts"])

    def test_opencv_only_does_not_call_zbar(self):
        result, _, _ = run_decode_pipeline(
            [{"score": 1.0}], "opencv_only",
            lambda item: self.fail("zbar raw must not run"),
            lambda item: self.fail("zbar enhanced must not run"),
            lambda item: (["opencv"], 3.0, "opencv_raw"),
        )
        self.assertEqual(["opencv"], result)

    def test_pending_slot_is_replaceable(self):
        queue = ReplaceableJobBuffer()
        self.assertFalse(queue.put({"sequence": 1}))
        self.assertTrue(queue.put({"sequence": 2}))
        self.assertEqual(2, queue.take()["sequence"])
        self.assertIsNone(queue.take())

    def test_legacy_url_payload(self):
        ok = legacy_url_payload("https://example.test", {"code": 200, "result": "apple"})
        self.assertEqual("success", ok["status"])
        failed = legacy_url_payload("https://example.test", {"code": -1, "error": "down"})
        self.assertEqual("request_error", failed["error_type"])

    def test_keyframe_decoder_contains_no_motion_decode_fallback(self):
        package_dir = os.path.abspath(os.path.join(
            os.path.dirname(__file__), ".."))
        node_path = os.path.join(package_dir, "scripts", "qr_decoder_node.py")
        launch_path = os.path.join(package_dir, "launch", "qr_decoder.launch")
        with open(node_path, "r", encoding="utf-8") as stream:
            node_source = stream.read()
        with open(launch_path, "r", encoding="utf-8") as stream:
            launch_source = stream.read()
        for forbidden in ("slow_motion_fallback", "moving_decode", "slow_motion_since"):
            self.assertNotIn(forbidden, node_source)
            self.assertNotIn(forbidden, launch_source)
        self.assertIn("default=1", node_source)
        self.assertIn("default=0.30", node_source)


if __name__ == "__main__":
    unittest.main()
