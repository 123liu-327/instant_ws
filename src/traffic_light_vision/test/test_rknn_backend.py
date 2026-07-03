#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import unittest

import numpy as np

SCRIPT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "scripts"))
sys.path.insert(0, SCRIPT_DIR)

from rknn_backend import (  # noqa: E402
    Detection,
    TemporalVoter,
    class_aware_nms,
    decode_yolov5_heads,
    letterbox,
    scale_detections,
    select_primary_detection,
)


class LetterboxTest(unittest.TestCase):
    def test_letterbox_and_scale_restore_original_coordinates(self):
        image = np.zeros((360, 640, 3), dtype=np.uint8)
        prepared, ratio, padding = letterbox(image, 640)
        self.assertEqual(prepared.shape, (640, 640, 3))
        self.assertAlmostEqual(ratio, 1.0)
        self.assertEqual(padding, (0.0, 140.0))

        detection = Detection(2, "left", 0.9, (100.0, 190.0, 200.0, 290.0))
        restored = scale_detections([detection], ratio, padding, image.shape[:2])[0]
        self.assertEqual(restored.box, (100.0, 50.0, 200.0, 150.0))


class DecodeTest(unittest.TestCase):
    def test_decode_three_nchw_heads(self):
        outputs = [
            np.zeros((1, 27, 80, 80), dtype=np.float32),
            np.zeros((1, 27, 40, 40), dtype=np.float32),
            np.zeros((1, 27, 20, 20), dtype=np.float32),
        ]
        # Anchor 0, grid y=10/x=20, class 2 (left).
        attributes = [0.5, 0.5, 0.5, 0.5, 0.9, 0.01, 0.01, 0.9, 0.01]
        for channel, value in enumerate(attributes):
            outputs[0][0, channel, 10, 20] = value

        detections = decode_yolov5_heads(outputs, 640, 0.55, 0.45)
        self.assertEqual(len(detections), 1)
        self.assertEqual(detections[0].state, "left")
        self.assertAlmostEqual(detections[0].confidence, 0.81, places=5)

    def test_rejects_wrong_head_count(self):
        with self.assertRaises(ValueError):
            decode_yolov5_heads([], 640, 0.55, 0.45)

    def test_class_aware_nms_keeps_different_classes(self):
        boxes = np.array([[0, 0, 10, 10], [1, 1, 11, 11], [1, 1, 11, 11]], dtype=np.float32)
        scores = np.array([0.9, 0.8, 0.7], dtype=np.float32)
        class_ids = np.array([0, 0, 1], dtype=np.int32)
        retained = class_aware_nms(boxes, scores, class_ids, 0.45)
        self.assertEqual(retained, [0, 2])


class TemporalVoterTest(unittest.TestCase):
    def test_direction_needs_five_votes_and_average_confidence(self):
        voter = TemporalVoter(7, 5, 0.65, 2)
        for _ in range(4):
            self.assertEqual(voter.update("left", 0.9), "unknown")
        self.assertEqual(voter.update("left", 0.9), "left")

    def test_low_confidence_direction_remains_unknown(self):
        voter = TemporalVoter(7, 5, 0.65, 2)
        state = "unknown"
        for _ in range(7):
            state = voter.update("right", 0.6)
        self.assertEqual(state, "unknown")

    def test_two_consecutive_red_frames_stop(self):
        voter = TemporalVoter(7, 5, 0.65, 2)
        self.assertEqual(voter.update("red", 0.8), "unknown")
        self.assertEqual(voter.update("red", 0.8), "red")

    def test_red_has_primary_detection_priority(self):
        detections = [
            Detection(2, "left", 0.95, (0, 0, 10, 10)),
            Detection(0, "red", 0.60, (0, 0, 10, 10)),
        ]
        self.assertEqual(select_primary_detection(detections).state, "red")


if __name__ == "__main__":
    unittest.main()

