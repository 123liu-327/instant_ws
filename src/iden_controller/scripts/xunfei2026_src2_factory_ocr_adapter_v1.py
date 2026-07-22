#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Adapt src2 PP-OCR RKNN results to the existing room-delivery protocol."""

import json
import time

import rospy
from std_msgs.msg import String


WORKSHOPS = {
    "food": "食品加工车间",
    "daily": "日用品加工车间",
    "electronic": "电子产品生产车间",
}


def normalize_category(value):
    text = str(value or "").strip().lower()
    if text in ("food", "食品", "食品加工车间"):
        return "food"
    if text in ("daily", "日用", "日用品", "日用品加工车间"):
        return "daily"
    if text in (
            "electronic", "electronics", "电子", "电子产品",
            "电子产品加工车间", "电子产品生产车间"):
        return "electronic"
    return ""


def axis_aligned_bbox(points):
    if not isinstance(points, (list, tuple)) or len(points) != 4:
        return None
    try:
        xs = [float(point[0]) for point in points]
        ys = [float(point[1]) for point in points]
    except (TypeError, ValueError, IndexError):
        return None
    return [min(xs), min(ys), max(xs), max(ys)]


class TemporalCategoryEvidence(object):
    """src2 target filter: repeated evidence, blanks keep, rivals reset.

    NOTE: the room-delivery manager (v6) runs an equivalent filter with
    the same parameters via ``_src2_push_category``.  This adapter-level
    filter is retained for backward compatibility and for consumers that
    read the ``stable`` / ``votes`` fields directly.  The authoritative
    target-lock decision is made inside the room manager.
    """

    def __init__(self, required=2, window_s=1.5):
        self.required = max(1, int(required))
        self.window_s = max(0.05, float(window_s))
        self.category = ""
        self.hits = []

    def reset(self):
        self.category = ""
        self.hits = []

    def push(self, category, now):
        self.hits = [stamp for stamp in self.hits
                     if now - stamp <= self.window_s]
        if not category:
            return False, len(self.hits)
        if category != self.category:
            self.category = category
            self.hits = []
        self.hits.append(now)
        return len(self.hits) >= self.required, len(self.hits)


class Src2FactoryOCRAdapter(object):
    def __init__(self):
        rospy.init_node("xunfei2026_src2_factory_ocr_adapter")
        self.raw_topic = rospy.get_param(
            "~raw_topic", "/factory_room/src2_ocr_raw")
        self.result_topic = rospy.get_param(
            "~result_topic", "/factory_room/ocr_result")
        self.control_topic = rospy.get_param(
            "~control_topic", "/factory_room/ocr_control")
        self.health_topic = rospy.get_param(
            "~health_topic", "/factory_room/ocr_health")
        self.enabled = bool(rospy.get_param("~enabled_on_start", False))
        self.filter = TemporalCategoryEvidence(
            rospy.get_param("~required_hits", 2),
            rospy.get_param("~evidence_window_s", 1.5))
        self.ready = False

        self.result_pub = rospy.Publisher(
            self.result_topic, String, queue_size=10, latch=True)
        self.health_pub = rospy.Publisher(
            self.health_topic, String, queue_size=2, latch=True)
        self.health_pub.publish(String(data="loading"))
        rospy.Subscriber(
            self.raw_topic, String, self.raw_callback, queue_size=10)
        rospy.Subscriber(
            self.control_topic, String, self.control_callback, queue_size=5)
        rospy.logwarn(
            "SRC2_FACTORY_OCR_ADAPTER loading raw=%s result=%s hits=%d/%.1fs",
            self.raw_topic, self.result_topic,
            self.filter.required, self.filter.window_s)

    def control_callback(self, message):
        command = str(message.data or "").strip().lower()
        if command in ("enable", "start", "scan", "on"):
            self.enabled = True
            self.filter.reset()
        elif command in ("disable", "stop", "off"):
            self.enabled = False
            self.filter.reset()
        elif command in ("reset", "clear"):
            self.filter.reset()
        rospy.loginfo("SRC2_FACTORY_OCR_CONTROL command=%s enabled=%s",
                      command, str(self.enabled))

    def raw_callback(self, message):
        try:
            source = json.loads(message.data)
        except Exception as exc:
            rospy.logwarn_throttle(
                1.0, "SRC2_FACTORY_OCR_BAD_JSON error=%s", exc)
            return

        error = str(source.get("error", "") or "").strip()
        if error:
            self.health_pub.publish(String(data="runtime_error: " + error))
            rospy.logerr_throttle(
                1.0, "SRC2_FACTORY_OCR_RUNTIME_ERROR %s", error)
            return
        if not self.ready:
            self.ready = True
            self.health_pub.publish(String(data="ready"))
            rospy.logwarn("SRC2_FACTORY_OCR_READY source=ppocr_rknn_system")
        if not self.enabled:
            return

        now = time.monotonic()
        category = normalize_category(source.get("category", ""))
        bbox = axis_aligned_bbox(source.get("target_bbox"))
        # A category without its independently supporting src2 text box is not
        # physical sign evidence and must not enter the temporal filter.
        observed = category if bbox is not None else ""
        stable, native_votes = self.filter.push(observed, now)
        votes = native_votes
        candidate_only = bool(source.get("candidate_only", False))
        # Forward every native src2 frame exactly once.  Target confirmation
        # belongs to src2's temporal filter in the room controller; duplicating
        # a candidate here would turn one camera frame into two observations.
        workshop = WORKSHOPS.get(observed, "")
        payload = {
            "label": workshop,
            "frame_label": workshop,
            "raw_text": str(source.get("raw_text", "") or ""),
            "score": float(source.get(
                "category_score", source.get("confidence", 0.0)) or 0.0),
            "stable": bool(stable and observed),
            "votes": int(votes),
            "vote_window": int(votes),
            "src2_native_votes": int(native_votes),
            "bbox": bbox,
            "image_width": int(source.get("image_width", 0) or 0),
            "image_height": int(source.get("image_height", 0) or 0),
            # src3 fields forwarded directly from the PP-OCR RKNN node:
            "view_scale": float(source.get("view_scale", 1.0) or 1.0),
            "match_debug": str(source.get("match_debug", "") or ""),
            "target_center_x": (None if source.get("target_center_x") is None
                                else float(source["target_center_x"])),
            "target_center_y": (None if source.get("target_center_y") is None
                                else float(source["target_center_y"])),
            "stamp": float(source.get("stamp", time.time()) or time.time()),
            "source": "src2_ppocr_rknn_system",
            "src2_category": observed,
            "src2_evidence": str(source.get("evidence", "") or ""),
            "src2_confidence": float(source.get("confidence", 0.0) or 0.0),
            "src2_candidate_count": int(source.get("candidate_count", 0) or 0),
            "candidate_only": candidate_only,
        }
        encoded = String(data=json.dumps(payload, ensure_ascii=False))
        self.result_pub.publish(encoded)
        rospy.loginfo_throttle(
            0.5,
            "SRC2_FACTORY_OCR category=%s raw=%s bbox=%s hits=%d/%d stable=%s",
            observed or "unknown", payload["raw_text"], str(bbox), native_votes,
            self.filter.required, str(payload["stable"]))


if __name__ == "__main__":
    Src2FactoryOCRAdapter()
    rospy.spin()
