#!/home/ucar/venv3.9/bin/python
# -*- coding: utf-8 -*-

"""RKNN 1.4-compatible src2 factory-sign OCR for the real vehicle.

The PP-OCRv4 models copied from src2 were compiled by RKNN Toolkit 2.3.2,
while the competition vehicle runs librknnrt 1.4.0.  This node therefore
uses the vehicle's proven 1.4.0 PP-OCR detector/recognizer to locate and read
the physical sign, then requires agreement from src2's 1.4.0 RKNN factory
sign classifier.  Only complete category words are accepted.  Results use
the src2 PP-OCR JSON protocol so the existing temporal adapter and parking
manager remain unchanged.
"""

import json
import logging
import os
import sys
import threading
import time

import cv2
import numpy as np


_ORIGINAL_CHECK_LEVEL = logging._checkLevel
_TEXT_LOG_LEVELS = {
    "NOTSET": logging.NOTSET,
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARN": logging.WARNING,
    "WARNING": logging.WARNING,
    "ERROR": logging.ERROR,
    "FATAL": logging.CRITICAL,
    "CRITICAL": logging.CRITICAL,
}


def _ros_compatible_check_level(level):
    if isinstance(level, str):
        normalized = level.strip().upper()
        if normalized in _TEXT_LOG_LEVELS:
            return _TEXT_LOG_LEVELS[normalized]
    return _ORIGINAL_CHECK_LEVEL(level)


logging._checkLevel = _ros_compatible_check_level
for _path in ("/usr/lib/python3/dist-packages",
              "/opt/ros/noetic/lib/python3/dist-packages"):
    if _path not in sys.path:
        sys.path.append(_path)

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
OCR_DIR = os.path.join(PACKAGE_DIR, "factory_ocr_car_deploy")
SRC2_CLASSIFIER_MODEL = os.path.join(
    OCR_DIR, "factory_sign_cls_src2_rk3588.rknn")
if OCR_DIR not in sys.path:
    sys.path.insert(0, OCR_DIR)
os.chdir(OCR_DIR)

import camera_det_rec_final_working as ocr_impl  # noqa: E402


for _level, _name in (
        (logging.DEBUG, "DEBUG"),
        (logging.INFO, "INFO"),
        (logging.WARNING, "WARNING"),
        (logging.ERROR, "ERROR"),
        (logging.CRITICAL, "CRITICAL")):
    logging.addLevelName(_level, _name)

try:
    import rosgraph.roslogging as _roslogging
    _roslogging._logging_to_rospy_names.update({
        "D": ("DEBUG", "\033[32m"),
        "I": ("INFO", None),
        "W": ("WARN", "\033[33m"),
        "E": ("ERROR", "\033[31m"),
        "F": ("FATAL", "\033[31m"),
    })
except Exception:
    pass


CATEGORIES = ("daily", "electronic", "food")
WORKSHOPS = {
    "food": "食品加工车间",
    "daily": "日用品加工车间",
    "electronic": "电子产品生产车间",
}
WORKSHOP_CATEGORIES = dict((value, key) for key, value in WORKSHOPS.items())


def reshape_raw_image(msg, data, channels):
    height = int(msg.height)
    width = int(msg.width)
    expected = height * width * channels
    if expected == data.size:
        shape = ((height, width, channels) if channels > 1
                 else (height, width))
        return data.reshape(shape)
    pixels = data.size // channels
    if pixels * channels != data.size:
        raise ValueError("image byte count is not divisible by channels")
    for actual_width, actual_height in (
            (800, 600), (640, 480), (1280, 720), (1280, 960),
            (1920, 1080), (320, 240)):
        if actual_width * actual_height == pixels:
            shape = ((actual_height, actual_width, channels)
                     if channels > 1 else (actual_height, actual_width))
            return data.reshape(shape)
    raise ValueError("cannot infer image shape")


def decode_image(msg):
    encoding = (msg.encoding or "").lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if encoding in ("rgb8", "bgr8"):
        image = reshape_raw_image(msg, data, 3)
        if encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(image)
    if encoding in ("rgba8", "bgra8"):
        image = reshape_raw_image(msg, data, 4)
        code = (cv2.COLOR_RGBA2BGR if encoding == "rgba8"
                else cv2.COLOR_BGRA2BGR)
        return cv2.cvtColor(image, code)
    if encoding in ("mono8", "8uc1"):
        gray = reshape_raw_image(msg, data, 1)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return None


def complete_text_category(raw_text):
    """Reject the old recognizer's dangerous one-character food/electric hits."""
    text = str(raw_text or "").replace(" ", "").replace("\n", "")
    found = []
    if "食品" in text:
        found.append("food")
    if "日用品" in text or "日用" in text:
        found.append("daily")
    if "电子" in text:
        found.append("electronic")
    return found[0] if len(set(found)) == 1 else ""


def axis_points(best):
    if not best or best.get("box") is None:
        return None
    points = np.asarray(best["box"], dtype=np.float32).reshape((-1, 2))
    if points.shape[0] < 4:
        return None
    x0, x1 = float(points[:, 0].min()), float(points[:, 0].max())
    y0, y1 = float(points[:, 1].min()), float(points[:, 1].max())
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


class Src2FactoryOCRCompatV2(object):
    def __init__(self):
        rospy.init_node("xunfei2026_src2_factory_ocr_compat_v2")
        self.image_topic = rospy.get_param(
            "~image_topic", "/ucar_camera/image_raw")
        self.result_topic = rospy.get_param(
            "~result_topic", "/factory_room/src2_ocr_raw")
        self.control_topic = rospy.get_param(
            "~control_topic", "/factory_room/ocr_control")
        self.health_topic = rospy.get_param(
            "~health_topic", "/factory_room/src2_compat_health")
        self.debug_topic = rospy.get_param(
            "~debug_topic", "/factory_room/ocr_debug")
        self.status_topic = rospy.get_param(
            "~status_topic", "/factory_room/xunfei2026_delivery_status")
        self.process_rate_hz = float(rospy.get_param("~process_rate_hz", 4.0))
        self.max_width = int(rospy.get_param("~max_width", 960))
        self.enabled = bool(rospy.get_param("~enabled_on_start", True))
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))
        self.classifier_model = str(rospy.get_param(
            "~classifier_model_path", SRC2_CLASSIFIER_MODEL))
        self.classifier_confidence = float(rospy.get_param(
            "~classifier_confidence_threshold", 0.55))
        self.classifier_min_margin = float(rospy.get_param(
            "~classifier_min_margin", 0.18))
        self.biases = np.asarray([
            float(rospy.get_param("~classifier_daily_logit_bias", 0.45)),
            float(rospy.get_param("~classifier_electronic_logit_bias", -0.60)),
            float(rospy.get_param("~classifier_food_logit_bias", 0.0)),
        ], dtype=np.float32)

        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_stamp = rospy.Time(0)
        self.last_processed_stamp = rospy.Time(0)
        self.det_rknn = None
        self.rec_rknn = None
        self.cls_rknn = None
        self.target_category = ""

        self.result_pub = rospy.Publisher(
            self.result_topic, String, queue_size=5, latch=True)
        self.health_pub = rospy.Publisher(
            self.health_topic, String, queue_size=2, latch=True)
        self.debug_pub = rospy.Publisher(
            self.debug_topic, Image, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_callback,
                         queue_size=1, buff_size=4 * 1024 * 1024)
        rospy.Subscriber(self.control_topic, String, self.control_callback,
                         queue_size=5)
        # The status topic is latched, so a late-started OCR process receives
        # the current real/simulation target immediately.
        rospy.Subscriber(self.status_topic, String, self.status_callback,
                         queue_size=10)

    def status_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        workshop = str(payload.get("warehouse", "") or "").strip()
        category = WORKSHOP_CATEGORIES.get(workshop, "")
        if category and category != self.target_category:
            self.target_category = category
            rospy.logwarn(
                "SRC2_FACTORY_OCR_TARGET category=%s workshop=%s",
                category, workshop)

    def image_callback(self, msg):
        try:
            image = decode_image(msg)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "SRC2_COMPAT image decode failed: %s", exc)
            return
        if image is None:
            return
        if self.max_width > 0 and image.shape[1] > self.max_width:
            scale = float(self.max_width) / float(image.shape[1])
            image = cv2.resize(
                image, (self.max_width, int(image.shape[0] * scale)),
                interpolation=cv2.INTER_AREA)
        with self.lock:
            self.latest_frame = image
            self.latest_stamp = (msg.header.stamp if msg.header.stamp
                                 else rospy.Time.now())

    def control_callback(self, msg):
        command = str(msg.data or "").strip().lower()
        with self.lock:
            if command in ("enable", "start", "scan", "on"):
                self.enabled = True
            elif command in ("disable", "stop", "off"):
                self.enabled = False
        rospy.loginfo("SRC2_COMPAT_CONTROL command=%s enabled=%s",
                      command, str(self.enabled))

    def initialize_models(self):
        self.health_pub.publish(String(data="loading"))
        self.det_rknn = ocr_impl.load_rknn(ocr_impl.DET_MODEL, "compat det")
        self.rec_rknn = ocr_impl.load_rknn(ocr_impl.REC_MODEL, "compat rec")
        self.cls_rknn = ocr_impl.load_rknn(
            self.classifier_model, "src2 factory classifier")
        self.health_pub.publish(String(data="ready"))
        rospy.logwarn(
            "SRC2_FACTORY_OCR_COMPAT_READY runtime=1.4 det_rec=compatible "
            "classifier=src2 consensus=complete_text image=%s result=%s",
            self.image_topic, self.result_topic)

    def snapshot(self):
        with self.lock:
            if (not self.enabled or self.latest_frame is None or
                    self.latest_stamp == self.last_processed_stamp):
                return None, None
            frame = self.latest_frame.copy()
            stamp = self.latest_stamp
            self.last_processed_stamp = stamp
        return frame, stamp

    def classify_src2(self, frame):
        height, width = frame.shape[:2]
        side = min(height, width)
        y0 = max(0, (height - side) // 2)
        x0 = max(0, (width - side) // 2)
        crop = frame[y0:y0 + side, x0:x0 + side]
        image = cv2.resize(crop, (224, 224), interpolation=cv2.INTER_LINEAR)
        image = np.expand_dims(image, axis=0).astype(np.uint8)
        outputs = self.cls_rknn.inference(
            inputs=[image], data_format=["nhwc"])
        if not outputs:
            return "", 0.0, 0.0, []
        raw = np.asarray(outputs[0], dtype=np.float32).reshape(-1)[:3]
        logits = raw + self.biases
        logits -= np.max(logits)
        exp = np.exp(logits)
        probs = exp / max(float(np.sum(exp)), 1e-9)
        order = np.argsort(probs)[::-1]
        top, second = int(order[0]), int(order[1])
        confidence = float(probs[top])
        margin = confidence - float(probs[second])
        category = CATEGORIES[top]
        if (confidence < self.classifier_confidence or
                margin < self.classifier_min_margin):
            category = ""
        return category, confidence, margin, probs.tolist()

    def publish_debug_image(self, frame, det_items, best, text_category,
                            src2_category, consensus, confidence, margin):
        if not self.publish_debug:
            return
        debug = frame.copy()
        for item in det_items:
            ocr_impl.draw_poly(debug, item["box"], (0, 180, 0), 1)
        if best is not None:
            color = (0, 0, 255) if consensus else (0, 165, 255)
            ocr_impl.draw_poly(debug, best["box"], color, 3)
        text = "text=%s src2=%s agree=%s p=%.2f m=%.2f" % (
            text_category or "none", src2_category or "none",
            consensus or "none", confidence, margin)
        cv2.putText(debug, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 255), 2)
        rgb = cv2.cvtColor(debug, cv2.COLOR_BGR2RGB)
        out = Image()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = "src2_factory_ocr_compat"
        out.height, out.width = rgb.shape[:2]
        out.encoding = "rgb8"
        out.is_bigendian = False
        out.step = out.width * 3
        out.data = rgb.tobytes()
        self.debug_pub.publish(out)

    def process_once(self):
        frame, stamp = self.snapshot()
        if frame is None:
            return
        try:
            det_items, layout, det_max, _ = ocr_impl.run_det(
                self.det_rknn, frame)
            crops, rejected = ocr_impl.build_good_crops(frame, det_items)
            results = ocr_impl.recognize_crops(self.rec_rknn, crops)
            old_label, raw_text, old_score, best = ocr_impl.decide_frame(results)
            text_category = complete_text_category(raw_text)
            src2_category, confidence, margin, probabilities = (
                self.classify_src2(frame))
            old_category = {
                "食品加工车间": "food",
                "日用品加工车间": "daily",
                "电子产品生产车间": "electronic",
            }.get(old_label, "")
            consensus = (src2_category if src2_category and
                         src2_category == old_category == text_category else "")
            # A complete physical OCR reading of the *requested* workshop is
            # strong enough to stop for confirmation even when the src2
            # whole-frame classifier disagrees.  It is deliberately not final
            # evidence: the adapter/manager asks for another fresh frame while
            # stopped before setting target_event.  Non-target OCR can never
            # enter this fallback, preventing the repeated food hallucination
            # seen during an electronic mission from stopping the vehicle.
            target_candidate = bool(
                not consensus and self.target_category and
                old_category == text_category == self.target_category and
                best is not None)
            published_category = (
                consensus or (self.target_category if target_candidate else ""))
            target_bbox = axis_points(best) if published_category else None
            payload = {
                "category": published_category or None,
                "workshop": WORKSHOPS.get(published_category, ""),
                "confidence": confidence,
                "category_score": confidence,
                "evidence": (
                    "src2_classifier+complete_text+physical_bbox"
                    if consensus else
                    "requested_complete_text_candidate"),
                "candidate_only": target_candidate,
                "target_category": self.target_category,
                "raw_text": raw_text,
                "target_bbox": target_bbox,
                "image_width": int(frame.shape[1]),
                "image_height": int(frame.shape[0]),
                "candidate_count": len(crops),
                "det_count": len(det_items),
                "det_layout": layout,
                "det_score_max": float(det_max),
                "old_ocr_category": old_category,
                "complete_text_category": text_category,
                "src2_category": src2_category,
                "src2_probabilities": probabilities,
                "src2_margin": margin,
                "old_ocr_score": float(old_score),
                "image_stamp": stamp.to_sec(),
                "stamp": time.time(),
                "error": None,
            }
            self.result_pub.publish(String(
                data=json.dumps(payload, ensure_ascii=False)))
            self.publish_debug_image(
                frame, det_items, best, text_category, src2_category,
                consensus, confidence, margin)
            if consensus:
                rospy.logwarn_throttle(
                    0.5, "SRC2_FACTORY_OCR_CONSENSUS category=%s raw=%s "
                    "confidence=%.3f margin=%.3f bbox=%s",
                    consensus, raw_text, confidence, margin, str(target_bbox))
            elif target_candidate:
                rospy.logwarn_throttle(
                    0.3, "SRC2_FACTORY_OCR_TARGET_SUSPECT target=%s raw=%s "
                    "src2=%s confidence=%.3f bbox=%s stop_to_confirm=true",
                    self.target_category, raw_text,
                    src2_category or "none", confidence, str(target_bbox))
            else:
                rospy.loginfo_throttle(
                    1.0, "SRC2_FACTORY_OCR_REJECT old=%s complete=%s src2=%s "
                    "raw=%s confidence=%.3f margin=%.3f det=%d crops=%d",
                    old_category or "none", text_category or "none",
                    src2_category or "none", raw_text or "none",
                    confidence, margin, len(det_items), len(crops))
        except Exception as exc:
            payload = {"category": None, "target_bbox": None,
                       "error": str(exc), "stamp": time.time()}
            self.result_pub.publish(String(
                data=json.dumps(payload, ensure_ascii=False)))
            self.health_pub.publish(String(data="runtime_error: " + str(exc)))
            rospy.logerr_throttle(1.0, "SRC2_COMPAT frame failed: %s", exc)

    def shutdown(self):
        for runtime in (self.det_rknn, self.rec_rknn, self.cls_rknn):
            if runtime is not None:
                try:
                    runtime.release()
                except Exception:
                    pass

    def run(self):
        self.initialize_models()
        rospy.on_shutdown(self.shutdown)
        rate = rospy.Rate(max(0.5, self.process_rate_hz))
        while not rospy.is_shutdown():
            self.process_once()
            rate.sleep()


if __name__ == "__main__":
    try:
        Src2FactoryOCRCompatV2().run()
    except rospy.ROSInterruptException:
        pass
