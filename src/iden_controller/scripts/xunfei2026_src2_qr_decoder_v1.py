#!/usr/bin/python3
# -*- coding: utf-8 -*-

"""Exact src2 QR image/decode core with a current-flow ROS output topic."""

import json
import math
import threading
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from pyzbar import pyzbar
from pyzbar.pyzbar import ZBarSymbol
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Src2QRDecoder(object):
    def __init__(self):
        self.image_topic = rospy.get_param(
            "~image_topic", "/ucar_camera/image_raw")
        self.result_topic = rospy.get_param(
            "~result_topic", "/xunfei2026/src2_qr_raw")
        self.candidate_topic = rospy.get_param(
            "~candidate_topic", "/xunfei2026/src2_qr_candidate")
        self.candidate_detection_enabled = bool(rospy.get_param(
            "~candidate_detection_enabled", False))
        self.decode_interval = max(
            0.0, float(rospy.get_param("~decode_interval_s", 0.10)))
        scale_text = str(rospy.get_param(
            "~decode_scales", "1.0,1.5,2.0"))
        self.decode_scales = self.parse_scales(scale_text)
        self.bridge = CvBridge()
        detector_type = getattr(cv2, "QRCodeDetector", None)
        self.detector = detector_type() if detector_type is not None else None
        self.clahe = cv2.createCLAHE(
            clipLimit=2.0, tileGridSize=(8, 8))
        self.lock = threading.Lock()
        self.last_decode_at = 0.0
        self.last_payload = ""
        self.last_publish_at = 0.0
        self.last_candidate_publish_at = 0.0
        self.publisher = rospy.Publisher(
            self.result_topic, String, queue_size=10)
        self.candidate_publisher = rospy.Publisher(
            self.candidate_topic, String, queue_size=5)
        rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1,
            buff_size=2 ** 24)
        rospy.logwarn(
            "SRC2_QR_DECODER_READY image=%s result=%s interval=%.2fs "
            "scales=%s opencv_qr=%s pyzbar=on",
            self.image_topic, self.result_topic, self.decode_interval,
            self.decode_scales, "on" if self.detector is not None else "off")

    @staticmethod
    def _finder_candidates(gray):
        """Locate nested QR finder patterns even when payload decode fails."""
        height, width = gray.shape[:2]
        if width > 640:
            ratio = 640.0 / float(width)
            work = cv2.resize(
                gray, (640, max(1, int(round(height * ratio)))),
                interpolation=cv2.INTER_AREA)
        else:
            ratio = 1.0
            work = gray
        blurred = cv2.GaussianBlur(work, (3, 3), 0)
        _value, binary = cv2.threshold(
            blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        found = cv2.findContours(
            binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours, hierarchy = found[-2], found[-1]
        if hierarchy is None:
            return []
        hierarchy = hierarchy[0]
        image_area = float(work.shape[0] * work.shape[1])
        candidates = []
        for index, contour in enumerate(contours):
            area = abs(float(cv2.contourArea(contour)))
            if area < 18.0 or area > image_area * 0.12:
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            if box_width < 5 or box_height < 5:
                continue
            aspect = float(box_width) / float(box_height)
            if not 0.58 <= aspect <= 1.72:
                continue
            depth = 0
            child = int(hierarchy[index][2])
            while child >= 0 and depth < 8:
                depth += 1
                child = int(hierarchy[child][2])
            if depth < 3:
                continue
            center_x = x + 0.5 * box_width
            center_y = y + 0.5 * box_height
            # Nested contours of one finder produce the same center. Keep the
            # strongest/largest representative only.
            duplicate = None
            for candidate in candidates:
                tolerance = max(5.0, 0.22 * max(box_width, box_height))
                if math.hypot(
                        center_x - candidate["cx"],
                        center_y - candidate["cy"]) <= tolerance:
                    duplicate = candidate
                    break
            record = {
                "cx": center_x, "cy": center_y, "x": x, "y": y,
                "w": box_width, "h": box_height, "depth": depth,
                "area": area,
            }
            if duplicate is None:
                candidates.append(record)
            elif (depth, area) > (duplicate["depth"], duplicate["area"]):
                duplicate.update(record)
        scale_back = 1.0 / ratio
        for candidate in candidates:
            for key in ("cx", "cy", "x", "y", "w", "h"):
                candidate[key] *= scale_back
        return candidates

    def publish_candidate(self, gray):
        candidates = self._finder_candidates(gray)
        if not candidates:
            return
        # Three finder patterns are ideal. A deeply nested partial finder is
        # still useful, but the motion manager requires temporal stability.
        strong = [item for item in candidates if item["depth"] >= 4]
        selected = strong if strong else candidates
        if len(selected) < 2 and selected[0]["depth"] < 5:
            return
        x1 = min(item["x"] for item in selected)
        y1 = min(item["y"] for item in selected)
        x2 = max(item["x"] + item["w"] for item in selected)
        y2 = max(item["y"] + item["h"] for item in selected)
        height, width = gray.shape[:2]
        center_x = 0.5 * (x1 + x2)
        center_y = 0.5 * (y1 + y2)
        now = time.monotonic()
        if now - self.last_candidate_publish_at < 0.16:
            return
        self.last_candidate_publish_at = now
        payload = {
            "stamp": rospy.Time.now().to_sec(),
            "finder_count": len(selected),
            "x_norm": center_x / max(1.0, float(width)),
            "y_norm": center_y / max(1.0, float(height)),
            "bbox": [x1, y1, x2, y2],
            "area_ratio": ((x2 - x1) * (y2 - y1) /
                           max(1.0, float(width * height))),
        }
        self.candidate_publisher.publish(String(data=json.dumps(payload)))

    @staticmethod
    def parse_scales(text):
        values = []
        for token in text.split(","):
            try:
                value = float(token.strip())
            except Exception:
                continue
            if value >= 1.0 and value not in values:
                values.append(value)
        if 1.0 not in values:
            values.insert(0, 1.0)
        return sorted(values)

    @staticmethod
    def append(values, text):
        text = str(text or "").strip()
        if text and text not in values:
            values.append(text)

    def decode_opencv(self, image, values):
        if self.detector is None:
            return
        try:
            ok, decoded, _points, _straight = (
                self.detector.detectAndDecodeMulti(image))
            if ok and decoded:
                for text in decoded:
                    self.append(values, text)
        except Exception:
            pass

    def decode_pyzbar(self, image, values):
        try:
            for code in pyzbar.decode(
                    image, symbols=[ZBarSymbol.QRCODE]):
                self.append(
                    values,
                    code.data.decode("utf-8", errors="ignore"))
        except Exception:
            pass

    def decode_qr(self, bgr):
        # This ordering intentionally matches src2/yolo/qr_collect_and_decode.py.
        values = []
        self.decode_opencv(bgr, values)
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        self.decode_pyzbar(gray, values)
        if values:
            return values
        enhanced = self.clahe.apply(gray)
        blurred = cv2.GaussianBlur(enhanced, (0, 0), 1.0)
        sharpened = cv2.addWeighted(enhanced, 1.6, blurred, -0.6, 0)
        for scale in self.decode_scales:
            candidate = sharpened
            if abs(scale - 1.0) > 1.0e-6:
                candidate = cv2.resize(
                    sharpened, None, fx=scale, fy=scale,
                    interpolation=cv2.INTER_CUBIC)
            self.decode_opencv(candidate, values)
            self.decode_pyzbar(candidate, values)
        return values

    def image_callback(self, message):
        now = time.monotonic()
        with self.lock:
            if now - self.last_decode_at < self.decode_interval:
                return
            self.last_decode_at = now
        try:
            bgr = self.bridge.imgmsg_to_cv2(message, "bgr8")
            values = self.decode_qr(bgr)
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0, "SRC2_QR_FRAME_ERROR %s", str(exc))
            return
        if not values:
            if self.candidate_detection_enabled:
                try:
                    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                    self.publish_candidate(gray)
                except Exception:
                    pass
            return
        payload = json.dumps(
            {"stamp": rospy.Time.now().to_sec(), "count": len(values),
             "items": [{"raw": value} for value in values]},
            ensure_ascii=False)
        wall_now = time.monotonic()
        if payload == self.last_payload and wall_now - self.last_publish_at < 1.0:
            return
        self.last_payload = payload
        self.last_publish_at = wall_now
        self.publisher.publish(String(data=payload))
        rospy.logwarn("SRC2_QR_DECODED count=%d", len(values))


if __name__ == "__main__":
    rospy.init_node("xunfei2026_src2_qr_decoder")
    Src2QRDecoder()
    rospy.spin()
