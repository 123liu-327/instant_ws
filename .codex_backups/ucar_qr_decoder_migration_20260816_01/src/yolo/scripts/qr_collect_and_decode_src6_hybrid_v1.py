#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
QR collector and decoder for ROS U-CAR competition.
- Subscribe /usb_cam/image_raw
- Save QR test images at fixed interval
- Decode one or multiple QR codes in camera image
- If QR content is URL, request it and parse returned JSON: {"code":200,"result":"xx"}
- Publish decoded result to /qr_code_results as std_msgs/String JSON
"""
import os
import time
import json
import argparse
import sys
import threading
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from urllib.parse import urlparse

import cv2
import rospy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import requests
except Exception:
    requests = None

try:
    from pyzbar import pyzbar
    from pyzbar.pyzbar import ZBarSymbol
except Exception:
    pyzbar = None
    ZBarSymbol = None


OFFLINE_ITEMS = {
    "food": ["苹果", "猪肉", "草莓", "饺子", "面条", "薯片", "馒头"],
    "daily": ["纸巾", "毛巾", "牙刷", "洗衣液", "T恤衫"],
    "electronic": ["手机", "耳机", "充电器", "鼠标", "数据线"],
}


class QRCollectAndDecode:
    def __init__(self, args):
        self.args = args
        self.bridge = CvBridge()
        self.detector = None
        try:
            self.detector = cv2.QRCodeDetector()
        except AttributeError:
            rospy.logwarn("cv2.QRCodeDetector not available (need opencv-contrib). "
                          "Will rely on pyzbar for QR decoding.")
        self.out_dir = os.path.expanduser(args.output)
        os.makedirs(self.out_dir, exist_ok=True)
        self.save_keyframes = str(args.save_keyframes).strip().lower() == "true"
        self.keyframe_output = os.path.abspath(os.path.expanduser(
            args.keyframe_output))
        if self.save_keyframes:
            os.makedirs(self.keyframe_output, exist_ok=True)
        self.last_save_time = 0.0
        self.save_count = 0
        self.last_publish_text = ""
        self.last_publish_time = 0.0
        self.last_decode_time = 0.0
        self.url_cache = {}
        self.url_lock = threading.RLock()
        self.publish_lock = threading.Lock()
        self.pending_urls = set()
        self.url_next_allowed = {}
        self.fetch_executor = ThreadPoolExecutor(max_workers=3)
        self.decode_interval = max(0.0, float(args.decode_interval))
        self.decode_scales = self.parse_decode_scales(args.decode_scales)
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        # The camera callback only keeps the newest image and, while the base is
        # confirmed stationary, collects a small bounded burst.  Heavy QR work
        # runs on one worker with one replaceable pending job, so stale frames
        # can never build an unbounded queue.
        self.state_lock = threading.RLock()
        self.decode_condition = threading.Condition(self.state_lock)
        self.shutdown_event = threading.Event()
        self.latest_msg = None
        self.motion_active = False
        self.odom_received_at = 0.0
        self.slow_motion_since = None
        self.stable_since = None
        self.stop_armed = True
        self.capture_active = False
        self.capture_deadline = 0.0
        self.capture_frames = []
        self.capture_previous_gray = None
        self.pending_decode_job = None
        self.decode_inflight = False
        self.last_moving_submit_time = 0.0
        self.keyframe_sequence = 0
        self.sample_count = max(1, int(args.keyframe_sample_count))
        self.sample_window = max(0.05, float(args.keyframe_sample_window))
        self.stationary_hold = max(0.05, float(args.stationary_hold))
        self.odom_stale_sec = max(0.05, float(args.odom_stale_sec))
        self.stationary_linear_tolerance = max(
            0.0, float(args.stationary_linear_tolerance))
        self.stationary_angular_tolerance = max(
            0.0, float(args.stationary_angular_tolerance))
        self.moving_decode_interval = max(
            0.10, float(args.moving_decode_interval))
        self.moving_decode_max_angular_speed = max(
            0.0, float(args.moving_decode_max_angular_speed))
        self.moving_decode_arm_sec = max(
            0.0, float(args.moving_decode_arm_sec))

        self.pub = rospy.Publisher(args.pub_topic, String, queue_size=10)
        self.status_pub = rospy.Publisher(
            args.status_topic, String, queue_size=10, latch=True)
        self.sub = rospy.Subscriber(args.topic, Image, self.image_cb, queue_size=1)
        self.odom_sub = rospy.Subscriber(
            args.odom_topic, Odometry, self.odom_cb, queue_size=10)
        self.decode_thread = threading.Thread(
            target=self.decode_worker, name="qr_decode_worker")
        self.decode_thread.daemon = True
        self.decode_thread.start()
        self.selector_thread = threading.Thread(
            target=self.keyframe_selector_worker, name="qr_keyframe_selector")
        self.selector_thread.daemon = True
        self.selector_thread.start()
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("QR image topic: %s", args.topic)
        rospy.loginfo("QR images save to: %s", self.out_dir)
        rospy.loginfo("QR result topic: %s", args.pub_topic)
        rospy.loginfo(
            "QR decode interval: %.3fs, enhanced scales: %s",
            self.decode_interval,
            ",".join("{:.2f}".format(value) for value in self.decode_scales),
        )
        rospy.loginfo(
            "QR keyframe mode: odom=%s sample=%d/%.3fs stationary=%.3fs; "
            "whole-frame decode only; save=%s output=%s",
            args.odom_topic,
            self.sample_count,
            self.sample_window,
            self.stationary_hold,
            self.save_keyframes,
            self.keyframe_output,
        )
        self.publish_decoder_status("ready")

    def shutdown(self):
        if self.shutdown_event.is_set():
            return
        self.shutdown_event.set()
        with self.decode_condition:
            self.decode_condition.notify_all()
        self.fetch_executor.shutdown(wait=False)

    def publish_decoder_status(self, state):
        with self.url_lock:
            pending_count = len(self.pending_urls)
        payload = {
            "stamp": time.time(),
            "state": str(state),
            "pending_count": pending_count,
        }
        self.status_pub.publish(String(
            data=json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))

    @staticmethod
    def parse_decode_scales(value):
        scales = []
        for token in str(value or "").split(','):
            try:
                scale = float(token.strip())
            except (TypeError, ValueError):
                continue
            if scale >= 1.0 and scale not in scales:
                scales.append(scale)
        if 1.0 not in scales:
            scales.insert(0, 1.0)
        return sorted(scales)

    def odom_cb(self, msg):
        twist = msg.twist.twist
        linear_speed = max(abs(float(twist.linear.x)), abs(float(twist.linear.y)))
        angular_speed = abs(float(twist.angular.z))
        moving = (
            linear_speed > self.stationary_linear_tolerance or
            angular_speed > self.stationary_angular_tolerance
        )
        slow_scan_motion = (
            linear_speed <= self.stationary_linear_tolerance and
            self.stationary_angular_tolerance < angular_speed <=
            self.moving_decode_max_angular_speed
        )
        now = time.monotonic()
        with self.state_lock:
            self.odom_received_at = now
            if moving:
                if not self.motion_active and slow_scan_motion:
                    self.slow_motion_since = now
                self.motion_active = True
                self.stable_since = None
                self.stop_armed = True
                self.capture_active = False
                self.capture_frames = []
                self.capture_previous_gray = None
                if not slow_scan_motion:
                    self.slow_motion_since = None
                elif self.slow_motion_since is None:
                    self.slow_motion_since = now
            else:
                if self.motion_active or self.stable_since is None:
                    self.stable_since = now
                self.motion_active = False
                self.slow_motion_since = None

    def image_cb(self, msg):
        """Keep the latest image; only convert frames inside a sample burst."""
        now_monotonic = time.monotonic()
        with self.state_lock:
            self.latest_msg = msg
            capture_active = self.capture_active
            odom_fresh = (
                self.odom_received_at > 0.0 and
                now_monotonic - self.odom_received_at <= self.odom_stale_sec
            )
            slow_motion_ready = (
                odom_fresh and
                self.motion_active and
                self.slow_motion_since is not None and
                now_monotonic - self.slow_motion_since >= self.moving_decode_arm_sec and
                now_monotonic - self.last_moving_submit_time >= self.moving_decode_interval
            )

        if not capture_active and not slow_motion_ready:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if self.args.flip:
                img = cv2.flip(img, 1)
        except Exception as e:
            rospy.logerr_throttle(2.0, "image conversion error: %s", str(e))
            return

        stamp = self.message_stamp(msg)
        if capture_active:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            with self.state_lock:
                previous_gray = self.capture_previous_gray
            score, metrics = self.frame_observability_score(
                img, gray, previous_gray)
            with self.state_lock:
                if (self.capture_active and
                        len(self.capture_frames) < self.sample_count):
                    self.capture_frames.append((score, stamp, img, metrics))
                    self.capture_previous_gray = gray

        if slow_motion_ready:
            with self.state_lock:
                if (self.motion_active and
                        now_monotonic - self.last_moving_submit_time >=
                        self.moving_decode_interval):
                    self.last_moving_submit_time = now_monotonic
                    self.submit_decode_job_locked(
                        img, stamp, "slow_motion_fallback", None)

    @staticmethod
    def message_stamp(msg):
        try:
            stamp = float(msg.header.stamp.to_sec())
            if stamp > 0.0:
                return stamp
        except Exception:
            pass
        return time.time()

    @staticmethod
    def frame_observability_score(image, gray, previous_gray=None):
        """Score a whole frame without locating or cropping a QR region."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        saturation = hsv[:, :, 1]
        value = hsv[:, :, 2]
        white_mask = (
            (value >= 155) & (value <= 248) & (saturation <= 80))
        white_ratio = float(white_mask.mean())
        dark_ratio = float((gray <= 85).mean())
        clipped_ratio = float(((gray <= 8) | (gray >= 252)).mean())
        contrast = min(1.0, float(gray.std()) / 64.0)
        sharpness_raw = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        sharpness = min(1.0, sharpness_raw / 350.0)
        white_visibility = min(1.0, white_ratio / 0.20)
        black_white_coexistence = min(
            white_visibility, min(1.0, dark_ratio / 0.08))
        exposure = max(0.0, 1.0 - clipped_ratio / 0.35)
        stability = 0.50
        frame_delta = None
        if previous_gray is not None and previous_gray.shape == gray.shape:
            frame_delta = float(cv2.absdiff(gray, previous_gray).mean())
            stability = max(0.0, 1.0 - frame_delta / 28.0)
        score = (
            0.25 * sharpness +
            0.20 * white_visibility +
            0.15 * black_white_coexistence +
            0.15 * exposure +
            0.15 * contrast +
            0.10 * stability
        )
        metrics = {
            "sharpness": sharpness_raw,
            "white_ratio": white_ratio,
            "dark_ratio": dark_ratio,
            "clipped_ratio": clipped_ratio,
            "contrast": contrast,
            "frame_delta": frame_delta,
        }
        return score, metrics

    def keyframe_selector_worker(self):
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            selected = None
            now = time.monotonic()
            with self.state_lock:
                odom_fresh = (
                    self.odom_received_at > 0.0 and
                    now - self.odom_received_at <= self.odom_stale_sec
                )
                if not odom_fresh and self.capture_active:
                    self.capture_active = False
                    self.capture_frames = []
                    self.capture_previous_gray = None
                if (odom_fresh and not self.motion_active and self.stop_armed and
                        self.stable_since is not None and
                        now - self.stable_since >= self.stationary_hold and
                        not self.capture_active):
                    self.capture_active = True
                    self.capture_deadline = now + self.sample_window
                    self.capture_frames = []
                    self.capture_previous_gray = None

                if (self.capture_active and
                        (len(self.capture_frames) >= self.sample_count or
                         now >= self.capture_deadline)):
                    if self.capture_frames:
                        selected = max(self.capture_frames, key=lambda item: item[0])
                    self.capture_active = False
                    self.capture_frames = []
                    self.capture_previous_gray = None
                    self.stop_armed = False

            if selected is not None:
                score, stamp, image, metrics = selected
                metrics = dict(metrics)
                metrics["score"] = score
                if self.args.white_contour_test:
                    self.run_white_contour_diagnostic(image)
                rospy.loginfo(
                    "QR_KEYFRAME_SELECTED score=%.3f sharpness=%.1f white=%.3f "
                    "dark=%.3f clipped=%.3f",
                    score,
                    metrics["sharpness"],
                    metrics["white_ratio"],
                    metrics["dark_ratio"],
                    metrics["clipped_ratio"],
                )
                self.submit_decode_job(image, stamp, "stationary_keyframe", metrics)
            self.shutdown_event.wait(0.01)

    def submit_decode_job(self, image, stamp, source, metrics):
        with self.decode_condition:
            self.submit_decode_job_locked(image, stamp, source, metrics)

    def submit_decode_job_locked(self, image, stamp, source, metrics):
        replaced = self.pending_decode_job is not None
        self.keyframe_sequence += 1
        self.pending_decode_job = {
            "sequence": self.keyframe_sequence,
            "image": image,
            "stamp": stamp,
            "source": source,
            "metrics": metrics,
        }
        self.decode_condition.notify()
        if replaced:
            rospy.loginfo("QR pending keyframe replaced by newer whole frame")

    def decode_worker(self):
        while not rospy.is_shutdown() and not self.shutdown_event.is_set():
            with self.decode_condition:
                while (self.pending_decode_job is None and
                       not self.shutdown_event.is_set() and
                       not rospy.is_shutdown()):
                    self.decode_condition.wait(0.20)
                if self.shutdown_event.is_set() or rospy.is_shutdown():
                    return
                job = self.pending_decode_job
                self.pending_decode_job = None
                self.decode_inflight = True
            try:
                wait_sec = self.decode_interval - (
                    time.monotonic() - self.last_decode_time)
                if wait_sec > 0.0:
                    self.shutdown_event.wait(wait_sec)
                self.last_decode_time = time.monotonic()
                started_at = time.monotonic()
                decoded_items = self.process_selected_frame(
                    job["image"], job["stamp"])
                elapsed_ms = (time.monotonic() - started_at) * 1000.0
                if (self.save_keyframes and
                        job["source"] == "stationary_keyframe"):
                    self.save_selected_keyframe(
                        job, elapsed_ms, decoded_items)
                rospy.loginfo(
                    "QR_DECODE_JOB sequence=%d source=%s elapsed_ms=%.1f",
                    job["sequence"],
                    job["source"],
                    elapsed_ms,
                )
            except Exception as exc:
                rospy.logerr("QR decode worker error: %s", exc)
            finally:
                with self.state_lock:
                    self.decode_inflight = False

    def process_selected_frame(self, img, stamp):
        decoded_items = self.decode_qr(img)
        if decoded_items:
            results = []
            for text in decoded_items:
                item = {
                    "raw": text,
                    "api": None,
                    "ok": False,
                    "result": None,
                    "error": None
                }
                if self.args.fetch and self.is_url(text):
                    self.schedule_url_resolution(text)
                    continue
                results.append(item)
            if results:
                self.publish_results(results, stamp)
            if self.args.save_on_detect:
                self.save_image(img, prefix="qr_detect")
        now = time.time()
        if self.args.save_all and now - self.last_save_time >= self.args.interval:
            self.save_image(img, prefix="qr_raw")
            self.last_save_time = now
        return decoded_items

    def save_selected_keyframe(self, job, decode_ms, decoded_items):
        metrics = dict(job.get("metrics") or {})
        score = float(metrics.get("score", 0.0))
        stamp_ms = int(float(job.get("stamp") or time.time()) * 1000.0)
        basename = "qr_keyframe_{:06d}_score_{:.3f}_{}".format(
            int(job["sequence"]), score, stamp_ms)
        image_path = os.path.join(self.keyframe_output, basename + ".jpg")
        json_path = os.path.join(self.keyframe_output, basename + ".json")
        if not cv2.imwrite(image_path, job["image"]):
            rospy.logwarn("failed to save selected QR keyframe: %s", image_path)
            return
        metadata = {
            "sequence": int(job["sequence"]),
            "stamp": float(job.get("stamp") or 0.0),
            "source": str(job.get("source") or ""),
            "image_path": image_path,
            "decode_ms": float(decode_ms),
            "decoded": list(decoded_items or []),
            "metrics": metrics,
        }
        try:
            with open(json_path, "w", encoding="utf-8") as handle:
                json.dump(metadata, handle, ensure_ascii=False, indent=2,
                          sort_keys=True)
                handle.write("\n")
        except Exception as exc:
            rospy.logwarn("failed to save keyframe metadata %s: %s", json_path, exc)
            return
        rospy.loginfo(
            "QR_KEYFRAME_SAVED image=%s metadata=%s", image_path, json_path)

    def run_white_contour_diagnostic(self, image):
        """Diagnostic only: never gates, crops, or changes the decode input."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, 155), (180, 80, 248))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contour_result = cv2.findContours(
            mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = contour_result[-2]
        image_area = float(image.shape[0] * image.shape[1])
        useful = 0
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if image_area > 0.0 and 0.002 <= area / image_area <= 0.50:
                useful += 1
        rospy.loginfo(
            "QR_WHITE_CONTOUR_TEST contours=%d useful=%d (diagnostic only)",
            len(contours), useful)

    def schedule_url_resolution(self, url):
        now = time.time()
        with self.url_lock:
            if url in self.pending_urls or now < self.url_next_allowed.get(url, 0.0):
                return
            self.pending_urls.add(url)
        self.publish_decoder_status("fetching")
        self.fetch_executor.submit(self.resolve_and_publish_url, url)

    def resolve_and_publish_url(self, url):
        started_at = time.monotonic()
        try:
            api_result = self.resolve_qr_url(url)
            item = {
                "raw": url,
                "api": api_result,
                "ok": bool(api_result and api_result.get("code") == 200),
                "result": api_result.get("result") if api_result else None,
                "error": None,
            }
            if api_result and api_result.get("code") != 200:
                item["error"] = "api_code_not_200"
            rospy.loginfo(
                "QR URL resolved in %.1fms: %s -> %s",
                (time.monotonic() - started_at) * 1000.0,
                url,
                item["result"],
            )
            self.publish_results([item], time.time())
        except Exception as exc:
            rospy.logwarn("QR URL resolution failed: %s -> %s", url, exc)
        finally:
            with self.url_lock:
                self.pending_urls.discard(url)
                self.url_next_allowed[url] = time.time() + self.args.repeat_period
                pending_count = len(self.pending_urls)
            self.publish_decoder_status(
                "fetching" if pending_count else "idle")

    def publish_results(self, results, stamp):
        payload = {"stamp": stamp, "count": len(results), "items": results}
        text_payload = json.dumps(payload, ensure_ascii=False)
        with self.publish_lock:
            rospy.loginfo("QR decoded: %s", text_payload)
            self.pub.publish(String(data=text_payload))
            self.last_publish_text = text_payload
            self.last_publish_time = stamp

    def decode_qr(self, img):
        texts = []

        # Fast path: retain the original OpenCV + pyzbar behavior on the
        # unmodified frame. Enhanced variants run only if this finds nothing.
        self.decode_opencv(img, texts)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.decode_pyzbar(gray, texts)
        if texts:
            return texts

        # Slow path for small/distant QR codes. CLAHE improves uneven light;
        # mild unsharp masking restores edges before cubic upscaling.
        enhanced = self.clahe.apply(gray)
        blurred = cv2.GaussianBlur(enhanced, (0, 0), 1.0)
        sharpened = cv2.addWeighted(enhanced, 1.6, blurred, -0.6, 0)
        for scale in self.decode_scales:
            if scale == 1.0:
                candidate = sharpened
            else:
                candidate = cv2.resize(
                    sharpened,
                    None,
                    fx=scale,
                    fy=scale,
                    interpolation=cv2.INTER_CUBIC,
                )
            self.decode_opencv(candidate, texts)
            self.decode_pyzbar(candidate, texts)
        return texts

    def decode_opencv(self, image, texts):
        if self.detector is not None:
            try:
                ok, decoded_info, points, straight_qrcode = self.detector.detectAndDecodeMulti(image)
                if ok and decoded_info:
                    for s in decoded_info:
                        normalized = str(s or '').strip()
                        if normalized and normalized not in texts:
                            texts.append(normalized)
            except Exception:
                pass

    def decode_pyzbar(self, gray, texts):
        if pyzbar is not None:
            try:
                kwargs = {"symbols": [ZBarSymbol.QRCODE]} if ZBarSymbol is not None else {}
                codes = pyzbar.decode(gray, **kwargs)
                for code in codes:
                    s = code.data.decode('utf-8', errors='ignore').strip()
                    if s and s not in texts:
                        texts.append(s)
            except Exception:
                pass

    def is_url(self, text):
        return text.startswith('http://') or text.startswith('https://')

    def fetch_url(self, url):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "U-CAR-QR-Scanner/1.0"})
            with urllib.request.urlopen(req, timeout=self.args.timeout) as resp:
                body = resp.read().decode('utf-8', errors='ignore')
            try:
                return json.loads(body)
            except Exception:
                return {"code": -1, "result": None, "raw_body": body}
        except Exception as e:
            return {"code": -1, "result": None, "error": str(e)}

    def fetch_cached_url(self, url):
        with self.url_lock:
            if url in self.url_cache:
                return self.url_cache[url]

        attempts = max(1, int(self.args.fetch_retries))
        result = None
        for attempt in range(1, attempts + 1):
            result = self.fetch_url(url)
            if result and result.get("code") == 200:
                with self.url_lock:
                    self.url_cache[url] = result
                rospy.loginfo(
                    "QR URL cached on attempt %d/%d: %s -> %s",
                    attempt, attempts, url, result.get("result"))
                return result
            if attempt < attempts and not rospy.is_shutdown():
                rospy.logwarn(
                    "QR URL attempt %d/%d failed; retrying %s: %s",
                    attempt, attempts, url,
                    (result or {}).get("error") or "invalid response")
                time.sleep(max(0.0, float(self.args.retry_backoff)) * attempt)
        return result

    def resolve_qr_url(self, url):
        offline_mode = getattr(self.args, "offline_mode", "off")
        if offline_mode == "force":
            return self.fetch_offline_url(url, reason="forced")

        result = self.fetch_cached_url(url)
        if result and result.get("code") == 200:
            return result

        if offline_mode == "fallback":
            offline = self.fetch_offline_url(url, reason=(result or {}).get("error") or "fetch_failed")
            if offline:
                rospy.logwarn("QR URL fetch failed, using offline fallback: %s -> %s", url, offline.get("result"))
                return offline
        return result

    def fetch_offline_url(self, url, reason="offline"):
        category = self.category_from_url(url)
        if not category:
            return None
        items = OFFLINE_ITEMS[category]
        index = abs(hash(url)) % len(items)
        return {
            "code": 200,
            "result": items[index],
            "offline": True,
            "category": category,
            "reason": reason,
        }

    def category_from_url(self, url):
        try:
            parsed = urlparse(url)
            text = ("%s/%s" % (parsed.netloc, parsed.path)).lower()
        except Exception:
            text = str(url).lower()

        if "electronic" in text or "electronics" in text:
            return "electronic"
        if "daily" in text:
            return "daily"
        if "food" in text:
            return "food"
        return None

    def save_image(self, img, prefix):
        now = time.time()
        filename = "%s_%06d_%d.jpg" % (prefix, self.save_count, int(now * 1000))
        path = os.path.join(self.out_dir, filename)
        ok = cv2.imwrite(path, img)
        if ok:
            self.save_count += 1
            rospy.loginfo("saved %s", path)
        else:
            rospy.logwarn("failed to save image: %s", path)


def main():
    parser = argparse.ArgumentParser(description='QR code collector and decoder for ROS U-CAR')
    parser.add_argument('--topic', default='/usb_cam/image_raw', help='camera image topic')
    parser.add_argument('--output', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'yolo_dataset', 'qr_images'), help='QR image save dir')
    parser.add_argument('--pub-topic', default='/qr_code_data', help='publish decoded QR json to this topic')
    parser.add_argument('--status-topic', default='/qr_decoder/status',
                        help='publish decoder readiness and pending URL count')
    parser.add_argument('--odom-topic', default='/odom',
                        help='odometry used to trigger one burst after each stop')
    parser.add_argument('--fetch', action='store_true', help='if QR content is URL, request it and parse JSON')
    parser.add_argument('--save-on-detect', action='store_true', help='save image whenever QR is detected')
    parser.add_argument('--save-all', action='store_true', help='save raw images periodically even if no QR is detected')
    parser.add_argument('--save-keyframes', choices=['true', 'false'], default='false',
                        help='save only selected stationary whole frames')
    parser.add_argument('--keyframe-output',
                        default='/home/ucar/instant_ws/qr_keyframes',
                        help='directory for selected keyframe JPG and JSON files')
    parser.add_argument('--interval', type=float, default=0.5, help='save interval when --save-all enabled')
    parser.add_argument('--repeat-period', type=float, default=2.0, help='republish same QR result after seconds')
    parser.add_argument('--timeout', type=float, default=3.0, help='HTTP timeout seconds')
    parser.add_argument('--fetch-retries', type=int, default=3,
                        help='HTTP attempts retained after a QR leaves the camera view')
    parser.add_argument('--retry-backoff', type=float, default=0.20,
                        help='base seconds between URL fetch retries')
    parser.add_argument('--decode-interval', type=float, default=0.10,
                        help='minimum seconds between decode attempts')
    parser.add_argument('--decode-scales', default='1.0,1.5,2.0',
                        help='comma-separated enhanced decode scales')
    parser.add_argument('--keyframe-sample-count', type=int, default=6,
                        help='whole frames sampled after each confirmed stop')
    parser.add_argument('--keyframe-sample-window', type=float, default=0.18,
                        help='seconds used to collect each stationary burst')
    parser.add_argument('--stationary-hold', type=float, default=0.15,
                        help='required stopped seconds before sampling')
    parser.add_argument('--odom-stale-sec', type=float, default=0.50,
                        help='stop sampling when odometry is older than this')
    parser.add_argument('--stationary-linear-tolerance', type=float, default=0.025,
                        help='maximum absolute odom linear speed treated as stopped')
    parser.add_argument('--stationary-angular-tolerance', type=float, default=0.05,
                        help='maximum absolute odom angular speed treated as stopped')
    parser.add_argument('--moving-decode-interval', type=float, default=0.30,
                        help='sparse whole-frame interval during final slow fallback')
    parser.add_argument('--moving-decode-max-angular-speed', type=float, default=0.25,
                        help='maximum slow-fallback angular speed eligible for decode')
    parser.add_argument('--moving-decode-arm-sec', type=float, default=0.75,
                        help='continuous slow motion required before fallback decoding')
    parser.add_argument('--white-contour-test', action='store_true',
                        help='log white-contour diagnostics after stops; never gate decode')
    parser.add_argument('--flip', action='store_true', help='horizontal flip image before decode')
    parser.add_argument('--offline-fallback', action='store_true', help='use local QR result if URL fetch fails')
    parser.add_argument('--offline-mode', choices=['off', 'fallback', 'force'], default=None,
                        help='off: only real URL; fallback: URL first then local; force: local only')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if args.offline_mode is None:
        args.offline_mode = 'fallback' if args.offline_fallback else 'off'

    rospy.init_node('qr_collect_and_decode')
    QRCollectAndDecode(args)
    rospy.spin()


if __name__ == '__main__':
    main()
