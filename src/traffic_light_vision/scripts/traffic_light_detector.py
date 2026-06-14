#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from collections import deque

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String


class TrafficLightDetector:
    def __init__(self):
        rospy.init_node("traffic_light_detector", anonymous=False)

        self.image_topic = rospy.get_param("~image_topic", "/ucar_camera/image_raw")
        self.state_topic = rospy.get_param("~state_topic", "/traffic_light/state")
        self.command_topic = rospy.get_param(
            "~command_topic",
            rospy.get_param("~start_topic", "/follow_begin"),
        )
        self.debug_topic = rospy.get_param("~debug_topic", "/traffic_light/debug_image")

        self.command_map = {
            "execute": rospy.get_param("~execute_command", "Middle"),
            "left": rospy.get_param("~left_command", "Left"),
            "right": rospy.get_param("~right_command", "Right"),
            "red": rospy.get_param("~red_command", "Stop"),
        }

        self.confirm_frames = max(1, int(rospy.get_param("~confirm_frames", 3)))
        self.publish_cooldown = float(rospy.get_param("~publish_cooldown", 2.0))
        self.repeat_same_state = self.get_bool_param("~repeat_same_state", False)
        self.history = deque(maxlen=self.confirm_frames)
        self.last_command_time = rospy.Time(0)
        self.last_command_state = None

        self.roi_x_min = float(rospy.get_param("~roi_x_min", 0.0))
        self.roi_x_max = float(rospy.get_param("~roi_x_max", 1.0))
        self.roi_y_min = float(rospy.get_param("~roi_y_min", 0.20))
        self.roi_y_max = float(rospy.get_param("~roi_y_max", 1.0))

        self.red_h1_min = int(rospy.get_param("~red_h1_min", 0))
        self.red_h1_max = int(rospy.get_param("~red_h1_max", 12))
        self.red_h2_min = int(rospy.get_param("~red_h2_min", 165))
        self.red_h2_max = int(rospy.get_param("~red_h2_max", 179))
        self.red_s_min = int(rospy.get_param("~red_s_min", 45))
        self.red_v_min = int(rospy.get_param("~red_v_min", 70))
        self.green_h_min = int(rospy.get_param("~green_h_min", 35))
        self.green_h_max = int(rospy.get_param("~green_h_max", 95))
        self.green_s_min = int(rospy.get_param("~green_s_min", 35))
        self.green_v_min = int(rospy.get_param("~green_v_min", 70))

        self.bright_v_min = int(rospy.get_param("~bright_v_min", 225))
        self.morph_kernel_size = max(1, int(rospy.get_param("~morph_kernel_size", 5)))
        self.candidate_halo_kernel = max(3, int(rospy.get_param("~candidate_halo_kernel", 17)))

        self.min_area = float(rospy.get_param("~min_area", 80.0))
        self.max_area_ratio = float(rospy.get_param("~max_area_ratio", 0.20))
        self.min_circularity = float(rospy.get_param("~min_circularity", 0.12))
        self.min_aspect_ratio = float(rospy.get_param("~min_aspect_ratio", 0.20))
        self.max_aspect_ratio = float(rospy.get_param("~max_aspect_ratio", 5.0))
        self.execute_circularity_min = float(rospy.get_param("~execute_circularity_min", 0.58))
        self.execute_aspect_min = float(rospy.get_param("~execute_aspect_min", 0.65))
        self.execute_aspect_max = float(rospy.get_param("~execute_aspect_max", 1.55))
        self.arrow_direction_bias_min = float(rospy.get_param("~arrow_direction_bias_min", 0.08))

        self.publish_debug = self.get_bool_param("~publish_debug", True)
        self.bridge = CvBridge()

        self.state_pub = rospy.Publisher(self.state_topic, String, queue_size=1)
        self.command_pub = rospy.Publisher(self.command_topic, String, queue_size=1)
        self.debug_pub = rospy.Publisher(self.debug_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24
        )

        rospy.loginfo(
            "traffic_light_detector started: image=%s state=%s command=%s",
            self.image_topic,
            self.state_topic,
            self.command_topic,
        )

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            rospy.logerr("cv_bridge error: %s", exc)
            return

        detected_state, debug_frame = self.detect(frame)
        self.history.append(detected_state)
        stable_state = self.stable_state()

        self.state_pub.publish(String(data=stable_state))
        self.maybe_publish_command(stable_state)

        if self.publish_debug:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except Exception as exc:
                rospy.logwarn("debug image publish failed: %s", exc)

    def stable_state(self):
        if len(self.history) < self.confirm_frames:
            return "unknown"
        first = self.history[0]
        if first == "unknown":
            return "unknown"
        if all(item == first for item in self.history):
            return first
        return "unknown"

    def maybe_publish_command(self, state):
        if state == "unknown":
            return

        now = rospy.Time.now()
        cooldown_ready = (now - self.last_command_time).to_sec() >= self.publish_cooldown
        same_state = state == self.last_command_state
        if same_state and not self.repeat_same_state:
            return
        if not cooldown_ready:
            return

        command = self.command_map.get(state)
        if not command:
            return

        self.command_pub.publish(String(data=command))
        self.last_command_time = now
        self.last_command_state = state
        rospy.logwarn("traffic light %s: published %s to %s", state, command, self.command_topic)

    def detect(self, frame):
        debug = frame.copy()
        height, width = frame.shape[:2]
        x1, y1, x2, y2 = self.search_rect(width, height)
        search_roi = frame[y1:y2, x1:x2]

        # 先在大范围 ROI 内做粗分割，缩小后续查找区域，减少反光和背景噪声。
        candidate_mask = self.build_candidate_mask(search_roi)
        contours = self.find_contours(candidate_mask)
        cv2.rectangle(debug, (x1, y1), (x2 - 1, y2 - 1), (255, 255, 0), 2)

        best = None
        for contour in contours:
            contour_area = cv2.contourArea(contour)
            max_area = self.max_area_ratio * search_roi.shape[0] * search_roi.shape[1]
            if contour_area < self.min_area or contour_area > max_area:
                continue

            rx, ry, rw, rh = cv2.boundingRect(contour)
            pad = max(8, int(max(rw, rh) * 0.45))
            cx1 = max(0, rx - pad)
            cy1 = max(0, ry - pad)
            cx2 = min(search_roi.shape[1], rx + rw + pad)
            cy2 = min(search_roi.shape[0], ry + rh + pad)
            candidate_roi = search_roi[cy1:cy2, cx1:cx2]

            result = self.classify_candidate(candidate_roi, x1 + cx1, y1 + cy1)
            if result is None:
                continue

            cv2.rectangle(debug, (x1 + rx, y1 + ry), (x1 + rx + rw, y1 + ry + rh), (180, 180, 180), 1)
            if best is None or result["score"] > best["score"]:
                best = result

        if best is None:
            cv2.putText(debug, "traffic: unknown", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            return "unknown", debug

        self.draw_detection(debug, best)
        return best["state"], debug

    def classify_candidate(self, roi, offset_x, offset_y):
        if roi.size == 0:
            return None

        # 候选 ROI 转 HSV 后分别提取红色和绿色，红色跨 HSV 色环两端。
        hsv = cv2.cvtColor(cv2.GaussianBlur(roi, (5, 5), 0), cv2.COLOR_BGR2HSV)
        red_mask, green_mask, bright_mask = self.color_masks(hsv)
        red_mask = self.clean_mask(red_mask)
        green_mask = self.clean_mask(green_mask)

        # 过曝光灯芯会变白，白色高亮只用来补候选范围，最终颜色仍看周边色晕。
        color_halo = cv2.dilate(cv2.bitwise_or(red_mask, green_mask), self.halo_kernel(), iterations=1)
        bright_near_color = cv2.bitwise_and(bright_mask, color_halo)
        red_mask = cv2.bitwise_or(red_mask, cv2.bitwise_and(bright_near_color, cv2.dilate(red_mask, self.halo_kernel())))
        green_mask = cv2.bitwise_or(green_mask, cv2.bitwise_and(bright_near_color, cv2.dilate(green_mask, self.halo_kernel())))

        red_result = self.best_color_result(red_mask, "red", offset_x, offset_y)
        green_result = self.best_color_result(green_mask, "green", offset_x, offset_y)

        if red_result is not None and green_result is not None:
            return red_result if red_result["score"] >= green_result["score"] else green_result
        return red_result or green_result

    def best_color_result(self, mask, color_name, offset_x, offset_y):
        best = None
        for contour in self.find_contours(mask):
            metrics = self.contour_metrics(contour, mask.shape)
            if not self.accept_metrics(metrics):
                continue

            if color_name == "red":
                state = "red"
            else:
                state = self.classify_green_shape(contour, metrics)

            result = {
                "state": state,
                "color": color_name,
                "score": metrics["area"],
                "rect": (
                    offset_x + metrics["x"],
                    offset_y + metrics["y"],
                    metrics["w"],
                    metrics["h"],
                ),
                "metrics": metrics,
            }
            if best is None or result["score"] > best["score"]:
                best = result
        return best

    def classify_green_shape(self, contour, metrics):
        circular = (
            metrics["circularity"] >= self.execute_circularity_min
            and self.execute_aspect_min <= metrics["aspect"] <= self.execute_aspect_max
        )
        if circular:
            return "execute"

        # 箭头方向用轮廓左右延展和质心偏移综合判断，适合发光箭头的轮廓。
        moments = cv2.moments(contour)
        if abs(moments["m00"]) < 1e-6:
            return "execute"

        centroid_x = moments["m10"] / moments["m00"]
        bbox_center_x = metrics["x"] + metrics["w"] * 0.5
        points = contour.reshape(-1, 2)
        left_extent = centroid_x - float(np.min(points[:, 0]))
        right_extent = float(np.max(points[:, 0])) - centroid_x
        extent_bias = (right_extent - left_extent) / max(float(metrics["w"]), 1.0)
        mass_bias = (bbox_center_x - centroid_x) / max(float(metrics["w"]), 1.0)
        direction_bias = 0.6 * extent_bias + 0.4 * mass_bias

        if direction_bias >= self.arrow_direction_bias_min:
            return "right"
        if direction_bias <= -self.arrow_direction_bias_min:
            return "left"
        return "execute"

    def build_candidate_mask(self, bgr_roi):
        hsv = cv2.cvtColor(cv2.GaussianBlur(bgr_roi, (5, 5), 0), cv2.COLOR_BGR2HSV)
        red_mask, green_mask, bright_mask = self.color_masks(hsv)
        color_mask = self.clean_mask(cv2.bitwise_or(red_mask, green_mask))
        color_halo = cv2.dilate(color_mask, self.halo_kernel(), iterations=1)
        bright_near_color = cv2.bitwise_and(bright_mask, color_halo)
        candidate_mask = cv2.bitwise_or(color_mask, bright_near_color)
        return self.clean_mask(candidate_mask)

    def color_masks(self, hsv):
        red_mask1 = cv2.inRange(
            hsv,
            (self.red_h1_min, self.red_s_min, self.red_v_min),
            (self.red_h1_max, 255, 255),
        )
        red_mask2 = cv2.inRange(
            hsv,
            (self.red_h2_min, self.red_s_min, self.red_v_min),
            (self.red_h2_max, 255, 255),
        )
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        green_mask = cv2.inRange(
            hsv,
            (self.green_h_min, self.green_s_min, self.green_v_min),
            (self.green_h_max, 255, 255),
        )
        _, _, v = cv2.split(hsv)
        bright_mask = cv2.inRange(v, self.bright_v_min, 255)
        return red_mask, green_mask, bright_mask

    def clean_mask(self, mask):
        kernel = self.morph_kernel()
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def contour_metrics(self, contour, mask_shape):
        area = float(cv2.contourArea(contour))
        perimeter = float(cv2.arcLength(contour, True))
        x, y, w, h = cv2.boundingRect(contour)
        circularity = 0.0 if perimeter <= 1e-6 else 4.0 * math.pi * area / (perimeter * perimeter)
        aspect = float(w) / max(float(h), 1.0)
        max_area = self.max_area_ratio * float(mask_shape[0] * mask_shape[1])
        return {
            "area": area,
            "perimeter": perimeter,
            "circularity": circularity,
            "aspect": aspect,
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "max_area": max_area,
        }

    def accept_metrics(self, metrics):
        return (
            metrics["area"] >= self.min_area
            and metrics["area"] <= metrics["max_area"]
            and metrics["circularity"] >= self.min_circularity
            and self.min_aspect_ratio <= metrics["aspect"] <= self.max_aspect_ratio
        )

    def draw_detection(self, debug, result):
        x, y, w, h = result["rect"]
        color = {
            "red": (0, 0, 255),
            "execute": (0, 255, 0),
            "left": (0, 255, 0),
            "right": (0, 255, 0),
        }.get(result["state"], (0, 255, 255))
        metrics = result["metrics"]
        label = "%s area=%.0f circ=%.2f asp=%.2f" % (
            result["state"],
            metrics["area"],
            metrics["circularity"],
            metrics["aspect"],
        )
        cv2.rectangle(debug, (x, y), (x + w, y + h), color, 2)
        cv2.putText(debug, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    def search_rect(self, width, height):
        x1 = self.clamp_int(self.roi_x_min * width, 0, width - 1)
        x2 = self.clamp_int(self.roi_x_max * width, x1 + 1, width)
        y1 = self.clamp_int(self.roi_y_min * height, 0, height - 1)
        y2 = self.clamp_int(self.roi_y_max * height, y1 + 1, height)
        return x1, y1, x2, y2

    def morph_kernel(self):
        size = self.morph_kernel_size
        if size % 2 == 0:
            size += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    def halo_kernel(self):
        size = self.candidate_halo_kernel
        if size % 2 == 0:
            size += 1
        return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))

    @staticmethod
    def find_contours(mask):
        result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(result) == 3:
            return result[1]
        return result[0]

    @staticmethod
    def clamp_int(value, low, high):
        return int(max(low, min(high, round(value))))

    @staticmethod
    def get_bool_param(name, default):
        value = rospy.get_param(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)


if __name__ == "__main__":
    try:
        TrafficLightDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
