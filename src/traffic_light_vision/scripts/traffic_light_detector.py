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
        self.canny_low = int(rospy.get_param("~canny_low", 40))
        self.canny_high = int(rospy.get_param("~canny_high", 120))
        self.core_erode_kernel_size = max(1, int(rospy.get_param("~core_erode_kernel_size", 3)))
        self.refine_contour_with_edges = self.get_bool_param("~refine_contour_with_edges", False)

        self.min_area = float(rospy.get_param("~min_area", 80.0))
        self.max_area_ratio = float(rospy.get_param("~max_area_ratio", 0.20))
        self.min_circularity = float(rospy.get_param("~min_circularity", 0.08))
        self.min_aspect_ratio = float(rospy.get_param("~min_aspect_ratio", 0.20))
        self.max_aspect_ratio = float(rospy.get_param("~max_aspect_ratio", 5.0))
        self.min_fill_ratio = float(rospy.get_param("~min_fill_ratio", 0.08))
        self.max_fill_ratio = float(rospy.get_param("~max_fill_ratio", 0.95))
        self.min_color_ratio = float(rospy.get_param("~min_color_ratio", 0.12))
        self.reflection_y_penalty_start = float(rospy.get_param("~reflection_y_penalty_start", 0.62))

        self.poly_epsilon_ratio = float(rospy.get_param("~poly_epsilon_ratio", 0.035))
        self.arrow_tip_min_distance_ratio = float(rospy.get_param("~arrow_tip_min_distance_ratio", 0.28))
        self.arrow_horizontal_min_cos = float(rospy.get_param("~arrow_horizontal_min_cos", 0.45))
        self.arrow_vertical_min_cos = float(rospy.get_param("~arrow_vertical_min_cos", 0.50))
        self.arrow_fallback_bias_min = float(rospy.get_param("~arrow_fallback_bias_min", 0.08))

        self.publish_debug = self.get_bool_param("~publish_debug", True)
        self.show_search_roi = self.get_bool_param("~show_search_roi", True)
        self.show_candidate_boxes = self.get_bool_param("~show_candidate_boxes", True)
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
        roi_area = float(search_roi.shape[0] * search_roi.shape[1])

        if self.show_search_roi:
            self.draw_corner_rect(debug, (x1, y1, x2 - x1, y2 - y1), (255, 255, 0), 2)

        # HSV locks lamp color first. Canny is used for scoring/direction, not for box size.
        hsv = cv2.cvtColor(cv2.GaussianBlur(search_roi, (5, 5), 0), cv2.COLOR_BGR2HSV)
        red_mask, green_mask, bright_mask = self.color_masks(hsv)
        candidates = []
        candidates.extend(self.collect_color_candidates(search_roi, red_mask, bright_mask, "red", x1, y1, roi_area))
        candidates.extend(self.collect_color_candidates(search_roi, green_mask, bright_mask, "green", x1, y1, roi_area))

        best = None
        for candidate in candidates:
            if self.show_candidate_boxes:
                x, y, w, h = candidate["rect"]
                self.draw_corner_rect(debug, (x, y, w, h), (160, 160, 160), 1)
            if best is None or candidate["score"] > best["score"]:
                best = candidate

        if best is None:
            cv2.putText(debug, "traffic: unknown", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            return "unknown", debug

        self.draw_detection(debug, best)
        return best["state"], debug

    def collect_color_candidates(self, bgr_roi, raw_color_mask, bright_mask, color_name, offset_x, offset_y, roi_area):
        color_mask = self.clean_mask(raw_color_mask)

        # Build contour boxes from a tight color core. This prevents floor reflections from
        # merging with the lamp into one large connected component.
        contour_mask = cv2.erode(color_mask, self.core_kernel(), iterations=1)
        contour_mask = self.clean_mask(contour_mask)

        halo = cv2.dilate(contour_mask, self.halo_kernel(), iterations=1)
        bright_near_color = cv2.bitwise_and(bright_mask, halo)
        score_mask = self.clean_mask(cv2.bitwise_or(color_mask, bright_near_color))

        gray = cv2.cvtColor(bgr_roi, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, self.canny_low, self.canny_high)
        edge_near_color = cv2.bitwise_and(edges, cv2.dilate(contour_mask, self.halo_kernel(), iterations=1))

        contour_source = contour_mask
        if self.refine_contour_with_edges:
            edge_inside_core = cv2.bitwise_and(edge_near_color, cv2.dilate(contour_mask, self.core_kernel(), iterations=1))
            contour_source = self.clean_mask(cv2.bitwise_or(contour_mask, edge_inside_core))

        candidates = []
        max_area = self.max_area_ratio * roi_area
        contours = self.find_contours(contour_source)
        for contour in contours:
            metrics = self.contour_metrics(contour, contour_source.shape, score_mask, edge_near_color, max_area)
            if not self.accept_metrics(metrics):
                continue

            state = "red" if color_name == "red" else self.classify_arrow(contour, metrics)
            score = self.score_candidate(metrics, state)
            x, y, w, h = metrics["x"], metrics["y"], metrics["w"], metrics["h"]
            candidates.append({
                "state": state,
                "color": color_name,
                "score": score,
                "rect": (offset_x + x, offset_y + y, w, h),
                "metrics": metrics,
            })
        return candidates

    def classify_arrow(self, contour, metrics):
        direction = self.arrow_direction_from_tip(contour, metrics)
        if direction is None:
            direction = self.arrow_direction_fallback(contour, metrics)
        return direction or "execute"

    def arrow_direction_from_tip(self, contour, metrics):
        moments = cv2.moments(contour)
        if abs(moments["m00"]) < 1e-6 or metrics["perimeter"] <= 1e-6:
            return None

        centroid = np.array([moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]], dtype=np.float32)
        epsilon = self.poly_epsilon_ratio * metrics["perimeter"]
        approx = cv2.approxPolyDP(contour, epsilon, True)
        hull = cv2.convexHull(approx if len(approx) >= 3 else contour)
        points = hull.reshape(-1, 2).astype(np.float32)
        if len(points) == 0:
            return None

        vectors = points - centroid
        distances = np.linalg.norm(vectors, axis=1)
        tip_index = int(np.argmax(distances))
        tip_distance = float(distances[tip_index])
        min_tip_distance = self.arrow_tip_min_distance_ratio * max(float(metrics["w"]), float(metrics["h"]), 1.0)
        if tip_distance < min_tip_distance:
            return None

        vx, vy = vectors[tip_index] / max(tip_distance, 1e-6)
        metrics["direction_angle"] = math.degrees(math.atan2(-vy, vx))
        metrics["tip_distance"] = tip_distance

        if vx <= -self.arrow_horizontal_min_cos:
            return "left"
        if vx >= self.arrow_horizontal_min_cos:
            return "right"
        if vy <= -self.arrow_vertical_min_cos:
            return "execute"
        return None

    def arrow_direction_fallback(self, contour, metrics):
        moments = cv2.moments(contour)
        if abs(moments["m00"]) < 1e-6:
            return None

        centroid_x = moments["m10"] / moments["m00"]
        bbox_center_x = metrics["x"] + metrics["w"] * 0.5
        points = contour.reshape(-1, 2)
        left_extent = centroid_x - float(np.min(points[:, 0]))
        right_extent = float(np.max(points[:, 0])) - centroid_x
        extent_bias = (right_extent - left_extent) / max(float(metrics["w"]), 1.0)
        mass_bias = (bbox_center_x - centroid_x) / max(float(metrics["w"]), 1.0)
        direction_bias = 0.6 * extent_bias + 0.4 * mass_bias
        metrics["direction_bias"] = direction_bias

        if direction_bias >= self.arrow_fallback_bias_min:
            return "right"
        if direction_bias <= -self.arrow_fallback_bias_min:
            return "left"

        rect = cv2.minAreaRect(contour)
        rw, rh = rect[1]
        if rw > 1e-6 and rh > 1e-6 and max(rw, rh) / min(rw, rh) >= 1.35:
            metrics["direction_angle"] = rect[2]
        return "execute"

    def score_candidate(self, metrics, state):
        area_norm = min(1.0, metrics["area"] / max(self.min_area * 6.0, 1.0))
        color_term = min(1.0, metrics["color_ratio"] * 2.0)
        edge_term = min(1.0, metrics["edge_ratio"] * 4.0)
        fill_term = 1.0 - min(1.0, abs(metrics["fill_ratio"] - 0.42) / 0.42)
        shape_term = min(1.0, metrics["circularity"] * 1.6)
        y_weight = metrics.get("y_weight", 1.0)
        arrow_bonus = 0.12 if state in ("left", "right", "execute") and metrics.get("tip_distance", 0.0) > 0 else 0.0
        red_bonus = 0.10 if state == "red" else 0.0
        return y_weight * (0.34 * area_norm + 0.28 * color_term + 0.14 * edge_term + 0.14 * fill_term + 0.10 * shape_term + arrow_bonus + red_bonus)

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

    def contour_metrics(self, contour, mask_shape, color_mask, edge_mask, max_area):
        area = float(cv2.contourArea(contour))
        perimeter = float(cv2.arcLength(contour, True))
        x, y, w, h = cv2.boundingRect(contour)
        bbox_area = max(float(w * h), 1.0)
        contour_mask = np.zeros(mask_shape, dtype=np.uint8)
        cv2.drawContours(contour_mask, [contour], -1, 255, -1)
        color_pixels = float(cv2.countNonZero(cv2.bitwise_and(color_mask, contour_mask)))
        edge_pixels = float(cv2.countNonZero(cv2.bitwise_and(edge_mask, contour_mask)))
        circularity = 0.0 if perimeter <= 1e-6 else 4.0 * math.pi * area / (perimeter * perimeter)
        aspect = float(w) / max(float(h), 1.0)
        center_y_ratio = (float(y) + 0.5 * float(h)) / max(float(mask_shape[0]), 1.0)
        y_weight = 1.0
        if center_y_ratio > self.reflection_y_penalty_start:
            y_weight = max(0.25, 1.0 - (center_y_ratio - self.reflection_y_penalty_start) * 1.5)
        return {
            "area": area,
            "perimeter": perimeter,
            "circularity": circularity,
            "aspect": aspect,
            "fill_ratio": area / bbox_area,
            "color_ratio": color_pixels / bbox_area,
            "edge_ratio": edge_pixels / max(perimeter, 1.0),
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "max_area": max_area,
            "y_weight": y_weight,
            "direction_angle": 0.0,
        }

    def accept_metrics(self, metrics):
        return (
            metrics["area"] >= self.min_area
            and metrics["area"] <= metrics["max_area"]
            and metrics["circularity"] >= self.min_circularity
            and self.min_aspect_ratio <= metrics["aspect"] <= self.max_aspect_ratio
            and self.min_fill_ratio <= metrics["fill_ratio"] <= self.max_fill_ratio
            and metrics["color_ratio"] >= self.min_color_ratio
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
        label = "%s s=%.2f a=%.0f ar=%.2f c=%.2f ang=%.0f" % (
            result["state"],
            result["score"],
            metrics["area"],
            metrics["aspect"],
            metrics["circularity"],
            metrics.get("direction_angle", 0.0),
        )
        cv2.rectangle(debug, (x, y), (x + w, y + h), color, 3)
        cv2.putText(debug, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)

    def draw_corner_rect(self, image, rect, color, thickness):
        x, y, w, h = rect
        length = max(10, int(min(w, h) * 0.18))
        cv2.line(image, (x, y), (x + length, y), color, thickness)
        cv2.line(image, (x, y), (x, y + length), color, thickness)
        cv2.line(image, (x + w, y), (x + w - length, y), color, thickness)
        cv2.line(image, (x + w, y), (x + w, y + length), color, thickness)
        cv2.line(image, (x, y + h), (x + length, y + h), color, thickness)
        cv2.line(image, (x, y + h), (x, y + h - length), color, thickness)
        cv2.line(image, (x + w, y + h), (x + w - length, y + h), color, thickness)
        cv2.line(image, (x + w, y + h), (x + w, y + h - length), color, thickness)

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

    def core_kernel(self):
        size = self.core_erode_kernel_size
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
