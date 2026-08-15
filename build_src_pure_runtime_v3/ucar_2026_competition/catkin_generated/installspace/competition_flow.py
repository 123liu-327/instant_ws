#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Competition state machine for the five smart-factory subtasks."""

from __future__ import annotations

import json
import math
import os
import signal
import socket
import subprocess
import threading
import time
import uuid
from collections import OrderedDict

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseActionGoal,
    MoveBaseActionResult,
    MoveBaseGoal,
)
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger, TriggerResponse
from ucar_2026_competition_speech.srv import Announce
from ucar_2026_smart_factory_llm.srv import ReasonPickupOrder
from ucar_2026_competition.logic import (
    CATEGORY_LABELS,
    base_is_stopped,
    TemporalTargetFilter,
    DirectedYawAccumulator,
    JsonLineBuffer,
    TRACK_CONFIG,
    normalize_angle,
    normalize_category,
    parse_category,
    parse_task_categories,
    qr_values_from_payload,
    split_rotation_steps,
    stage_sequence,
    task4_handoff_required,
    task4_start_action,
    traffic_decision_from_payload,
    trigger_delivery_state,
)


class StageError(RuntimeError):
    pass


class Aborted(RuntimeError):
    pass


def bool_param(name, default=False):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class CompetitionFlow:
    def __init__(self):
        self.mode = rospy.get_param("~start_stage", "full").strip().lower()
        self.enable_simulation = bool_param("~enable_simulation", False)
        self.debug = bool_param("~debug", False)
        self.aborted = threading.Event()
        self.resume_event = threading.Event()
        self.children = {}
        self.child_log_handles = {}
        self.lock = threading.RLock()
        self.voice_transition_lock = threading.Lock()

        self.status_pub = rospy.Publisher(
            rospy.get_param("~status_topic", "/competition/status"),
            String,
            queue_size=20,
            latch=True,
        )
        self.result_pub = rospy.Publisher(
            rospy.get_param("~task1_result_topic", "/competition/task1_result"),
            String,
            queue_size=5,
            latch=True,
        )
        self.traffic_pub = rospy.Publisher(
            "/competition/traffic_decision", String, queue_size=5, latch=True
        )
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
        self.vision_target_pub = rospy.Publisher(
            "/vision/target", String, queue_size=10)

        self.wakeup_received = False
        self.voice_prompt_started = False
        self.voice_listening = False
        self.voice_command_acknowledged = False
        self.voice_command_ack_in_progress = False
        self.voice_handshake_error = ""
        self.voice_wakeup_generation = 0
        self.question = ""
        self.category = normalize_category(rospy.get_param("~target_category", ""))
        self.sim_category = normalize_category(rospy.get_param("~sim_category", ""))
        self.task1_result = {
            "pickup_item": rospy.get_param("~target_item", "").strip(),
            "pickup_workshop": rospy.get_param("~target_workshop", "").strip(),
            "sim_item": rospy.get_param("~sim_item", "").strip(),
            "sim_workshop": rospy.get_param("~sim_workshop", "").strip(),
        }

        self.qr_items = OrderedDict()
        self.qr_collecting = False
        self.qr_navigation_watching = False
        self.qr_navigation_goal_id = ""
        self.qr_navigation_result = None
        self.qr_odom_yaw = None
        self.qr_odom_position = None
        self.qr_odom_received_at = 0.0
        self.qr_map_yaw = None
        self.qr_map_odom_yaw = None
        self.qr_map_received_at = 0.0
        self.qr_scan = None
        self.qr_scan_received_at = 0.0
        self.task1_instruction = ""
        self.task1_llm_generation = 0
        self.task1_llm_thread = None
        self.task1_llm_done = threading.Event()
        self.task1_llm_result = None
        self.task1_llm_error = ""
        self.task1_llm_items = []
        self.task3_thread = None
        self.task3_done = threading.Event()
        self.task3_result_text = ""
        self.task3_error = ""
        self.base_twist = None
        self.handoff_scan_received_at = 0.0
        self.handoff_costmap_received_at = 0.0
        self.ocr_target = None
        self.ocr_last_message_at = 0.0
        self.ocr_last_logged_category = None
        self.ocr_last_log_signature = None
        self.ocr_terminal_log_interval = max(0.2, float(rospy.get_param(
            "~ocr_terminal_log_interval_sec", 1.2)))
        self.ocr_terminal_last_at = {}
        self.coverage_observation_log_signature = None
        self.ocr_filter = TemporalTargetFilter(
            rospy.get_param("~ocr_required_hits", 2),
            rospy.get_param("~ocr_evidence_window_sec", 1.5),
        )
        self.sim_preview_filter = TemporalTargetFilter(
            rospy.get_param("~simulation_cache_required_hits", 2),
            rospy.get_param("~simulation_cache_window_sec", 1.8),
        )
        self.factory_parking_index = 0
        self.coverage_observation = None
        self.coverage_observation_received_at = 0.0
        self.last_coverage_anchor_index = 0
        self.first_parking_anchor_index = 0
        self.cached_sim_observation = None
        self.vision_trigger_latched = False
        self.trigger_request_pending = False
        self.trigger_request_started_at = 0.0
        self.trigger_service_accepted = False
        self.trigger_acknowledged = False
        self.trigger_service_name = rospy.get_param(
            "~target_trigger_service", "/vision_triggered_navigator/trigger_target")
        self.trigger_ack_timeout = float(rospy.get_param("~trigger_ack_timeout_sec", 2.0))
        self.navigator_status = ""
        self.task1_task2_handoff_prepared = False
        self.traffic_decision = rospy.get_param("~traffic_decision", "").strip().lower()
        self.red_announced = False
        self.strict_mission_status = {}
        self.track_status = {}

        rospy.Subscriber("/wakeup", String, self._wakeup_cb, queue_size=5)
        rospy.Subscriber("/question", String, self._question_cb, queue_size=5)
        rospy.Subscriber("/qr_code_data", String, self._qr_cb, queue_size=20)
        rospy.Subscriber(
            rospy.get_param("~qr_odom_topic", "/odom"),
            Odometry,
            self._qr_odom_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~qr_map_pose_topic", "/amcl_pose"),
            PoseWithCovarianceStamped,
            self._qr_map_pose_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            "/scan", LaserScan, self._handoff_scan_cb,
            queue_size=1, buff_size=1024 * 1024, tcp_nodelay=True)
        rospy.Subscriber(
            "/move_base/local_costmap/costmap", OccupancyGrid,
            self._handoff_costmap_cb,
            queue_size=1, buff_size=4 * 1024 * 1024, tcp_nodelay=True)
        rospy.Subscriber(
            "/move_base/goal", MoveBaseActionGoal, self._qr_move_base_goal_cb, queue_size=5
        )
        rospy.Subscriber(
            "/move_base/result",
            MoveBaseActionResult,
            self._qr_move_base_result_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            "/factory_sign_ppocr_rknn_test/result", String, self._ocr_cb, queue_size=20
        )
        rospy.Subscriber(
            "/vision_triggered_navigator/status", String, self._navigator_cb, queue_size=20
        )
        rospy.Subscriber(
            rospy.get_param(
                "~coverage_observation_topic",
                "/vision_triggered_navigator/coverage_observation",
            ),
            String,
            self._coverage_observation_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            "/traffic_light_rknn_test/detections", String, self._traffic_cb, queue_size=20
        )
        rospy.Subscriber(
            "/strict_mission/status", String, self._strict_mission_cb, queue_size=20
        )
        for _, topic, _ in TRACK_CONFIG.values():
            rospy.Subscriber(topic, String, self._track_cb, callback_args=topic, queue_size=10)

        rospy.Service("/competition/resume", Trigger, self._resume_cb)
        rospy.Service("/competition/abort", Trigger, self._abort_cb)
        rospy.on_shutdown(self.shutdown)

        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.publish_status("startup", "ready", "competition controller ready")

    # ------------------------------ callbacks ------------------------------
    def _wakeup_cb(self, _msg):
        with self.lock:
            if self.voice_command_acknowledged or self.voice_command_ack_in_progress:
                return
            self.wakeup_received = True
            self.voice_prompt_started = True
            self.voice_wakeup_generation += 1
            generation = self.voice_wakeup_generation
        threading.Thread(
            target=self._prompt_and_start_listening,
            args=(generation,),
            name="voice-wakeup-handshake",
            daemon=True,
        ).start()

    def _question_cb(self, msg):
        question = msg.data.strip()
        physical_category, sim_category = parse_task_categories(question)
        with self.lock:
            if not self.voice_listening or self.voice_command_ack_in_progress:
                rospy.logwarn("ignoring /question outside active voice window: %s", question)
                return
            if not physical_category or not sim_category:
                rospy.logwarn("ignoring incomplete dual-category command: %s", question)
                self.publish_status(
                    "task1",
                    "listening_command",
                    "waiting for physical and simulation categories: {}".format(question),
                )
                return
            if physical_category == sim_category:
                rospy.logwarn("ignoring command with identical categories: %s", question)
                self.publish_status(
                    "task1",
                    "listening_command",
                    "physical and simulation categories must be different",
                )
                return
            self.question = question
            self.voice_listening = False
            self.voice_command_ack_in_progress = True
        threading.Thread(
            target=self._finish_voice_command,
            args=(physical_category, sim_category, question),
            name="voice-command-handshake",
            daemon=True,
        ).start()

    def _voice_control(self, param_name, default_service):
        service = rospy.get_param(param_name, default_service)
        try:
            rospy.wait_for_service(service, timeout=5.0)
            response = rospy.ServiceProxy(service, Trigger)()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise StageError("voice control service failed: {}".format(exc))
        if not response.success:
            raise StageError("voice control rejected: {}".format(response.message))

    def _start_voice_listening(self):
        self._voice_control(
            "~voice_start_listening_service",
            "/speech_command_node/start_listening",
        )

    def _stop_voice_listening(self):
        self._voice_control(
            "~voice_stop_listening_service",
            "/speech_command_node/stop_listening",
        )

    def _set_voice_handshake_error(self, exc):
        with self.lock:
            self.voice_handshake_error = str(exc)
            self.voice_listening = False
            self.voice_command_ack_in_progress = False
        rospy.logerr("voice handshake failed: %s", exc)

    def _prompt_and_start_listening(self, generation):
        try:
            with self.voice_transition_lock:
                with self.lock:
                    if generation != self.voice_wakeup_generation or self.voice_command_acknowledged:
                        return
                    was_listening = self.voice_listening
                    self.voice_listening = False
                if was_listening:
                    self._stop_voice_listening()

                reply = rospy.get_param("~voice_wakeup_reply", "我在").strip() or "我在"
                self.publish_status("task1", "wakeup_ack", "replying and preparing ASR")
                self.announce("custom", text=reply)

                with self.lock:
                    if generation != self.voice_wakeup_generation or self.voice_command_acknowledged:
                        return
                self._start_voice_listening()
                with self.lock:
                    self.voice_listening = True
                self.publish_status(
                    "task1", "listening_command", "waiting for 取得食品/日用品/电子产品"
                )
        except Exception as exc:
            self._set_voice_handshake_error(exc)

    def _finish_voice_command(self, physical_category, sim_category, question):
        try:
            with self.voice_transition_lock:
                self._stop_voice_listening()
                reply = rospy.get_param("~voice_command_reply", "好的").strip() or "好的"
                self.publish_status(
                    "task1",
                    "command_ack",
                    "physical={} simulation={} reply={}".format(
                        physical_category, sim_category, reply
                    ),
                )
                self.announce("custom", text=reply)
                with self.lock:
                    self.category = physical_category
                    self.sim_category = sim_category
                    self.voice_command_acknowledged = True
                    self.voice_command_ack_in_progress = False
                self.publish_status(
                    "task1", "voice_ready", "voice command accepted; navigation may start"
                )
        except Exception as exc:
            self._set_voice_handshake_error(exc)

    def _voice_command_ready(self):
        with self.lock:
            if self.voice_handshake_error:
                raise StageError(self.voice_handshake_error)
            return (
                self.wakeup_received
                and self.voice_command_acknowledged
                and self.category
                and self.sim_category
            )

    def _qr_cb(self, msg):
        if not self.qr_collecting:
            return
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        accepted = []
        expected_count = int(rospy.get_param("~qr_expected_count", 3))
        for key, result in qr_values_from_payload(payload):
            with self.lock:
                if key not in self.qr_items:
                    self.qr_items[key] = result
                    accepted.append((len(self.qr_items), key, result))
        for count, key, result in accepted:
            self.publish_status(
                "task1",
                "qr_item_accepted",
                "[QR {}/{}] item={} identity={}".format(
                    count, expected_count, result, key
                ),
            )
            rospy.logwarn(
                "[二维码 %d/%d] 货品=%s | URL=%s",
                count, expected_count, result, key,
            )
        self._start_task1_reasoning_async()

    def _qr_odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        with self.lock:
            self.qr_odom_yaw = yaw
            self.qr_odom_position = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
            )
            self.qr_odom_received_at = time.monotonic()
            self.base_twist = (
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.linear.y),
                float(msg.twist.twist.angular.z),
            )

    def _qr_map_pose_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        with self.lock:
            self.qr_map_yaw = yaw
            self.qr_map_odom_yaw = self.qr_odom_yaw
            self.qr_map_received_at = time.monotonic()

    def _handoff_scan_cb(self, msg):
        with self.lock:
            self.qr_scan = msg
            self.qr_scan_received_at = time.monotonic()
            self.handoff_scan_received_at = time.monotonic()

    def _handoff_costmap_cb(self, _msg):
        with self.lock:
            self.handoff_costmap_received_at = time.monotonic()

    def _qr_move_base_goal_cb(self, msg):
        with self.lock:
            if self.qr_navigation_watching:
                self.qr_navigation_goal_id = msg.goal_id.id

    def _qr_move_base_result_cb(self, msg):
        with self.lock:
            if not self.qr_navigation_watching or not self.qr_navigation_goal_id:
                return
            if msg.status.goal_id.id != self.qr_navigation_goal_id:
                return
            self.qr_navigation_result = msg.status.status

    def _ocr_cb(self, msg):
        if not self.ocr_target:
            return
        try:
            payload = json.loads(msg.data)
            category = normalize_category(payload.get("category"))
        except Exception:
            return
        now = time.monotonic()
        self.ocr_last_message_at = now
        if category == self.ocr_target and payload.get("target_bbox"):
            self.vision_target_pub.publish(msg)
        confirmed = self.ocr_filter.push(
            self.ocr_target, category, now)
        physical_category = (
            normalize_category(self.task1_result.get("category")) or
            normalize_category(self.task1_result.get("pickup_major"))
        )
        simulation_category = normalize_category(self.sim_category)
        log_signature = (
            self.ocr_target,
            category,
            min(self.ocr_filter.hit_count, self.ocr_filter.required),
            physical_category,
            simulation_category,
        )
        last_terminal_at = self.ocr_terminal_last_at.get(category, 0.0)
        if (category and
                (log_signature != self.ocr_last_log_signature or
                 now - last_terminal_at >= self.ocr_terminal_log_interval)):
            workshop = str(payload.get("workshop", "") or category)
            target_workshop = CATEGORY_LABELS.get(
                self.ocr_target, (self.ocr_target, self.ocr_target))[1]
            physical_workshop = CATEGORY_LABELS.get(
                physical_category,
                (physical_category or "未设置", physical_category or "未设置"),
            )[1]
            simulation_workshop = CATEGORY_LABELS.get(
                simulation_category,
                (simulation_category or "未设置", simulation_category or "未设置"),
            )[1]
            if self.ocr_target == physical_category:
                parking_role = "真实"
            elif self.ocr_target == simulation_category:
                parking_role = "仿真"
            else:
                parking_role = target_workshop
            rospy.logwarn(
                "[厂牌] 识别=%s | 真实目标=%s(%s) | 仿真目标=%s(%s) | "
                "当前停车=%s %d/%d",
                workshop,
                physical_workshop,
                "符合" if category == physical_category else "不符合",
                simulation_workshop,
                "符合" if category == simulation_category else "不符合",
                parking_role,
                min(self.ocr_filter.hit_count, self.ocr_filter.required),
                self.ocr_filter.required,
            )
            self.ocr_terminal_last_at[category] = now
            self.ocr_last_logged_category = category
            self.ocr_last_log_signature = log_signature

        if (self.factory_parking_index == 1 and self.sim_category and
                self.sim_category != self.ocr_target):
            sim_confirmed = self.sim_preview_filter.push(
                self.sim_category, category, now)
            if (sim_confirmed and category == self.sim_category and
                    payload.get("target_bbox")):
                self._remember_simulation_observation(payload)
        if (confirmed and category == self.ocr_target and payload.get("target_bbox") and
                not self.vision_trigger_latched):
            if self.factory_parking_index == 1:
                with self.lock:
                    self.first_parking_anchor_index = int(
                        self.last_coverage_anchor_index)
                rospy.loginfo(
                    "parking 1 target locked at scan anchor %d; "
                    "parking 2 will continue from the following anchor when uncached",
                    self.first_parking_anchor_index,
                )
            self.vision_trigger_latched = True
            self.trigger_request_pending = True
            self.trigger_request_started_at = time.monotonic()
            self.trigger_service_accepted = False
            self.trigger_acknowledged = False
            self.publish_status(
                "task2", "trigger_pending",
                "OCR target confirmed; requesting navigator acknowledgement")
            rospy.loginfo(
                "task2 OCR target confirmed: target=%s hits=%d/%d; "
                "reliable trigger pending (will not retrigger)",
                self.ocr_target,
                self.ocr_filter.hit_count,
                self.ocr_filter.required,
            )

    def _coverage_observation_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            anchor_index = int(payload.get("anchor_index", 0))
            if anchor_index <= 0:
                return
            float(payload.get("x"))
            float(payload.get("y"))
            float(payload.get("yaw"))
        except (TypeError, ValueError, KeyError, json.JSONDecodeError):
            return
        with self.lock:
            self.coverage_observation = payload
            self.coverage_observation_received_at = time.monotonic()
            self.last_coverage_anchor_index = anchor_index
        state = str(payload.get("state", ""))
        signature = (anchor_index, state,
                     round(float(payload.get("x")), 2),
                     round(float(payload.get("y")), 2))
        if signature != self.coverage_observation_log_signature:
            self.coverage_observation_log_signature = signature
            if state in ("navigating", "fallback_selected", "scanning", "covered"):
                rospy.logwarn(
                    "[SCAN POINT %d] state=%s pose=(%.2f, %.2f, %.1fdeg)",
                    anchor_index, state, float(payload.get("x")),
                    float(payload.get("y")),
                    math.degrees(float(payload.get("yaw"))),
                )

    def _remember_simulation_observation(self, ocr_payload):
        with self.lock:
            observation = dict(self.coverage_observation or {})
            observation_age = (
                time.monotonic() - self.coverage_observation_received_at
            )
        max_age = float(rospy.get_param(
            "~simulation_cache_observation_max_age_sec", 60.0))
        if not observation or observation_age > max_age:
            return
        # A remembered target must belong to a point where the vehicle has
        # actually stopped to scan.  A delayed OCR result while travelling to
        # the next point must not move the memory to that next point.
        observation_state = str(observation.get("state", "")).strip().lower()
        if observation_state not in ("scanning", "covered"):
            return

        try:
            center_x = float(ocr_payload.get("target_center_x"))
            image_width = max(1.0, float(ocr_payload.get("image_width")))
        except (TypeError, ValueError):
            center_x = 0.5
            image_width = 1.0
        center_error = abs(center_x / image_width - 0.5) * 2.0
        score = (
            float(ocr_payload.get("category_score", 0.0) or 0.0)
            + float(ocr_payload.get("confidence", 0.0) or 0.0)
            - center_error
        )
        observation.update({
            "category": self.sim_category,
            "score": score,
            "center_error": center_error,
            "cached_at": time.time(),
            "seen_at_monotonic": time.monotonic(),
        })
        with self.lock:
            previous = self.cached_sim_observation
            previous_seen_at = (
                float(previous.get("seen_at_monotonic", -1.0))
                if previous is not None else -1.0
            )
            if observation["seen_at_monotonic"] < previous_seen_at:
                return
            self.cached_sim_observation = observation

        previous_anchor = (
            int(previous.get("anchor_index", 0))
            if previous is not None else 0
        )
        current_anchor = int(observation.get("anchor_index", 0))
        if previous_anchor == current_anchor:
            return
        self.publish_status(
            "task2",
            "simulation_anchor_cached",
            "[CACHE] latest simulation={} anchor={} pose=({:.2f},{:.2f},{:.2f})".format(
                self.sim_category,
                current_anchor,
                float(observation["x"]),
                float(observation["y"]),
                float(observation["yaw"]),
            ),
        )
        rospy.logwarn(
            "[OCR CACHE] latest simulation=%s scan_point=%d "
            "pose=(%.2f, %.2f, %.1fdeg) previous=%d",
            self.sim_category, current_anchor,
            float(observation["x"]), float(observation["y"]),
            math.degrees(float(observation["yaw"])), previous_anchor,
        )

    def _navigator_cb(self, msg):
        status = msg.data.strip().lower()
        if status != self.navigator_status:
            rospy.logwarn("[ROOM NAV] state=%s", status)
        self.navigator_status = status

    def _deliver_target_trigger(self):
        """Deliver one OCR lock through a synchronous service and wait for status ACK."""
        if not self.trigger_request_pending or self.trigger_acknowledged:
            return
        elapsed = time.monotonic() - self.trigger_request_started_at
        delivery_state = trigger_delivery_state(
            self.trigger_service_accepted,
            self.navigator_status,
            elapsed,
            self.trigger_ack_timeout,
        )
        if delivery_state == "acknowledged":
            self.trigger_acknowledged = True
            self.trigger_request_pending = False
            self.publish_status(
                "task2", "trigger_acknowledged",
                "navigator accepted and acknowledged the OCR target")
            rospy.loginfo(
                "task2 trigger acknowledged by navigator status=%s",
                self.navigator_status)
            return

        if delivery_state == "failed":
            self.publish_status(
                "task2", "trigger_delivery_failed", "",
                "navigator did not acknowledge target within {:.1f}s".format(
                    self.trigger_ack_timeout))
            raise StageError(
                "trigger_delivery_failed: no navigator acknowledgement within {:.1f}s".format(
                    self.trigger_ack_timeout))

        if self.trigger_service_accepted:
            return
        try:
            rospy.wait_for_service(self.trigger_service_name, timeout=0.15)
            response = rospy.ServiceProxy(self.trigger_service_name, Trigger)()
            if not response.success:
                rospy.logwarn_throttle(
                    0.5, "target trigger service rejected request: %s", response.message)
                return
            self.trigger_service_accepted = True
            rospy.loginfo("task2 target trigger service accepted: %s", response.message)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn_throttle(
                0.5, "waiting for reliable target trigger service %s: %s",
                self.trigger_service_name, str(exc))

    def _traffic_cb(self, msg):
        try:
            decision = traffic_decision_from_payload(json.loads(msg.data))
            if decision:
                self.traffic_decision = decision
        except Exception:
            return

    def _strict_mission_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                self.strict_mission_status = payload
        except Exception:
            return

    def _track_cb(self, msg, topic):
        self.track_status[topic] = msg.data.strip().lower()

    def _resume_cb(self, _req):
        if self.aborted.is_set():
            return TriggerResponse(False, "competition already aborted")
        self.resume_event.set()
        return TriggerResponse(True, "competition resume requested")

    def _abort_cb(self, _req):
        self.aborted.set()
        self.resume_event.set()
        self.safe_stop(cancel_navigation=True)
        self.stop_all_children()
        return TriggerResponse(True, "competition aborted and vehicle stopped")

    # ------------------------------ infrastructure ------------------------------
    def publish_status(self, stage, state, message="", error=""):
        payload = {
            "stage": stage,
            "state": state,
            "message": message,
            "error": error,
            "stamp": time.time(),
        }
        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        if error:
            rospy.logerr(
                "competition %s/%s: %s | error=%s",
                stage, state, message, error,
            )
        else:
            rospy.loginfo("competition %s/%s: %s", stage, state, message)

    def check_abort(self):
        if self.aborted.is_set() or rospy.is_shutdown():
            raise Aborted("competition aborted")

    def pause_and_retry(self, stage, error):
        self.safe_stop(cancel_navigation=True)
        self.publish_status(stage, "paused", "call /competition/resume after fixing it", str(error))
        self.resume_event.clear()
        while not rospy.is_shutdown() and not self.resume_event.wait(0.2):
            self.check_abort()
        self.check_abort()
        self.publish_status(stage, "resuming", "retrying current stage")

    def run_stage(self, stage, function):
        while not rospy.is_shutdown():
            self.check_abort()
            try:
                return function()
            except StageError as exc:
                self.stop_all_children()
                self.pause_and_retry(stage, exc)

    def safe_stop(self, cancel_navigation=False):
        if cancel_navigation:
            try:
                self.move_base.cancel_all_goals()
            except Exception:
                pass
        for _ in range(3):
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.03)

    def task1_task2_handoff(self):
        """Keep localization alive while proving all motion authority is idle."""
        self.publish_status(
            "task1", "task2_handoff",
            "cancelling navigation and waiting for a stationary base")
        self.safe_stop(cancel_navigation=True)
        timeout = float(rospy.get_param("~task1_task2_handoff_timeout_sec", 5.0))
        stable_required = float(rospy.get_param(
            "~task1_task2_handoff_stable_sec", 0.5))
        deadline = time.monotonic() + timeout
        stable_since = None
        stationary_ready = False
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            self.safe_stop(cancel_navigation=True)
            state = self.move_base.get_state()
            with self.lock:
                twist = self.base_twist
                odom_age = time.monotonic() - self.qr_odom_received_at
            idle = state not in (GoalStatus.PENDING, GoalStatus.ACTIVE)
            stopped = (twist is not None and odom_age <= 0.5 and
                       base_is_stopped(*twist))
            if idle and stopped:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= stable_required:
                    stationary_ready = True
                    break
            else:
                stable_since = None
            rospy.sleep(0.05)
        if not stationary_ready:
            raise StageError(
                "task1->task2 handoff did not reach {:.1f}s stationary idle state".format(
                    stable_required))

        self.publish_status(
            "task1", "task2_costmap_refreshing",
            "clearing QR-scan obstacle history before task2 coverage navigation")
        try:
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=2.0)
            rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise StageError(
                "task1->task2 costmap refresh failed: {}".format(exc))
        cleared_at = time.monotonic()
        refresh_deadline = cleared_at + 2.0
        while time.monotonic() < refresh_deadline and not rospy.is_shutdown():
            self.check_abort()
            with self.lock:
                scan_fresh = self.handoff_scan_received_at > cleared_at
                costmap_fresh = self.handoff_costmap_received_at > cleared_at
            if scan_fresh and costmap_fresh:
                break
            rospy.sleep(0.05)
        else:
            raise StageError(
                "task1->task2 costmap refresh produced no fresh scan/costmap snapshot")
        self.safe_stop(cancel_navigation=True)
        self.publish_status(
            "task1", "task2_handoff_ready",
            "move_base idle; fresh costmap; preserving AMCL state")

    def production_task4_handoff(self, source_stage):
        """Resume physical navigation without resetting the current factory pose."""
        source_stage = str(source_stage or "task3").strip().lower()
        self.publish_status(
            source_stage, "task4_handoff",
            "preserving AMCL pose and preparing physical navigation")
        self.safe_stop(cancel_navigation=True)
        timeout = float(rospy.get_param(
            "~task3_task4_handoff_timeout_sec", 5.0))
        stable_required = float(rospy.get_param(
            "~task3_task4_handoff_stable_sec", 0.5))
        deadline = time.monotonic() + timeout
        stable_since = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            self.safe_stop(cancel_navigation=True)
            state = self.move_base.get_state()
            with self.lock:
                twist = self.base_twist
                odom_age = time.monotonic() - self.qr_odom_received_at
            idle = state not in (GoalStatus.PENDING, GoalStatus.ACTIVE)
            stopped = (twist is not None and odom_age <= 0.5 and
                       base_is_stopped(*twist))
            if idle and stopped:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= stable_required:
                    break
            else:
                stable_since = None
            rospy.sleep(0.05)
        else:
            raise StageError(
                "{}->task4 handoff did not reach {:.1f}s stationary idle state".format(
                    source_stage, stable_required))

        try:
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=2.0)
            rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise StageError(
                "{}->task4 costmap refresh failed: {}".format(source_stage, exc))
        cleared_at = time.monotonic()
        refresh_deadline = cleared_at + 2.0
        while time.monotonic() < refresh_deadline and not rospy.is_shutdown():
            self.check_abort()
            with self.lock:
                scan_fresh = self.handoff_scan_received_at > cleared_at
                costmap_fresh = self.handoff_costmap_received_at > cleared_at
            if scan_fresh and costmap_fresh:
                break
            rospy.sleep(0.05)
        else:
            raise StageError(
                "{}->task4 costmap refresh produced no fresh scan/costmap snapshot".format(
                    source_stage))
        self.safe_stop(cancel_navigation=True)
        self.publish_status(
            source_stage, "task4_handoff_ready",
            "current AMCL pose preserved; fresh costmap; task4 may navigate")

    def start_child(self, key, package, launch_file, args=None, reuse_running=False):
        existing = self.children.get(key)
        if reuse_running and existing is not None and existing.poll() is None:
            rospy.loginfo("reusing prewarmed child: %s", key)
            return existing
        self.stop_child(key)
        command = ["roslaunch", package, launch_file]
        for name, value in (args or {}).items():
            command.append("{}:={}".format(name, str(value).lower() if isinstance(value, bool) else value))
        if bool_param("~quiet_child_logs", True):
            log_dir = os.path.expanduser(rospy.get_param(
                "~child_log_dir", "~/.ros/src3_competition_children"))
            os.makedirs(log_dir, exist_ok=True)
            log_path = os.path.join(
                log_dir, "{}_{}.log".format(key, int(time.time())))
            log_handle = open(log_path, "ab", buffering=0)
            self.child_log_handles[key] = log_handle
            rospy.loginfo("starting %s (details: %s)", key, log_path)
            self.children[key] = subprocess.Popen(
                command, start_new_session=True,
                stdout=log_handle, stderr=subprocess.STDOUT)
        else:
            rospy.loginfo("starting child: %s", " ".join(command))
            self.children[key] = subprocess.Popen(
                command, start_new_session=True)
        return self.children[key]

    def stop_child(self, key):
        proc = self.children.pop(key, None)
        log_handle = self.child_log_handles.pop(key, None)
        if proc is None or proc.poll() is not None:
            if log_handle is not None:
                log_handle.close()
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=5.0)
        except Exception:
            try:
                proc.terminate()
                proc.wait(timeout=2.0)
            except Exception:
                proc.kill()
        finally:
            if log_handle is not None:
                log_handle.close()

    def stop_all_children(self):
        for key in list(self.children):
            self.stop_child(key)

    def wait_loop(self, timeout, predicate, child_key=None):
        deadline = time.time() + timeout if timeout > 0 else None
        while not rospy.is_shutdown():
            self.check_abort()
            result = predicate()
            if result:
                return result
            if child_key and child_key in self.children:
                code = self.children[child_key].poll()
                if code is not None:
                    raise StageError("{} exited unexpectedly with code {}".format(child_key, code))
            if deadline and time.time() >= deadline:
                raise StageError("stage timed out after {:.1f}s".format(timeout))
            rospy.sleep(0.1)

    def navigate(self, x, y, yaw, stage, timeout_sec=None, status_state="navigating"):
        timeout = float(
            timeout_sec
            if timeout_sec is not None
            else rospy.get_param("~move_base_timeout_sec", 90.0)
        )
        if not self.move_base.wait_for_server(rospy.Duration(min(timeout, 30.0))):
            raise StageError("move_base action server unavailable")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        self.publish_status(
            stage,
            status_state,
            "goal x={:.3f} y={:.3f} yaw={:.3f}".format(x, y, yaw),
        )
        self.move_base.send_goal(goal)
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            self.check_abort()
            state = self.move_base.get_state()
            if state not in (GoalStatus.PENDING, GoalStatus.ACTIVE):
                if state == GoalStatus.SUCCEEDED:
                    return
                raise StageError("move_base failed with state {}".format(state))
            rospy.sleep(0.1)
        self.move_base.cancel_goal()
        raise StageError("move_base goal timed out")

    def announce(self, event, item="", workshop="", decision="", text="", wait=True):
        service = rospy.get_param("~announce_service", "/competition_speech/announce")
        try:
            rospy.wait_for_service(service, timeout=5.0)
            response = rospy.ServiceProxy(service, Announce)(
                event, item, workshop, decision, text, bool(wait)
            )
        except (rospy.ROSException, rospy.ServiceException) as exc:
            raise StageError("speech service failed: {}".format(exc))
        if not response.success:
            raise StageError("speech rejected: {}".format(response.message))

    def navigate_to_qr_area(self):
        """Run the untouched simple_navigator and observe its move_base result."""
        timeout = float(rospy.get_param("~qr_navigation_timeout_sec", 120.0))
        self.safe_stop(cancel_navigation=True)
        with self.lock:
            self.qr_navigation_watching = True
            self.qr_navigation_goal_id = ""
            self.qr_navigation_result = None
        self.publish_status(
            "task1",
            "navigating",
            "running roslaunch simple_navigator navigate.launch",
        )
        try:
            self.start_child("qr_navigator", "simple_navigator", "navigate.launch")
            deadline = time.time() + timeout
            child_exited_at = None
            while not rospy.is_shutdown():
                self.check_abort()
                with self.lock:
                    result = self.qr_navigation_result
                if result is not None:
                    if result == GoalStatus.SUCCEEDED:
                        break
                    raise StageError(
                        "simple_navigator move_base result state={}".format(result)
                    )

                proc = self.children.get("qr_navigator")
                if proc and proc.poll() is not None:
                    if child_exited_at is None:
                        child_exited_at = time.time()
                    elif time.time() - child_exited_at >= 1.0:
                        raise StageError(
                            "simple_navigator exited without a move_base result (code {})".format(
                                proc.returncode
                            )
                        )
                if time.time() >= deadline:
                    raise StageError(
                        "simple_navigator timed out after {:.1f}s".format(timeout)
                    )
                rospy.sleep(0.1)
        finally:
            with self.lock:
                self.qr_navigation_watching = False
            self.stop_child("qr_navigator")
            self.safe_stop(cancel_navigation=True)

        self.publish_status(
            "task1",
            "qr_area_arrived",
            "simple_navigator reached the configured QR-area waypoint",
        )

    def _qr_count(self):
        with self.lock:
            return len(self.qr_items)

    def _check_qr_decoder(self):
        proc = self.children.get("qr_decoder")
        if proc and proc.poll() is not None:
            raise StageError(
                "QR decoder exited unexpectedly with code {}".format(proc.returncode)
            )

    def _fresh_qr_odom_yaw(self, stale_sec):
        with self.lock:
            yaw = self.qr_odom_yaw
            received_at = self.qr_odom_received_at
        if yaw is None or time.monotonic() - received_at > stale_sec:
            raise StageError(
                "QR scan odometry is missing or stale for more than {:.2f}s".format(
                    stale_sec
                )
            )
        return yaw

    def _wait_for_qr_odom(self, wait_sec, stale_sec):
        deadline = time.monotonic() + wait_sec
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            self._check_qr_decoder()
            with self.lock:
                received_at = self.qr_odom_received_at
                yaw = self.qr_odom_yaw
            if yaw is not None and time.monotonic() - received_at <= stale_sec:
                return yaw
            rospy.sleep(0.05)
        raise StageError("QR scan did not receive fresh odometry within {:.1f}s".format(wait_sec))

    def _fresh_qr_map_yaw(self, stale_sec):
        with self.lock:
            map_yaw = self.qr_map_yaw
            map_odom_yaw = self.qr_map_odom_yaw
            odom_yaw = self.qr_odom_yaw
            odom_received_at = self.qr_odom_received_at
        if map_yaw is None:
            raise StageError(
                "QR scan has not received an AMCL map orientation"
            )
        # AMCL may intentionally stop publishing while the base is stationary.
        # Keep its latest map-to-odom heading anchor and propagate the current
        # heading with fresh odometry instead of treating the cached pose as bad.
        if map_odom_yaw is None or odom_yaw is None:
            return map_yaw
        if time.monotonic() - odom_received_at > stale_sec:
            raise StageError(
                "QR scan odometry is stale while deriving the map orientation"
            )
        return normalize_angle(
            map_yaw + normalize_angle(odom_yaw - map_odom_yaw)
        )

    def _align_qr_scan_start_yaw(self):
        target_yaw = float(
            rospy.get_param("~qr_scan_start_yaw_rad", math.pi / 2.0)
        )
        tolerance = abs(float(
            rospy.get_param("~qr_scan_start_yaw_tolerance_rad", math.radians(2.0))
        ))
        max_speed = abs(float(
            rospy.get_param("~qr_scan_align_max_angular_speed", 0.55)
        ))
        min_speed = abs(float(
            rospy.get_param("~qr_scan_align_min_angular_speed", 0.10)
        ))
        timeout = float(rospy.get_param("~qr_scan_align_timeout_sec", 12.0))
        stale_sec = float(rospy.get_param("~qr_odom_stale_sec", 0.5))
        wait_deadline = time.monotonic() + min(timeout, 3.0)
        while not rospy.is_shutdown():
            try:
                self._fresh_qr_map_yaw(stale_sec)
                break
            except StageError:
                if time.monotonic() >= wait_deadline:
                    raise
                rospy.sleep(0.05)

        self.publish_status(
            "task1",
            "qr_aligning_start_yaw",
            "aligning continuous scan start to map yaw {:.1f}deg".format(
                math.degrees(target_yaw)
            ),
        )
        deadline = time.monotonic() + timeout
        stable_cycles = 0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            self._check_qr_decoder()
            error = normalize_angle(target_yaw - self._fresh_qr_map_yaw(stale_sec))
            if abs(error) <= tolerance:
                stable_cycles += 1
                self.cmd_pub.publish(Twist())
                if stable_cycles >= 3:
                    self.safe_stop()
                    self.publish_status(
                        "task1",
                        "qr_start_yaw_ready",
                        "continuous scan starts at map yaw {:.1f}deg".format(
                            math.degrees(target_yaw)
                        ),
                    )
                    return
            else:
                stable_cycles = 0
                command_speed = min(max_speed, max(min_speed, abs(error) * 0.9))
                twist = Twist()
                twist.angular.z = command_speed if error > 0.0 else -command_speed
                self.cmd_pub.publish(twist)
            rospy.sleep(0.05)
        self.safe_stop()
        raise StageError("QR scan could not align to its 90-degree start yaw")

    @staticmethod
    def _qr_percentile(values, fraction):
        ordered = sorted(values)
        if not ordered:
            return None
        index = int(round((len(ordered) - 1) * float(fraction)))
        return ordered[max(0, min(len(ordered) - 1, index))]

    def _qr_wall_distance(self, scan, center_angle, half_angle):
        projected = []
        angle = float(scan.angle_min)
        minimum = max(0.05, float(scan.range_min))
        maximum = float(scan.range_max)
        for distance in scan.ranges:
            delta = normalize_angle(angle - center_angle)
            if (
                abs(delta) <= half_angle
                and math.isfinite(distance)
                and minimum <= distance <= maximum
            ):
                projected.append(float(distance) * math.cos(delta))
            angle += float(scan.angle_increment)
        if len(projected) < 5:
            return None
        return self._qr_percentile(projected, 0.38)

    def _center_qr_scan_pose(self):
        if not bool_param("~qr_center_enabled", True):
            return False
        timeout = float(rospy.get_param("~qr_center_timeout_sec", 5.0))
        tolerance = abs(float(rospy.get_param("~qr_center_tolerance_m", 0.035)))
        max_speed = abs(float(rospy.get_param("~qr_center_max_speed", 0.08)))
        gain = abs(float(rospy.get_param("~qr_center_gain", 0.75)))
        max_move = abs(float(rospy.get_param("~qr_center_max_move_m", 0.16)))
        hard_clearance = abs(float(
            rospy.get_param("~qr_center_hard_clearance_m", 0.20)
        ))
        half_angle = math.radians(abs(float(
            rospy.get_param("~qr_center_sector_half_deg", 28.0)
        )))
        pair_min = float(rospy.get_param("~qr_center_pair_sum_min_m", 0.65))
        pair_max = float(rospy.get_param("~qr_center_pair_sum_max_m", 2.8))
        error_limit = abs(float(
            rospy.get_param("~qr_center_axis_error_limit_m", 0.32)
        ))

        ready_deadline = time.monotonic() + 2.0
        while time.monotonic() < ready_deadline and not rospy.is_shutdown():
            with self.lock:
                ready = (
                    self.qr_scan is not None
                    and self.qr_odom_position is not None
                    and time.monotonic() - self.qr_scan_received_at <= 0.7
                    and time.monotonic() - self.qr_odom_received_at <= 0.7
                )
            if ready:
                break
            rospy.sleep(0.05)
        else:
            self.publish_status(
                "task1", "qr_center_skipped", "no fresh lidar/odometry snapshot"
            )
            return False

        with self.lock:
            start_position = self.qr_odom_position
        self.publish_status(
            "task1", "qr_centering", "centering in the QR room with opposite walls"
        )
        deadline = time.monotonic() + timeout
        stable_cycles = 0
        centered = False
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            self._check_qr_decoder()
            with self.lock:
                scan = self.qr_scan
                position = self.qr_odom_position
                scan_age = time.monotonic() - self.qr_scan_received_at
                odom_age = time.monotonic() - self.qr_odom_received_at
            if scan is None or position is None or scan_age > 0.7 or odom_age > 0.7:
                self.cmd_pub.publish(Twist())
                rospy.sleep(0.05)
                continue

            moved = math.hypot(
                position[0] - start_position[0], position[1] - start_position[1]
            )
            if moved >= max_move:
                rospy.logwarn("QR centering move limit reached: %.3fm", moved)
                break

            front = self._qr_wall_distance(scan, 0.0, half_angle)
            left = self._qr_wall_distance(scan, math.pi / 2.0, half_angle)
            rear = self._qr_wall_distance(scan, math.pi, half_angle)
            right = self._qr_wall_distance(scan, -math.pi / 2.0, half_angle)
            if any(value is None for value in (front, left, rear, right)):
                self.cmd_pub.publish(Twist())
                rospy.sleep(0.05)
                continue

            valid_x = pair_min <= front + rear <= pair_max
            valid_y = pair_min <= left + right <= pair_max
            error_x = 0.5 * (front - rear) if valid_x else 0.0
            error_y = 0.5 * (left - right) if valid_y else 0.0
            if abs(error_x) > error_limit:
                valid_x = False
                error_x = 0.0
            if abs(error_y) > error_limit:
                valid_y = False
                error_y = 0.0
            if not valid_x and not valid_y:
                rospy.logwarn("QR centering wall pairs are unreliable; using current pose")
                break

            if ((not valid_x or abs(error_x) <= tolerance)
                    and (not valid_y or abs(error_y) <= tolerance)):
                stable_cycles += 1
                if stable_cycles >= 5:
                    centered = True
                    break
            else:
                stable_cycles = 0

            vx = max(-max_speed, min(max_speed, gain * error_x))
            vy = max(-max_speed, min(max_speed, gain * error_y))
            if (vx > 0.0 and front <= hard_clearance) or (
                    vx < 0.0 and rear <= hard_clearance):
                vx = 0.0
            if (vy > 0.0 and left <= hard_clearance) or (
                    vy < 0.0 and right <= hard_clearance):
                vy = 0.0
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            self.cmd_pub.publish(twist)
            rospy.sleep(1.0 / 15.0)

        self.safe_stop()
        self.publish_status(
            "task1",
            "qr_center_ready" if centered else "qr_center_best_effort",
            "QR-room centering finished before continuous scan",
        )
        return centered

    def _settle_for_qr(self, duration, expected_count, scan_deadline, stale_sec):
        self.safe_stop()
        settle_deadline = min(scan_deadline, time.monotonic() + duration)
        while time.monotonic() < settle_deadline and not rospy.is_shutdown():
            self.check_abort()
            self._check_qr_decoder()
            self._fresh_qr_odom_yaw(stale_sec)
            rospy.sleep(0.05)
        return self._qr_count() >= expected_count

    def _rotate_qr_step(
        self,
        angle,
        speed,
        direction,
        stale_sec,
        scan_deadline,
        step_margin,
    ):
        tracker = DirectedYawAccumulator(direction=direction)
        tracker.reset(self._fresh_qr_odom_yaw(stale_sec))
        step_deadline = min(
            scan_deadline,
            time.monotonic() + angle / speed + step_margin,
        )
        twist = Twist()
        twist.angular.z = speed * direction
        while tracker.progress < angle and not rospy.is_shutdown():
            self.check_abort()
            self._check_qr_decoder()
            if time.monotonic() >= step_deadline:
                raise StageError(
                    "QR scan failed to rotate {:.1f} degrees before step timeout".format(
                        math.degrees(angle)
                    )
                )
            yaw = self._fresh_qr_odom_yaw(stale_sec)
            if tracker.update(yaw) >= angle:
                break
            self.cmd_pub.publish(twist)
            rospy.sleep(0.05)
        self.safe_stop()

    def _return_qr_to_yaw(self, target_yaw, speed, tolerance, stale_sec, timeout):
        min_speed = min(speed, abs(float(
            rospy.get_param("~qr_final_return_min_angular_speed", 0.25)
        )))
        slowdown_angle = max(tolerance, abs(float(
            rospy.get_param(
                "~qr_final_return_slowdown_angle_rad", math.radians(25.0)
            )
        )))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            self._check_qr_decoder()
            current_yaw = self._fresh_qr_odom_yaw(stale_sec)
            error = normalize_angle(target_yaw - current_yaw)
            if abs(error) <= tolerance:
                self.safe_stop()
                return
            command_speed = speed
            if abs(error) < slowdown_angle:
                command_speed = max(
                    min_speed,
                    speed * abs(error) / slowdown_angle,
                )
            twist = Twist()
            twist.angular.z = command_speed if error > 0.0 else -command_speed
            self.cmd_pub.publish(twist)
            rospy.sleep(0.05)
        self.safe_stop()
        raise StageError("QR scan failed to return to its original final yaw")

    def scan_qr_at_current_pose(self, status_state):
        """Continuously scan from a known map yaw until all physical URLs are seen."""
        expected_count = int(rospy.get_param("~qr_expected_count", 3))
        speed = abs(float(rospy.get_param("~qr_scan_angular_speed", 0.22)))
        min_speed = abs(float(rospy.get_param("~qr_scan_min_angular_speed", 0.08)))
        acceleration = abs(float(rospy.get_param("~qr_scan_angular_accel", 0.35)))
        direction = 1.0 if float(rospy.get_param("~qr_scan_direction", 1.0)) >= 0 else -1.0
        scan_timeout = max(0.0, float(rospy.get_param("~qr_scan_timeout_sec", 0.0)))
        stale_sec = float(rospy.get_param("~qr_odom_stale_sec", 0.5))
        odom_wait_sec = float(rospy.get_param("~qr_odom_wait_sec", 2.0))
        if expected_count <= 0 or speed <= 0.0 or stale_sec <= 0.0:
            raise StageError("QR scan motion parameters must be positive")

        self._align_qr_scan_start_yaw()
        self._center_qr_scan_pose()
        # Translating does not intentionally change yaw, but AMCL may settle by
        # a few degrees while centering. Recheck the required 90-degree start.
        self._align_qr_scan_start_yaw()
        initial_yaw = self._wait_for_qr_odom(odom_wait_sec, stale_sec)
        with self.lock:
            self.qr_collecting = True
        tracker = DirectedYawAccumulator(direction=direction)
        tracker.reset(initial_yaw)
        started_at = time.monotonic()
        reported_rounds = 0
        self.publish_status(
            "task1",
            status_state,
            "[QR] scan started: count={}/{} speed={:.2f}rad/s".format(
                self._qr_count(), expected_count, speed
            ),
        )
        try:
            while self._qr_count() < expected_count and not rospy.is_shutdown():
                self.check_abort()
                self._check_qr_decoder()
                elapsed = time.monotonic() - started_at
                if scan_timeout > 0.0 and elapsed >= scan_timeout:
                    raise StageError(
                        "continuous QR scan timed out with {}/{} unique URLs".format(
                            self._qr_count(), expected_count
                        )
                    )
                yaw = self._fresh_qr_odom_yaw(stale_sec)
                progress = tracker.update(yaw)
                rounds = int(progress / (2.0 * math.pi))
                if rounds > reported_rounds:
                    reported_rounds = rounds
                    self.publish_status(
                        "task1",
                        status_state,
                        "[QR] round {} complete: count={}/{}".format(
                            rounds, self._qr_count(), expected_count
                        ),
                    )
                command_speed = min(
                    speed,
                    max(min_speed, min_speed + acceleration * elapsed),
                )
                twist = Twist()
                twist.angular.z = direction * command_speed
                self.cmd_pub.publish(twist)
                rospy.sleep(0.05)
        finally:
            self.safe_stop()

        if rospy.is_shutdown():
            raise Aborted("competition stopped during continuous QR scan")
        self.publish_status(
            "task1",
            "qr_continuous_scan_completed",
            "[QR] scan complete: {} unique URLs".format(
                self._qr_count()
            ),
        )
        return True

    def _reset_task1_reasoning(self, instruction):
        with self.lock:
            self.task1_llm_generation += 1
            self.task1_instruction = str(instruction or "").strip()
            self.task1_llm_thread = None
            self.task1_llm_done = threading.Event()
            self.task1_llm_result = None
            self.task1_llm_error = ""
            self.task1_llm_items = []

    def _factory_ocr_args(self):
        return {
            "start_camera": False,
            "start_competition_speech": False,
            "start_viewer": self.debug,
            "recognition_mode": "ppocr_rknn_system",
            # The same RKNN process serves both parking targets.  Mission-level
            # filtering below decides which category may trigger navigation.
            "target_category": "",
            "enable_speech": False,
            "required": True,
        }

    def _factory_navigator_args(self, center_only, start_paused,
                                preferred_coverage_anchor=0):
        return {
            "trigger_mode": "vision",
            "vision_topic": "/vision/detected",
            "target_topic": "/vision/target",
            "trigger_service": self.trigger_service_name,
            "start_paused": start_paused,
            "start_navigation_service": (
                "/vision_triggered_navigator/start_navigation"
            ),
            "coverage_observation_topic": rospy.get_param(
                "~coverage_observation_topic",
                "/vision_triggered_navigator/coverage_observation",
            ),
            "preferred_coverage_anchor": int(preferred_coverage_anchor),
            "publish_initial_pose": (
                False if self.mode == "task1_task2" else
                bool_param("~navigator_publish_initial_pose", False)
            ),
            "navigate_to_end_after_trigger": False,
            "coverage_search_mode": True,
            "target_center_steering_sign": rospy.get_param(
                "~target_center_steering_sign", -1.0),
            "parking_recenter_lateral_sign": rospy.get_param(
                "~parking_recenter_lateral_sign", 1.0),
            "camera_boresight_yaw_offset": rospy.get_param(
                "~camera_boresight_yaw_offset", 0.0),
            "camera_horizontal_fov_deg": rospy.get_param(
                "~camera_horizontal_fov_deg", 70.0),
            "camera_bearing_sign": rospy.get_param(
                "~camera_bearing_sign", -1.0),
            "center_only": center_only,
            "validate_parking_box": not center_only,
            "max_coverage_anchors": int(rospy.get_param(
                "~max_coverage_anchors", 0)),
            "vision_offset": rospy.get_param("~task2_vision_offset", 0.4),
            "parking_goal_offset": rospy.get_param(
                "~parking_goal_offset", 0.26),
            "parking_staging_offset": rospy.get_param(
                "~parking_staging_offset", 0.55),
            "parking_staging_timeout_sec": rospy.get_param(
                "~parking_staging_timeout_sec", 20.0),
            "parking_staging_position_tolerance": rospy.get_param(
                "~parking_staging_position_tolerance", 0.10),
            "parking_staging_yaw_tolerance": rospy.get_param(
                "~parking_staging_yaw_tolerance", 0.10),
            "parking_docking_timeout_sec": rospy.get_param(
                "~parking_docking_timeout_sec", 15.0),
            "parking_dock_max_x": rospy.get_param("~parking_dock_max_x", 0.12),
            "parking_dock_max_y": rospy.get_param("~parking_dock_max_y", 0.08),
            "parking_dock_max_yaw": rospy.get_param(
                "~parking_dock_max_yaw", 0.22),
            "parking_dock_min_yaw": rospy.get_param(
                "~parking_dock_min_yaw", 0.05),
            "parking_dock_normal_tolerance": rospy.get_param(
                "~parking_dock_normal_tolerance", 0.035),
            "parking_dock_tangent_tolerance": rospy.get_param(
                "~parking_dock_tangent_tolerance", 0.015),
            "parking_dock_yaw_tolerance": rospy.get_param(
                "~parking_dock_yaw_tolerance", 0.05),
            "parking_dock_translation_yaw_gate": rospy.get_param(
                "~parking_dock_translation_yaw_gate", 0.22),
            "parking_dock_forward_yaw_gate": rospy.get_param(
                "~parking_dock_forward_yaw_gate", 0.14),
            "parking_dock_forward_tangent_gate": rospy.get_param(
                "~parking_dock_forward_tangent_gate", 0.12),
            "parking_min_wall_distance": rospy.get_param(
                "~parking_min_wall_distance", 0.19),
            "parking_lidar_stop_distance": rospy.get_param(
                "~parking_lidar_stop_distance", 0.15),
            "parking_recenter_tolerance": rospy.get_param(
                "~parking_recenter_tolerance", 0.04),
            "parking_recenter_timeout_sec": rospy.get_param(
                "~parking_recenter_timeout_sec", 4.0),
            "parking_recenter_initial_wait_sec": rospy.get_param(
                "~parking_recenter_initial_wait_sec", 1.0),
            "parking_recenter_lateral_kp": rospy.get_param(
                "~parking_recenter_lateral_kp", 0.16),
            "parking_recenter_min_lateral": rospy.get_param(
                "~parking_recenter_min_lateral", 0.015),
            "parking_recenter_max_lateral": rospy.get_param(
                "~parking_recenter_max_lateral", 0.065),
            "parking_recenter_max_travel": rospy.get_param(
                "~parking_recenter_max_travel", 0.30),
            "parking_recenter_yaw_kp": rospy.get_param(
                "~parking_recenter_yaw_kp", 1.0),
            "parking_recenter_yaw_tolerance_deg": rospy.get_param(
                "~parking_recenter_yaw_tolerance_deg", 2.0),
            "parking_recenter_max_yaw": rospy.get_param(
                "~parking_recenter_max_yaw", 0.18),
            "parking_recenter_stable_sec": rospy.get_param(
                "~parking_recenter_stable_sec", 0.25),
            "parking_recenter_required_hits": rospy.get_param(
                "~parking_recenter_required_hits", 3),
            "parking_recenter_side_half_angle_deg": rospy.get_param(
                "~parking_recenter_side_half_angle_deg", 25.0),
            "parking_recenter_side_stop_m": rospy.get_param(
                "~parking_recenter_side_stop_m", 0.18),
            "parking_recenter_side_slow_m": rospy.get_param(
                "~parking_recenter_side_slow_m", 0.28),
            "parking_wall_fit_half_angle_deg": rospy.get_param(
                "~parking_wall_fit_half_angle_deg", 35.0),
            "parking_wall_fit_min_points": rospy.get_param(
                "~parking_wall_fit_min_points", 12),
            "parking_wall_fit_min_span": rospy.get_param(
                "~parking_wall_fit_min_span", 0.25),
            "parking_wall_fit_near_min_span": rospy.get_param(
                "~parking_wall_fit_near_min_span", 0.18),
            "parking_wall_fit_max_distance_jump": rospy.get_param(
                "~parking_wall_fit_max_distance_jump", 0.05),
            "parking_wall_fit_max_normal_jump_deg": rospy.get_param(
                "~parking_wall_fit_max_normal_jump_deg", 8.0),
            "parking_wall_fit_max_residual": rospy.get_param(
                "~parking_wall_fit_max_residual", 0.015),
            "parking_wall_fit_max_normal_error_deg": rospy.get_param(
                "~parking_wall_fit_max_normal_error_deg", 20.0),
            "parking_wall_fit_grace_sec": rospy.get_param(
                "~parking_wall_fit_grace_sec", 1.5),
            "parking_wall_fit_filter_alpha": rospy.get_param(
                "~parking_wall_fit_filter_alpha", 0.45),
            "parking_normal_offset": rospy.get_param("~parking_normal_offset", 0.0),
            "parking_tangent_offset": rospy.get_param(
                "~parking_tangent_offset", 0.0),
            "parking_box_width": rospy.get_param("~parking_box_width", 0.50),
            "parking_box_depth": rospy.get_param("~parking_box_depth", 0.50),
            "parking_xy_tolerance": rospy.get_param("~parking_xy_tolerance", 0.04),
            "parking_yaw_tolerance": rospy.get_param("~parking_yaw_tolerance", 0.06),
            "target_center_coarse_step_deg": rospy.get_param(
                "~target_center_coarse_step_deg", 4.0),
            "target_center_fine_step_deg": rospy.get_param(
                "~target_center_fine_step_deg", 2.0),
            "target_center_start_speed": rospy.get_param(
                "~target_center_start_speed", 0.20),
            "target_center_step_max_speed": rospy.get_param(
                "~target_center_max_speed", 0.35),
            "target_center_timeout_sec": rospy.get_param(
                "~target_center_timeout_sec", 12.0),
            "coverage_scan_step_deg": rospy.get_param(
                "~coverage_scan_step_deg", 20.0),
            "coverage_scan_angular_speed": rospy.get_param(
                "~coverage_scan_angular_speed", 0.35),
            "coverage_scan_dwell_sec": rospy.get_param(
                "~coverage_scan_dwell_sec", 0.65),
            "coverage_candidate_hold_sec": rospy.get_param(
                "~coverage_candidate_hold_sec", 1.2),
            "coverage_scan_max_dwell_sec": rospy.get_param(
                "~coverage_scan_max_dwell_sec", 2.0),
            "coverage_scan_pose_timeout_sec": rospy.get_param(
                "~coverage_scan_pose_timeout_sec", 0.5),
            "coverage_goal_soft_timeout_sec": rospy.get_param(
                "~coverage_goal_soft_timeout_sec", 25.0),
            "coverage_goal_hard_timeout_sec": rospy.get_param(
                "~coverage_goal_hard_timeout_sec", 40.0),
            "coverage_goal_progress_window_sec": rospy.get_param(
                "~coverage_goal_progress_window_sec", 5.0),
            "coverage_goal_min_progress": rospy.get_param(
                "~coverage_goal_min_progress", 0.03),
            "coverage_anchor_position_tolerance": rospy.get_param(
                "~coverage_anchor_position_tolerance", 0.28),
            "coverage_anchor_yaw_tolerance_deg": rospy.get_param(
                "~coverage_anchor_yaw_tolerance_deg", 5.0),
            "coverage_anchor_yaw_hold_sec": rospy.get_param(
                "~coverage_anchor_yaw_hold_sec", 0.20),
            "coverage_anchor_yaw_timeout_sec": rospy.get_param(
                "~coverage_anchor_yaw_timeout_sec", 12.0),
            "coverage_no_progress_timeout_sec": rospy.get_param(
                "~coverage_no_progress_timeout_sec", 5.5),
            "coverage_fallback_enabled": bool_param(
                "~coverage_fallback_enabled", True),
            "coverage_fallback_make_plan_tolerance_m": rospy.get_param(
                "~coverage_fallback_make_plan_tolerance_m", 0.12),
        }

    def _factory_ocr_is_running(self):
        proc = self.children.get("factory_ocr")
        return proc is not None and proc.poll() is None

    def _prewarm_task2_stack(self):
        if "task2" not in stage_sequence(self.mode, self.enable_simulation):
            return False
        if not self._factory_ocr_is_running():
            self.ocr_target = self.category
            self.ocr_filter.reset()
            self.ocr_last_message_at = 0.0
            self.publish_status(
                "task1",
                "prewarming_task2",
                "QR scan completed; preparing warehouse OCR and navigator",
            )
            self.start_child(
                "factory_ocr",
                "factory_sign_ppocr_rknn_test",
                "factory_sign_ppocr_rknn_test.launch",
                self._factory_ocr_args(),
            )
        navigator = self.children.get("factory_navigator")
        if navigator is None or navigator.poll() is not None:
            self.start_child(
                "factory_navigator",
                "vision_triggered_navigator",
                "vision_triggered_navigator.launch",
                self._factory_navigator_args(
                    bool_param("~task2_center_only", False),
                    True,
                ),
            )
        return True

    def _prewarm_second_parking_stack(self):
        """Load the second OCR/navigator during the first result announcement."""
        simulation_category = normalize_category(self.sim_category)
        if (not simulation_category or
                simulation_category not in CATEGORY_LABELS):
            return False
        preferred_anchor, preferred_source = self._second_parking_anchor_choice()
        physical_category = self.category
        self.ocr_target = None
        self.factory_parking_index = 0
        try:
            self.category = simulation_category
            self.start_child(
                "factory_ocr",
                "factory_sign_ppocr_rknn_test",
                "factory_sign_ppocr_rknn_test.launch",
                self._factory_ocr_args(),
                reuse_running=True,
            )
            self.start_child(
                "factory_navigator",
                "vision_triggered_navigator",
                "vision_triggered_navigator.launch",
                self._factory_navigator_args(
                    bool_param("~task2_center_only", False),
                    True,
                    preferred_anchor,
                ),
                reuse_running=True,
            )
        finally:
            self.category = physical_category
        self.publish_status(
            "task2", "second_parking_prewarming",
            "second OCR/navigation loading during first announcement; "
            "preferred_anchor={} source={}".format(
                preferred_anchor, preferred_source))
        return True

    def _second_parking_anchor_choice(self):
        """Choose a cached target point, otherwise continue after parking 1."""
        if self.cached_sim_observation:
            anchor = int(self.cached_sim_observation.get("anchor_index", 0))
            if anchor > 0:
                return anchor, "cached_simulation"

        physical_anchor = int(self.first_parking_anchor_index)
        if physical_anchor <= 0:
            return 0, "default_start"
        patrol_points = rospy.get_param(
            "/vision_triggered_navigator/patrol_points", [])
        point_count = len(patrol_points) if isinstance(patrol_points, list) else 0
        if point_count <= 0:
            point_count = 9
        return physical_anchor % point_count + 1, "after_physical_{}".format(
            physical_anchor)

    def _wait_task2_prewarm_ready(self):
        if "task2" not in stage_sequence(self.mode, self.enable_simulation):
            return
        timeout = float(rospy.get_param("~task2_prewarm_timeout_sec", 12.0))
        deadline = time.monotonic() + timeout
        start_service = "/vision_triggered_navigator/start_navigation"
        while time.monotonic() < deadline and not rospy.is_shutdown():
            self.check_abort()
            for key in ("factory_ocr", "factory_navigator"):
                proc = self.children.get(key)
                if proc is None or proc.poll() is not None:
                    raise StageError("task2 prewarm process {} is not running".format(key))
            service_ready = False
            try:
                rospy.wait_for_service(start_service, timeout=0.1)
                service_ready = True
            except rospy.ROSException:
                pass
            if self.ocr_last_message_at and service_ready:
                self.publish_status(
                    "task1",
                    "task2_prewarm_ready",
                    "warehouse OCR and paused navigator are ready before announcement",
                )
                return
            self.safe_stop()
        raise StageError(
            "task2 prewarm was not ready before announcement within {:.1f}s".format(
                timeout
            )
        )

    def _start_task1_reasoning_async(self):
        expected_count = int(rospy.get_param("~qr_expected_count", 3))
        with self.lock:
            if (
                self.task1_llm_thread is not None
                or not self.task1_instruction
                or len(self.qr_items) < expected_count
            ):
                return False
            items = list(self.qr_items.values())[:expected_count]
            if len(items) < 3:
                return False
            instruction = self.task1_instruction
            generation = self.task1_llm_generation
            done_event = self.task1_llm_done
            worker = threading.Thread(
                target=self._task1_reasoning_worker,
                args=(generation, done_event, items, instruction),
                name="task1-llm-reasoning",
                daemon=True,
            )
            self.task1_llm_thread = worker
            self.task1_llm_items = items
        self.publish_status(
            "task1",
            "reasoning_during_qr_scan",
            "three QR items collected; Spark X2 reasoning started in parallel",
        )
        worker.start()
        self._prewarm_task2_stack()
        return True

    def _task1_reasoning_worker(self, generation, done_event, items, instruction):
        result = None
        error = ""
        service = rospy.get_param(
            "~llm_service", "/smart_factory_llm/reason_pickup_order"
        )
        try:
            rospy.wait_for_service(service, timeout=15.0)
            result = rospy.ServiceProxy(service, ReasonPickupOrder)(
                items[0], items[1], items[2], instruction
            )
            if not result.success:
                error = "LLM reasoning failed: {}".format(result.error_message)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            error = "LLM service failed: {}".format(exc)
        except Exception as exc:
            error = "LLM reasoning worker failed: {}".format(exc)
        with self.lock:
            if generation != self.task1_llm_generation:
                return
            self.task1_llm_result = result
            self.task1_llm_error = error
        done_event.set()

    def _wait_task1_reasoning(self):
        with self.lock:
            done_event = self.task1_llm_done
        if not done_event.is_set():
            self.publish_status(
                "task1",
                "waiting_reasoning_at_final_yaw",
                "vehicle is at the final yaw; waiting for Spark X2",
            )
        while not done_event.wait(0.1):
            self.check_abort()
            self.safe_stop()
        with self.lock:
            result = self.task1_llm_result
            error = self.task1_llm_error
            items = list(self.task1_llm_items)
        if error:
            raise StageError(error)
        if result is None:
            raise StageError("LLM reasoning completed without a result")
        return result, items

    # ------------------------------ stages ------------------------------
    def task1(self):
        self.task1_task2_handoff_prepared = False
        with self.lock:
            if self.voice_handshake_error and not self.voice_command_acknowledged:
                self.wakeup_received = False
                self.voice_prompt_started = False
                self.voice_listening = False
                self.voice_command_ack_in_progress = False
                self.voice_handshake_error = ""
        self.publish_status(
            "task1",
            "waiting_voice",
            "waiting for wake word and the physical/simulation category order",
        )
        self.wait_loop(0, self._voice_command_ready)

        category_name = CATEGORY_LABELS[self.category][0]
        sim_category_name = CATEGORY_LABELS[self.sim_category][0]
        instruction = self.question.strip()
        if not instruction:
            instruction = "请取得{}类产品，并领取仿真环境中需要的{}类产品".format(
                category_name,
                sim_category_name,
            )
        self.navigate_to_qr_area()
        # The configured simple_navigator goal has completed, but explicitly
        # revoke all navigation authority before this node can rotate the base.
        self.safe_stop(cancel_navigation=True)

        with self.lock:
            self.qr_items.clear()
        self._reset_task1_reasoning(instruction)
        try:
            self.start_child("qr_decoder", "ucar_2026_competition", "qr_decoder.launch")
            # Decoder startup overlaps alignment/centering, but item acceptance
            # begins only after the robot is centered and facing map yaw 90 deg.
            self.qr_collecting = False
            completed = self.scan_qr_at_current_pose("scanning_qr_primary")

            expected_count = int(rospy.get_param("~qr_expected_count", 3))
            if not completed or self._qr_count() < expected_count:
                raise StageError(
                    "continuous QR scan ended with {}/{} unique URL(s)".format(
                        self._qr_count(), expected_count
                    )
                )
            self.publish_status(
                "task1",
                "qr_scan_completed",
                "collected {} unique QR items".format(self._qr_count()),
            )
        finally:
            self.qr_collecting = False
            self.stop_child("qr_decoder")
            self.safe_stop(cancel_navigation=True)

        self._start_task1_reasoning_async()
        result, items = self._wait_task1_reasoning()
        if normalize_category(result.pickup_major) != self.category:
            raise StageError("LLM physical category does not match voice category")
        if normalize_category(result.sim_major) != self.sim_category:
            raise StageError("LLM simulation category does not match voice category")
        if normalize_category(result.pickup_major) == normalize_category(result.sim_major):
            raise StageError("LLM returned the same category for physical and simulation")

        self.task1_result = {
            "qr_items": items,
            "category": self.category,
            "category_name": category_name,
            "sim_category": self.sim_category,
            "sim_category_name": sim_category_name,
            "pickup_item": result.pickup_item,
            "pickup_major": result.pickup_major,
            "pickup_workshop": result.pickup_workshop,
            "sim_item": result.sim_item,
            "sim_major": result.sim_major,
            "sim_workshop": result.sim_workshop,
            "announcement": result.announcement_full,
        }
        self.result_pub.publish(String(data=json.dumps(self.task1_result, ensure_ascii=False)))
        if "task2" in stage_sequence(self.mode, self.enable_simulation):
            self.task1_task2_handoff()
            self._wait_task2_prewarm_ready()
            self.task1_task2_handoff_prepared = True
        wait_for_announcement = bool_param(
            "~task1_wait_for_announcement_completion", True)
        self.announce(
            "task1", text=result.announcement_full,
            wait=wait_for_announcement)
        if not wait_for_announcement:
            release_delay = max(0.0, float(rospy.get_param(
                "~task1_navigation_release_delay_sec", 0.8)))
            self.publish_status(
                "task1", "announcement_published",
                "task announcement is playing; task2 may overlap after {:.1f}s".format(
                    release_delay))
            deadline = time.monotonic() + release_delay
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                self.check_abort()
                self.safe_stop()
                rospy.sleep(0.05)
        self.publish_status(
            "task1", "completed",
            "voice, QR and reasoning completed; warehouse navigation released")

    def _run_factory_parking(self, item, workshop, announcement_text,
                             parking_index):
        if not self.category:
            raise StageError("task2 target_category is missing")
        if not item:
            raise StageError("task2 target_item is missing")
        center_only = bool_param("~task2_center_only", False)

        ocr_prewarmed = self._factory_ocr_is_running()
        self.factory_parking_index = int(parking_index)
        if parking_index == 1:
            self.first_parking_anchor_index = 0
            self.last_coverage_anchor_index = 0
        self.ocr_target = self.category
        self.ocr_last_logged_category = None
        self.ocr_last_log_signature = None
        self.ocr_filter.reset()
        if parking_index == 1:
            self.sim_preview_filter.reset()
            self.cached_sim_observation = None
        self.ocr_last_message_at = 0.0
        self.vision_trigger_latched = False
        self.trigger_request_pending = False
        self.trigger_request_started_at = 0.0
        self.trigger_service_accepted = False
        self.trigger_acknowledged = False
        self.navigator_status = ""
        preferred_anchor = 0
        preferred_source = "default_start"
        if parking_index == 2:
            preferred_anchor, preferred_source = (
                self._second_parking_anchor_choice())
        if parking_index == 2 and preferred_anchor > 0:
            self.publish_status(
                "task2", "simulation_scan_resume",
                "parking 2 starts at anchor {} source={}".format(
                    preferred_anchor, preferred_source))
        self.publish_status(
            "task2", "searching",
            "parking {}: searching {} with existing 9-point navigation".format(
                parking_index, workshop))
        parking_succeeded = False
        try:
            if not ocr_prewarmed:
                self.start_child(
                    "factory_ocr",
                    "factory_sign_ppocr_rknn_test",
                    "factory_sign_ppocr_rknn_test.launch",
                    self._factory_ocr_args(),
                )
            else:
                self.publish_status(
                    "task2",
                    "ocr_prewarmed",
                    "reusing warehouse OCR started during Spark X2 reasoning",
                )
            self.publish_status("task2", "waiting_ocr", "waiting for first OCR result before motion")
            ocr_ready_deadline = time.time() + float(
                rospy.get_param("~ocr_ready_timeout_sec", 12.0))
            while time.time() < ocr_ready_deadline and not self.ocr_last_message_at:
                self.check_abort()
                proc = self.children.get("factory_ocr")
                if proc and proc.poll() is not None:
                    raise StageError(
                        "factory_ocr exited before ready with code {}".format(proc.returncode))
                rospy.sleep(0.1)
            if not self.ocr_last_message_at:
                raise StageError("factory OCR produced no result before motion timeout")
            self.start_child(
                "factory_navigator",
                "vision_triggered_navigator",
                "vision_triggered_navigator.launch",
                {
                    "trigger_mode": "vision",
                    "vision_topic": "/vision/detected",
                    "target_topic": "/vision/target",
                    "trigger_service": self.trigger_service_name,
                    "start_paused": True,
                    "start_navigation_service": (
                        "/vision_triggered_navigator/start_navigation"
                    ),
                    "coverage_observation_topic": rospy.get_param(
                        "~coverage_observation_topic",
                        "/vision_triggered_navigator/coverage_observation",
                    ),
                    "preferred_coverage_anchor": preferred_anchor,
                    "publish_initial_pose": (
                        False if self.mode == "task1_task2" else
                        bool_param("~navigator_publish_initial_pose", False)),
                    "navigate_to_end_after_trigger": False,
                    "coverage_search_mode": True,
                    "target_center_steering_sign": rospy.get_param(
                        "~target_center_steering_sign", -1.0),
                    "parking_recenter_lateral_sign": rospy.get_param(
                        "~parking_recenter_lateral_sign", 1.0),
                    "camera_boresight_yaw_offset": rospy.get_param(
                        "~camera_boresight_yaw_offset", 0.0),
                    "center_only": center_only,
                    "validate_parking_box": not center_only,
                    "max_coverage_anchors": int(rospy.get_param(
                        "~max_coverage_anchors", 0)),
                    "vision_offset": rospy.get_param("~task2_vision_offset", 0.4),
                    "parking_goal_offset": rospy.get_param(
                        "~parking_goal_offset", 0.26),
                    "parking_staging_offset": rospy.get_param(
                        "~parking_staging_offset", 0.55),
                    "parking_staging_timeout_sec": rospy.get_param(
                        "~parking_staging_timeout_sec", 20.0),
                    "parking_staging_position_tolerance": rospy.get_param(
                        "~parking_staging_position_tolerance", 0.10),
                    "parking_staging_yaw_tolerance": rospy.get_param(
                        "~parking_staging_yaw_tolerance", 0.10),
                    "parking_docking_timeout_sec": rospy.get_param(
                        "~parking_docking_timeout_sec", 15.0),
                    "parking_dock_max_x": rospy.get_param(
                        "~parking_dock_max_x", 0.12),
                    "parking_dock_max_y": rospy.get_param(
                        "~parking_dock_max_y", 0.08),
                    "parking_dock_max_yaw": rospy.get_param(
                        "~parking_dock_max_yaw", 0.22),
                    "parking_dock_min_yaw": rospy.get_param(
                        "~parking_dock_min_yaw", 0.05),
                    "parking_dock_normal_tolerance": rospy.get_param(
                        "~parking_dock_normal_tolerance", 0.035),
                    "parking_dock_tangent_tolerance": rospy.get_param(
                        "~parking_dock_tangent_tolerance", 0.015),
                    "parking_dock_yaw_tolerance": rospy.get_param(
                        "~parking_dock_yaw_tolerance", 0.05),
                    "parking_dock_translation_yaw_gate": rospy.get_param(
                        "~parking_dock_translation_yaw_gate", 0.22),
                    "parking_dock_forward_yaw_gate": rospy.get_param(
                        "~parking_dock_forward_yaw_gate", 0.14),
                    "parking_dock_forward_tangent_gate": rospy.get_param(
                        "~parking_dock_forward_tangent_gate", 0.12),
                    "parking_min_wall_distance": rospy.get_param(
                        "~parking_min_wall_distance", 0.19),
                    "parking_lidar_stop_distance": rospy.get_param(
                        "~parking_lidar_stop_distance", 0.15),
                    "parking_recenter_tolerance": rospy.get_param(
                        "~parking_recenter_tolerance", 0.04),
                    "parking_recenter_timeout_sec": rospy.get_param(
                        "~parking_recenter_timeout_sec", 4.0),
                    "parking_recenter_initial_wait_sec": rospy.get_param(
                        "~parking_recenter_initial_wait_sec", 1.0),
                    "parking_recenter_lateral_kp": rospy.get_param(
                        "~parking_recenter_lateral_kp", 0.16),
                    "parking_recenter_min_lateral": rospy.get_param(
                        "~parking_recenter_min_lateral", 0.015),
                    "parking_recenter_max_lateral": rospy.get_param(
                        "~parking_recenter_max_lateral", 0.065),
                    "parking_recenter_max_travel": rospy.get_param(
                        "~parking_recenter_max_travel", 0.30),
                    "parking_recenter_yaw_kp": rospy.get_param(
                        "~parking_recenter_yaw_kp", 1.0),
                    "parking_recenter_yaw_tolerance_deg": rospy.get_param(
                        "~parking_recenter_yaw_tolerance_deg", 2.0),
                    "parking_recenter_max_yaw": rospy.get_param(
                        "~parking_recenter_max_yaw", 0.18),
                    "parking_recenter_stable_sec": rospy.get_param(
                        "~parking_recenter_stable_sec", 0.25),
                    "parking_recenter_required_hits": rospy.get_param(
                        "~parking_recenter_required_hits", 3),
                    "parking_recenter_side_half_angle_deg": rospy.get_param(
                        "~parking_recenter_side_half_angle_deg", 25.0),
                    "parking_recenter_side_stop_m": rospy.get_param(
                        "~parking_recenter_side_stop_m", 0.18),
                    "parking_recenter_side_slow_m": rospy.get_param(
                        "~parking_recenter_side_slow_m", 0.28),
                    "parking_wall_fit_half_angle_deg": rospy.get_param(
                        "~parking_wall_fit_half_angle_deg", 35.0),
                    "parking_wall_fit_min_points": rospy.get_param(
                        "~parking_wall_fit_min_points", 12),
                    "parking_wall_fit_min_span": rospy.get_param(
                        "~parking_wall_fit_min_span", 0.25),
                    "parking_wall_fit_near_min_span": rospy.get_param(
                        "~parking_wall_fit_near_min_span", 0.18),
                    "parking_wall_fit_max_distance_jump": rospy.get_param(
                        "~parking_wall_fit_max_distance_jump", 0.05),
                    "parking_wall_fit_max_normal_jump_deg": rospy.get_param(
                        "~parking_wall_fit_max_normal_jump_deg", 8.0),
                    "parking_wall_fit_max_residual": rospy.get_param(
                        "~parking_wall_fit_max_residual", 0.015),
                    "parking_wall_fit_max_normal_error_deg": rospy.get_param(
                        "~parking_wall_fit_max_normal_error_deg", 20.0),
                    "parking_wall_fit_grace_sec": rospy.get_param(
                        "~parking_wall_fit_grace_sec", 1.5),
                    "parking_wall_fit_filter_alpha": rospy.get_param(
                        "~parking_wall_fit_filter_alpha", 0.45),
                    "parking_normal_offset": rospy.get_param(
                        "~parking_normal_offset", 0.0),
                    "parking_tangent_offset": rospy.get_param(
                        "~parking_tangent_offset", 0.0),
                    "parking_box_width": rospy.get_param("~parking_box_width", 0.50),
                    "parking_box_depth": rospy.get_param("~parking_box_depth", 0.50),
                    "parking_xy_tolerance": rospy.get_param(
                        "~parking_xy_tolerance", 0.04),
                    "parking_yaw_tolerance": rospy.get_param(
                        "~parking_yaw_tolerance", 0.06),
                    "target_center_coarse_step_deg": rospy.get_param(
                        "~target_center_coarse_step_deg", 4.0),
                    "target_center_fine_step_deg": rospy.get_param(
                        "~target_center_fine_step_deg", 2.0),
                    "target_center_start_speed": rospy.get_param(
                        "~target_center_start_speed", 0.20),
                    "target_center_step_max_speed": rospy.get_param(
                        "~target_center_max_speed", 0.35),
                    "target_center_timeout_sec": rospy.get_param(
                        "~target_center_timeout_sec", 12.0),
                    "coverage_scan_step_deg": rospy.get_param(
                        "~coverage_scan_step_deg", 20.0),
                    "coverage_scan_angular_speed": rospy.get_param(
                        "~coverage_scan_angular_speed", 0.35),
                    "coverage_scan_dwell_sec": rospy.get_param(
                        "~coverage_scan_dwell_sec", 0.65),
                    "coverage_candidate_hold_sec": rospy.get_param(
                        "~coverage_candidate_hold_sec", 1.2),
                    "coverage_scan_max_dwell_sec": rospy.get_param(
                        "~coverage_scan_max_dwell_sec", 2.0),
                    "coverage_scan_pose_timeout_sec": rospy.get_param(
                        "~coverage_scan_pose_timeout_sec", 0.5),
                    "coverage_goal_soft_timeout_sec": rospy.get_param(
                        "~coverage_goal_soft_timeout_sec", 25.0),
                    "coverage_goal_hard_timeout_sec": rospy.get_param(
                        "~coverage_goal_hard_timeout_sec", 40.0),
                    "coverage_goal_progress_window_sec": rospy.get_param(
                        "~coverage_goal_progress_window_sec", 5.0),
                    "coverage_goal_min_progress": rospy.get_param(
                        "~coverage_goal_min_progress", 0.03),
                    "coverage_anchor_position_tolerance": rospy.get_param(
                        "~coverage_anchor_position_tolerance", 0.28),
                    "coverage_anchor_yaw_tolerance_deg": rospy.get_param(
                        "~coverage_anchor_yaw_tolerance_deg", 5.0),
                    "coverage_anchor_yaw_hold_sec": rospy.get_param(
                        "~coverage_anchor_yaw_hold_sec", 0.20),
                    "coverage_anchor_yaw_timeout_sec": rospy.get_param(
                        "~coverage_anchor_yaw_timeout_sec", 12.0),
                    "coverage_no_progress_timeout_sec": rospy.get_param(
                        "~coverage_no_progress_timeout_sec", 5.5),
                    "coverage_fallback_enabled": bool_param(
                        "~coverage_fallback_enabled", True),
                    "coverage_fallback_make_plan_tolerance_m": rospy.get_param(
                        "~coverage_fallback_make_plan_tolerance_m", 0.12),
                },
                reuse_running=True,
            )
            start_service = "/vision_triggered_navigator/start_navigation"
            try:
                rospy.wait_for_service(
                    start_service,
                    timeout=float(rospy.get_param(
                        "~navigator_ready_timeout_sec", 12.0)),
                )
                start_response = rospy.ServiceProxy(start_service, Trigger)()
            except (rospy.ROSException, rospy.ServiceException) as exc:
                raise StageError(
                    "prewarmed navigator start service failed: {}".format(exc)
                )
            if not start_response.success:
                raise StageError(
                    "prewarmed navigator refused start: {}".format(
                        start_response.message
                    )
                )
            self.publish_status(
                "task2",
                "navigation_released",
                "announcement completed; prewarmed navigation released",
            )
            timeout = float(rospy.get_param("~factory_navigation_timeout_sec", 420.0))
            deadline = time.time() + timeout
            while time.time() < deadline:
                self.check_abort()
                self._deliver_target_trigger()
                if self.navigator_status == "arrived":
                    break
                if center_only and self.navigator_status == "centered":
                    break
                if self.navigator_status == "failed":
                    raise StageError("factory navigation failed")
                if self.navigator_status in (
                        "centering_failed", "parking_staging_failed",
                        "parking_recenter_failed", "parking_wall_fit_failed",
                        "parking_docking_failed", "parking_validation_failed",
                        "coverage_recovery_disable_failed"):
                    raise StageError("factory navigation {}".format(
                        self.navigator_status))
                for key in ("factory_navigator", "factory_ocr"):
                    proc = self.children.get(key)
                    if proc and proc.poll() is not None:
                        raise StageError("{} exited unexpectedly with code {}".format(key, proc.returncode))
                rospy.sleep(0.1)
            else:
                raise StageError("factory navigation timed out after {:.1f}s".format(timeout))
            parking_succeeded = (
                self.navigator_status == "arrived" or
                (center_only and self.navigator_status == "centered"))
        finally:
            self.ocr_target = None
            self.factory_parking_index = 0
            self.vision_trigger_latched = False
            self.trigger_request_pending = False
            self.trigger_service_accepted = False
            self.trigger_acknowledged = False
            self.safe_stop(cancel_navigation=True)
            if not parking_succeeded:
                self.stop_child("factory_ocr")
                self.stop_child("factory_navigator")
        if center_only:
            self.stop_child("factory_ocr")
            self.stop_child("factory_navigator")
            self.publish_status("task2", "center_test_completed", "target centering test completed")
            return
        self.safe_stop(cancel_navigation=True)
        if self.navigator_status != "arrived":
            raise StageError(
                "refusing parking completion before confirmed arrived state")
        if parking_index == 1 and not center_only:
            try:
                # The completed navigator normally exits on its own.  Retiring
                # only this light child keeps the expensive RKNN OCR model alive.
                self.stop_child("factory_navigator")
                self._prewarm_second_parking_stack()
            except Exception as exc:
                self.publish_status(
                    "task2", "second_parking_prewarm_deferred",
                    "second stack will start after announcement: {}".format(exc))
                self.stop_child("factory_ocr")
                self.stop_child("factory_navigator")
        if announcement_text:
            self.publish_status(
                "task2", "announcing",
                "parking {} confirmed; announcing result".format(parking_index))
            self.announce("custom", text=announcement_text)
            self.publish_status(
                "task2", "announcement_completed",
                "parking {} announcement completed".format(parking_index))
        if parking_index != 1:
            # Cleanup happens after the spoken result, so process shutdown can
            # never create a silent multi-second gap after the robot parks.
            self.stop_child("factory_ocr")
            self.stop_child("factory_navigator")
        self.publish_status(
            "task2", "parking_completed",
            "parking {} completed at {}".format(parking_index, workshop))

    def task2(self):
        if not self.category:
            raise StageError("task2 target_category is missing")
        physical_category = self.category
        physical_item = self.task1_result.get("pickup_item")
        physical_workshop = (
            self.task1_result.get("pickup_workshop")
            or CATEGORY_LABELS[physical_category][1]
        )
        physical_major = (
            self.task1_result.get("pickup_major")
            or CATEGORY_LABELS[physical_category][0]
        )
        physical_announcement = "已将{}放入{}。".format(
            physical_item, physical_workshop)
        self._run_factory_parking(
            physical_item, physical_workshop, physical_announcement, 1)

        if bool_param("~task2_center_only", False):
            return

        simulation_category = normalize_category(self.sim_category)
        simulation_item = self.task1_result.get("sim_item")
        simulation_workshop = self.task1_result.get("sim_workshop")
        simulation_major = self.task1_result.get("sim_major")
        if not simulation_category or simulation_category not in CATEGORY_LABELS:
            raise StageError("second parking simulation category is missing")
        if simulation_category == physical_category:
            raise StageError("second parking category duplicates physical category")
        if not simulation_item:
            raise StageError("second parking simulation item is missing")
        if not simulation_workshop:
            simulation_workshop = CATEGORY_LABELS[simulation_category][1]
        if not simulation_major:
            simulation_major = CATEGORY_LABELS[simulation_category][0]
        # 第二次停车不立即播报；等 task3 仿真真正完成后再播报。
        simulation_announcement = ""

        self.publish_status(
            "task2", "second_parking_start",
            "starting physical parking for simulation target {}".format(
                simulation_workshop))
        try:
            self.category = simulation_category
            self._run_factory_parking(
                simulation_item, simulation_workshop,
                simulation_announcement, 2)
        finally:
            self.category = physical_category

        # 第二次停车完成后立即启动仿真（后台线程），与 task3 等待并行。
        if self.enable_simulation:
            self._start_task3_async()

        if not self.enable_simulation:
            hold_sec = max(0.0, float(rospy.get_param(
                "~simulation_placeholder_hold_sec", 2.0)))
            self.publish_status(
                "task2", "simulation_placeholder",
                "simulation postponed; holding stop for {:.1f}s before next stage".format(
                    hold_sec))
            deadline = time.monotonic() + hold_sec
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                self.check_abort()
                self.safe_stop(cancel_navigation=True)
                rospy.sleep(0.05)
        self.publish_status(
            "task2", "completed",
            "both parking operations completed{}".format(
                "; simulation placeholder completed"
                if not self.enable_simulation else ""))

    def _start_task3_async(self):
        if "task3" not in stage_sequence(self.mode, self.enable_simulation):
            return False
        if not self.sim_category:
            return False
        with self.lock:
            if self.task3_thread is not None:
                return True
            worker = threading.Thread(
                target=self._task3_worker,
                name="task3-simulation",
                daemon=True,
            )
            self.task3_thread = worker
        self.publish_status(
            "task3",
            "starting",
            "starting simulation task in parallel",
        )
        worker.start()
        return True

    def _task3_worker(self):
        """UDP-based simulation bridge (xunfei2026_virtual_collaboration_v1).

        Sends a start_simulation datagram to the Gazebo side receiver and waits
        for a matching simulation_complete callback.  The Gazebo workspace ships
        with virtual_collaboration_receiver_v1.py (UDP 39026) and
        simulation_completion_bridge_v1.py (callback UDP 39027), so no changes
        are needed on the simulation side.
        """
        result_text = ""
        error = ""
        send_sock = None
        recv_sock = None
        host = rospy.get_param("~sim_bridge_host", "").strip()
        send_port = int(rospy.get_param("~sim_udp_send_port", 39026))
        recv_port = int(rospy.get_param("~sim_udp_recv_port", 39027))
        timeout = float(rospy.get_param("~sim_timeout_sec", 900.0))
        try:
            if not host:
                raise StageError("SIM_BRIDGE_HOST / sim_bridge_host is missing")

            sim_category = normalize_category(self.sim_category)
            sim_item = self.task1_result.get("sim_item", "")
            sim_major = (
                self.task1_result.get("sim_major")
                or CATEGORY_LABELS.get(sim_category, ("", ""))[0]
            )
            sim_workshop = (
                self.task1_result.get("sim_workshop")
                or CATEGORY_LABELS.get(sim_category, ("", ""))[1]
            )
            mission_id = str(uuid.uuid4())

            if sim_category not in CATEGORY_LABELS:
                raise StageError("invalid simulation category: {}".format(sim_category))
            if not sim_item:
                raise StageError("task3 sim_item is missing")

            # Bind local UDP port to receive the simulation completion callback.
            recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            recv_sock.bind(("0.0.0.0", recv_port))
            recv_sock.settimeout(1.0)

            # Send the start command (repeated for UDP reliability).
            send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            start_payload = {
                "protocol": "xunfei2026_virtual_collaboration_v1",
                "type": "start_simulation",
                "mission_id": mission_id,
                "sim_target_key": sim_category,
                "sim_selected_item": sim_item,
                "sim_target_category": sim_major,
                "sim_target_warehouse": sim_workshop,
            }
            start_bytes = json.dumps(start_payload, ensure_ascii=False).encode("utf-8")
            for _ in range(3):
                send_sock.sendto(start_bytes, (host, send_port))
                time.sleep(0.1)

            self.publish_status(
                "task3", "running_parallel",
                "simulation started via UDP: target={} item={} mission={}".format(
                    sim_category, sim_item, mission_id)
            )

            # Wait for the matching simulation_complete callback.
            deadline = time.time() + timeout
            completed = False
            while time.time() < deadline:
                if self.aborted.is_set() or rospy.is_shutdown():
                    raise Aborted("competition aborted during simulation")
                try:
                    data, _addr = recv_sock.recvfrom(65535)
                except socket.timeout:
                    continue
                try:
                    payload = json.loads(data.decode("utf-8"))
                except Exception:
                    continue
                if (payload.get("protocol") != "xunfei2026_virtual_collaboration_v1"
                        or payload.get("type") != "simulation_complete"):
                    continue
                if str(payload.get("mission_id", "")) != mission_id:
                    continue
                completed = True
                result_text = "SUCCESS: simulation completed item={} warehouse={}".format(
                    payload.get("sim_selected_item", ""),
                    payload.get("sim_target_warehouse", ""))
                self.publish_status("task3", "completed", result_text)
                break

            if not completed:
                raise StageError("simulation task timed out")
        except Exception as exc:
            error = str(exc)
        finally:
            if send_sock is not None:
                try:
                    send_sock.close()
                except Exception:
                    pass
            if recv_sock is not None:
                try:
                    recv_sock.close()
                except Exception:
                    pass
            with self.lock:
                self.task3_result_text = result_text
                self.task3_error = error
            self.task3_done.set()

    def task3(self):
        if not self.sim_category:
            raise StageError("task3 sim_category is missing")
        self._start_task3_async()
        if self.task3_thread is None:
            raise StageError("task3 simulation worker did not start")
        if not self.task3_done.is_set():
            self.publish_status(
                "task3",
                "waiting_parallel_result",
                "simulation running in parallel; waiting for completion",
            )
        while not self.task3_done.wait(0.1):
            self.check_abort()
        with self.lock:
            result_text = self.task3_result_text
            error = self.task3_error
        if error:
            raise StageError(error)
        if not result_text.startswith("SUCCESS:"):
            raise StageError("simulation completed without a success result")
        item = self.task1_result.get("sim_item") or self.task1_result.get("pickup_item")
        workshop = (
            self.task1_result.get("sim_workshop")
            or CATEGORY_LABELS[self.sim_category][1]
        )
        if not item:
            raise StageError("task3 sim_item is missing")
        self.announce("task3", item=item, workshop=workshop)
        self.publish_status("task3", "completed", result_text)

    def approach_task4_stop_line(self):
        self.strict_mission_status = {}
        self.publish_status(
            "task4", "approaching_stop_line",
            "navigating to staging pose, then approaching the stop line visually")
        self.start_child(
            "strict_line",
            "ucar_2026_strict_mission",
            "strict_mission.launch",
            {
                "start_traffic_detector": False,
                "start_viewer": self.debug,
                "traffic_pose_configured": True,
                "traffic_staging_x": float(rospy.get_param("~traffic_x")),
                "traffic_staging_y": float(rospy.get_param("~traffic_y")),
                "traffic_staging_yaw": float(rospy.get_param("~traffic_yaw")),
            },
        )
        try:
            rospy.wait_for_service("/strict_mission/start", timeout=10.0)
            response = rospy.ServiceProxy("/strict_mission/start", Trigger)()
            if not response.success:
                raise StageError(
                    "strict stop-line approach refused start: {}".format(response.message))

            timeout = (
                float(rospy.get_param("~move_base_timeout_sec", 90.0))
                + float(rospy.get_param("~line_approach_timeout_sec", 45.0))
                + 15.0
            )
            deadline = time.time() + timeout
            while time.time() < deadline:
                self.check_abort()
                status = self.strict_mission_status
                state = str(status.get("state", ""))
                if state == "WAIT_TRAFFIC":
                    distance = status.get("distance_m")
                    self.publish_status(
                        "task4", "stop_line_reached",
                        "vehicle held before stop line; distance_m={}".format(distance))
                    return
                if state == "FAULT":
                    raise StageError(
                        "strict stop-line approach failed: {}".format(
                            status.get("error") or status.get("detail") or "unknown fault"))
                proc = self.children.get("strict_line")
                if proc and proc.poll() is not None:
                    raise StageError("strict stop-line approach exited unexpectedly")
                rospy.sleep(0.1)
            raise StageError(
                "strict stop-line approach timed out after {:.1f}s".format(timeout))
        finally:
            self.stop_child("strict_line")
            self.safe_stop(cancel_navigation=True)

    def task4(self):
        skip_approach = bool_param("~skip_task4_stop_line_approach", False)
        configured = bool_param("~traffic_pose_configured", False)
        try:
            start_action = task4_start_action(skip_approach, configured)
        except ValueError as exc:
            raise StageError(str(exc))
        if start_action == "approach":
            self.approach_task4_stop_line()
        else:
            self.safe_stop(cancel_navigation=True)
            self.publish_status(
                "task4", "stop_line_ready",
                "using manually positioned stop-line start; vehicle held stopped")
        self.traffic_decision = ""
        self.red_announced = False
        self.publish_status("task4", "detecting", "waiting for traffic-light consensus")
        try:
            self.start_child(
                "traffic_light",
                "ucar_2026_traffic_light_rknn_test",
                "traffic_light_rknn_x11_speak_test.launch",
                {
                    "start_camera": False,
                    "start_tts": False,
                    "start_competition_speech": False,
                    "start_viewer": self.debug,
                    "enable_speech": False,
                    "required": True,
                },
            )
            deadline = time.time() + float(rospy.get_param("~traffic_timeout_sec", 180.0))
            while time.time() < deadline:
                self.check_abort()
                if self.traffic_decision == "stop":
                    self.safe_stop()
                    if not self.red_announced:
                        self.announce("task4", decision="stop")
                        self.red_announced = True
                        self.publish_status("task4", "red_wait", "red light: holding stop")
                    self.traffic_decision = ""
                elif self.traffic_decision in ("left", "right", "straight"):
                    decision = self.traffic_decision
                    self.announce("task4", decision=decision)
                    self.traffic_pub.publish(String(data=decision))
                    self.publish_status("task4", "completed", "decision={}".format(decision))
                    self.traffic_decision = decision
                    return
                proc = self.children.get("traffic_light")
                if proc and proc.poll() is not None:
                    raise StageError("traffic-light detector exited unexpectedly")
                rospy.sleep(0.1)
            raise StageError("traffic-light recognition timed out")
        finally:
            self.stop_child("traffic_light")
            self.safe_stop(cancel_navigation=True)

    def task5(self):
        decision = self.traffic_decision or rospy.get_param("~traffic_decision", "").strip().lower()
        if decision not in TRACK_CONFIG:
            raise StageError("task5 traffic_decision must be left/right/straight")
        launch_file, status_topic, finish_value = TRACK_CONFIG[decision]
        self.safe_stop(cancel_navigation=True)
        self.track_status[status_topic] = ""
        self.publish_status("task5", "line_following", "launching {}".format(launch_file))
        try:
            self.start_child(
                "line_follow",
                "ucar_2026_track_end_stop",
                launch_file,
                {"start_driver": False, "start_camera": False, "start_viewer": self.debug},
            )
            timeout = float(rospy.get_param("~track_timeout_sec", 420.0))
            self.wait_loop(
                timeout,
                lambda: self.track_status.get(status_topic) == finish_value,
                child_key="line_follow",
            )
        finally:
            self.stop_child("line_follow")
            self.safe_stop(cancel_navigation=True)
        self.announce("task5")
        self.publish_status("task5", "completed", "competition completed")

    def run(self):
        try:
            handlers = {
                "task1": self.task1,
                "task2": self.task2,
                "task3": self.task3,
                "task4": self.task4,
                "task5": self.task5,
            }
            previous_stage = None
            for stage in stage_sequence(self.mode, self.enable_simulation):
                if previous_stage == "task1" and stage == "task2":
                    if not self.task1_task2_handoff_prepared:
                        self.run_stage("task1", self.task1_task2_handoff)
                if task4_handoff_required(previous_stage, stage):
                    source_stage = previous_stage
                    self.run_stage(
                        source_stage,
                        lambda: self.production_task4_handoff(source_stage),
                    )
                self.run_stage(stage, handlers[stage])
                previous_stage = stage
            self.publish_status("competition", "completed", "requested flow completed")
        except Aborted as exc:
            self.publish_status("competition", "aborted", error=str(exc))
        except rospy.ROSInterruptException:
            rospy.loginfo("competition controller interrupted during ROS shutdown")
        except Exception as exc:
            if rospy.is_shutdown():
                rospy.loginfo("competition controller stopped during ROS shutdown")
            else:
                rospy.logerr("unhandled competition error: %s", exc)
                self.safe_stop(cancel_navigation=True)
                self.publish_status("competition", "failed", error=str(exc))
        finally:
            self.shutdown()

    def shutdown(self):
        self.stop_all_children()
        try:
            self.safe_stop(cancel_navigation=True)
        except Exception:
            pass


def main():
    rospy.init_node("competition_flow")
    CompetitionFlow().run()


if __name__ == "__main__":
    main()
