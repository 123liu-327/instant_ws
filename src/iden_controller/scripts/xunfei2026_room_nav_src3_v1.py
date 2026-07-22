#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Room-delivery navigation with src3-inspired stop-and-look scanning.

Sits between the voice/QR/LLM fusion node and the simulation handoff.
Replaces the entire anchor-coverage class hierarchy with a single focused
manager that borrows clean patterns from src3 without the legacy layering.

Key ideas borrowed from src3 (not copied):
- DirectedYawAccumulator  – odometry-closed yaw tracking with wrap handling
- TemporalTargetFilter   – time-window evidence with blank-preserve semantics
- VoteWindow             – sliding-window majority voting before target lock
- stop-then-look         – dwell → rotate step → dwell → …  (not continuous)
- top-N candidate ranking by bbox centre proximity (Euclidean, not horizontal)

Architecture
------------
  voice/QR/LLM fusion  ──→  dual-order result  ──→  this node
                                                            │
                ┌───────────────────────────────────────────┤
                │  room entry (reuse first-stage AMCL/map)  │
                │  for each anchor:                         │
                │    move_base goal → arrive → scan         │
                │    OCR confirmed? → lock → centre → park  │
                │  real delivery → TTS → simulation target  │
                │  publish parking success → handoff node   │
                └───────────────────────────────────────────┘
"""

import json
import math
import os
import signal
import subprocess
import threading
import time

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


# ---------------------------------------------------------------------------
# src3-inspired helpers (not copied – rewritten for this codebase)
# ---------------------------------------------------------------------------

def _norm_angle(a):
    """Normalise to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_quat(q):
    """Extract yaw from a geometry_msgs/Quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _split_rotation_steps(total, step):
    """Split a positive angle into exact bounded steps (src3 pattern)."""
    remaining = max(0.0, float(total))
    step = float(step)
    steps = []
    while remaining > 1e-9:
        s = min(step, remaining)
        steps.append(s)
        remaining -= s
    return steps


class _YawTracker:
    """src3 DirectedYawAccumulator – odometry-closed yaw progress."""

    def __init__(self, direction):
        self.direction = 1.0 if direction >= 0.0 else -1.0
        self.start = None
        self.progress = 0.0

    def reset(self, yaw):
        self.start = float(yaw)
        self.progress = 0.0

    def update(self, yaw):
        net = _norm_angle(float(yaw) - self.start) * self.direction
        if net > self.progress:
            self.progress = net
        return self.progress


class _TemporalFilter:
    """src3 TemporalTargetFilter – time-window, blanks preserve, rivals reset."""

    def __init__(self, required, window_s):
        self.required = max(1, int(required))
        self.window_s = max(0.05, float(window_s))
        self._stamps = []

    @property
    def hits(self):
        return len(self._stamps)

    def reset(self):
        self._stamps = []

    def push(self, observed, now=None):
        now = time.monotonic() if now is None else float(now)
        self._stamps = [s for s in self._stamps if now - s <= self.window_s]
        if not observed:
            return self.hits >= self.required
        self._stamps.append(now)
        return self.hits >= self.required


class _VoteWindow:
    """src3 VoteWindow – sliding majority consensus."""

    def __init__(self, size, min_count):
        self.size = max(1, int(size))
        self.min_count = max(1, int(min_count))
        self._items = []

    def push(self, category):
        self._items.append(category)
        if len(self._items) > self.size:
            self._items.pop(0)
        counts = {}
        for c in self._items:
            if c:
                counts[c] = counts.get(c, 0) + 1
        if not counts:
            return None
        top = max(counts.values())
        if top < self.min_count:
            return None
        winners = {c for c, n in counts.items() if n == top}
        for c in reversed(self._items):
            if c in winners:
                return c
        return None

    def reset(self):
        self._items = []


# ---------------------------------------------------------------------------
# Workshop / category helpers
# ---------------------------------------------------------------------------

CATEGORY_WORKSHOP = {
    "food":       "食品加工车间",
    "daily":      "日用品加工车间",
    "electronic": "电子产品生产车间",
}
WORKSHOP_CATEGORY = {v: k for k, v in CATEGORY_WORKSHOP.items()}
WORKSHOP_CATEGORY["电子产品加工车间"] = "electronic"


def _payload_category(payload):
    """Extract normalised category from an OCR adapter frame."""
    src2 = str(payload.get("src2_category", "") or "").strip().lower()
    if src2 in CATEGORY_WORKSHOP:
        return src2
    label = str(payload.get("label", "") or "").strip()
    if label == "电子产品加工车间":
        label = "电子产品生产车间"
    return WORKSHOP_CATEGORY.get(label, "")


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class Src3RoomNav:
    def __init__(self):
        rospy.init_node("xunfei2026_room_nav_src3")

        # ---- topics ----
        self._tts_pub     = rospy.Publisher(
            rospy.get_param("~tts_topic", "/factory/tts_text"), String, queue_size=5)
        self._status_pub  = rospy.Publisher(
            rospy.get_param("~status_topic", "/factory_room/xunfei2026_delivery_status"),
            String, queue_size=10, latch=True)
        self._cone_pub    = rospy.Publisher(
            rospy.get_param("~cone_control_topic", "/factory_room/navigation_active"),
            Bool, queue_size=2, latch=True)
        self._ocr_ctl_pub = rospy.Publisher(
            rospy.get_param("~ocr_control_topic", "/factory_room/ocr_control"),
            String, queue_size=3, latch=True)
        self._cmd_pub     = rospy.Publisher("/cmd_vel", Twist, queue_size=2)

        rospy.Subscriber(rospy.get_param("~ocr_topic", "/factory_room/ocr_result"),
                         String, self._ocr_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/odom"),
                         Odometry, self._odom_cb, queue_size=10)
        rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"),
                         LaserScan, self._scan_cb, queue_size=5)

        # ---- params ----
        self._scan_step   = math.radians(abs(rospy.get_param("~scan_step_deg", 20.0)))
        self._scan_speed  = abs(rospy.get_param("~scan_speed_rps", 0.35))
        self._scan_dwell  = max(0.0, rospy.get_param("~scan_dwell_s", 0.65))
        self._scan_max    = max(self._scan_dwell, rospy.get_param("~scan_max_dwell_s", 2.0))
        self._ocr_hits    = max(1, int(rospy.get_param("~ocr_required_hits", 2)))
        self._ocr_window  = max(0.1, rospy.get_param("~ocr_window_s", 1.5))
        self._vote_size   = max(3, int(rospy.get_param("~vote_window_size", 7)))
        self._vote_min    = max(2, int(rospy.get_param("~vote_min_count", 3)))
        self._nav_timeout = max(10.0, rospy.get_param("~nav_goal_timeout_s", 40.0))
        self._nav_tol_xy  = abs(rospy.get_param("~nav_xy_tolerance_m", 0.12))
        self._nav_tol_yaw = math.radians(abs(rospy.get_param("~nav_yaw_tolerance_deg", 10.0)))
        self._mission_to  = max(30.0, rospy.get_param("~mission_timeout_s", 420.0))
        self._wait_tts    = max(0.0, rospy.get_param("~wait_after_tts_s", 0.5))
        self._same_ws_s   = max(0.3, rospy.get_param("~same_workshop_confirm_s", 0.5))

        # LiDAR offset – must match ydlidar.launch static transform
        self._laser_x  = rospy.get_param("~laser_offset_x_m", 0.11)
        self._laser_y  = rospy.get_param("~laser_offset_y_m", 0.0)
        self._laser_yaw = rospy.get_param("~laser_yaw_rad", -0.07)

        # ---- anchors (same 8-anchor route as v1/v6) ----
        self._anchors = self._default_anchors()
        rospy.loginfo("SRC3_NAV ready  anchors=%d  step=%.0fdeg  dwell=%.2fs",
                      len(self._anchors), math.degrees(self._scan_step), self._scan_dwell)

        # ---- state ----
        self._lock = threading.Lock()
        self._odom_yaw   = None
        self._odom_stamp = 0.0
        self._front_m    = float("inf")
        self._front_stamp = 0.0
        self._target_warehouse = ""
        self._search_active = False

        # OCR filtering
        self._temporal = _TemporalFilter(self._ocr_hits, self._ocr_window)
        self._vote     = _VoteWindow(self._vote_size, self._vote_min)
        self._target_locked = threading.Event()
        self._ocr_latest    = None
        self._ocr_bbox      = None
        self._ocr_img_w     = 0
        self._ocr_img_h     = 0

        # delivery state
        self._real_wh = ""
        self._sim_wh  = ""
        self._stage   = "real"
        self._finished = threading.Event()

        # move_base
        self._mb_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        # tf listener (persistent – creating one per call misses the cache)
        import tf
        self._tf_listener = tf.TransformListener()

        # OCR subprocess
        self._ocr_process = None
        self._ocr_restarts = 0
        self._ocr_max_restarts = max(0, int(rospy.get_param("~ocr_max_restarts", 2)))
        self._ocr_ready_timeout = rospy.get_param("~ocr_ready_timeout_s", 12.0)

        rospy.loginfo("SRC3_NAV ready  anchors=%d  step=%.0fdeg  dwell=%.2fs",
                      len(self._anchors), math.degrees(self._scan_step), self._scan_dwell)

    # ------------------------------------------------------------------
    # anchors (hardcoded — same 8 points as v1/v6)
    # ------------------------------------------------------------------
    @staticmethod
    def _default_anchors():
        def _a(name, x, y, yaw, sweeps):
            return {"name": name, "x": x, "y": y, "yaw": math.radians(yaw), "sweeps": sweeps}
        return [
            _a("d1", -0.67, -2.55, -50,  [(-math.radians(100), "cw")]),
            _a("d2", -1.55, -2.55, -90,  [(-math.radians(120), "cw")]),
            _a("d3", -1.53, -2.16, 180,  [(-math.radians(90),  "cw")]),
            _a("d5",  0.33, -1.54,  51,  [( math.radians(100), "ccw")]),
            _a("d6",  1.29, -1.54,  49,  [( math.radians(100), "ccw")]),
            _a("d7",  2.34, -1.24,   0,  [( math.radians(90),  "ccw")]),
            _a("d8",  2.34, -2.34,-125,  [( 2.25,             "ccw")]),
            _a("d9",  1.30, -2.05, -45,  [(-math.radians(90),  "cw")]),
        ]

    # ------------------------------------------------------------------
    # OCR subprocess management (same pattern as original v3)
    # ------------------------------------------------------------------
    def _start_ocr(self):
        pkg  = rospy.get_param("~room_launch_pkg", "iden_controller")
        launch_file = rospy.get_param("~ocr_launch_file",
                                       "xunfei2026_factory_ocr_src2_v1.launch")
        self._ocr_process = subprocess.Popen(
            ["roslaunch", pkg, launch_file],
            start_new_session=True)
        deadline = time.monotonic() + self._ocr_ready_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._ocr_process.poll() is not None:
                raise RuntimeError("OCR launch exited before ready (code {})".format(
                    self._ocr_process.returncode))
            rospy.sleep(0.1)
        self._ocr_ctl_pub.publish(String(data="reset"))
        self._ocr_ctl_pub.publish(String(data="enable"))
        rospy.loginfo("SRC3_NAV OCR ready")

    def _stop_ocr(self):
        proc = self._ocr_process
        self._ocr_process = None
        if proc is None or proc.poll() is not None:
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

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _odom_cb(self, msg):
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        with self._lock:
            self._odom_yaw = yaw
            self._odom_stamp = time.monotonic()

    def _scan_cb(self, msg):
        # nearest front distance for safety
        if msg.ranges:
            front = min(msg.ranges[:15] + msg.ranges[-15:])
        else:
            front = float("inf")
        with self._lock:
            self._front_m = front
            self._front_stamp = time.monotonic()

    def _ocr_cb(self, msg):
        if not self._search_active:
            return
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        now = time.monotonic()
        observed = _payload_category(payload)
        bbox = payload.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) < 4:
            observed = ""
        confirmed = self._temporal.push(observed, now)
        if observed:
            self._vote.push(observed)
        if not (observed and confirmed):
            return
        # cache best bbox view
        target = WORKSHOP_CATEGORY.get(self._target_warehouse, "")
        if observed != target:
            return
        # VoteWindow consensus gate
        if self._vote.push(observed) != observed:
            return
        with self._lock:
            self._ocr_latest = dict(payload)
            self._ocr_bbox = bbox
            self._ocr_img_w = int(payload.get("image_width", 0) or 0)
            self._ocr_img_h = int(payload.get("image_height", 0) or 0)
        self._target_locked.set()
        rospy.loginfo("SRC3_NAV TARGET_LOCKED %s hits=%d",
                      self._target_warehouse, self._temporal.hits)

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _stop(self, duration=0.05):
        for _ in range(max(1, int(duration / 0.025))):
            self._cmd_pub.publish(Twist())
            rospy.sleep(0.025)

    def _status(self, state, **kw):
        payload = {"state": state, "stamp": time.time()}
        payload.update(kw)
        self._status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        rospy.loginfo("SRC3_NAV %s %s", state, json.dumps(kw, ensure_ascii=False))

    def _publish_handoff(self, same_workshop=False):
        """Tell the simulation handoff node we're done.  Must match the
        format expected by xunfei2026_simulation_handoff_second_parking_v3:
        state=SIM_TARGET_PARKED + parking_success=True."""
        self._status("SIM_TARGET_PARKED",
                     parking_success=True,
                     same_workshop=bool(same_workshop),
                     simulation_trigger_authorized=True)

    def _current_pose(self):
        """Get AMCL-corrected pose via TF (map frame)."""
        try:
            self._tf_listener.waitForTransform(
                "map", "base_link", rospy.Time(0), rospy.Duration(0.5))
            (pos, quat) = self._tf_listener.lookupTransform(
                "map", "base_link", rospy.Time(0))
            yaw = _yaw_from_quat(Quaternion(*quat))
            return (pos[0], pos[1], yaw)
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "SRC3_NAV TF failed: %s", exc)
            return None

    def _navigate(self, x, y, yaw, timeout=None):
        """Send move_base goal and wait for result."""
        timeout = timeout or self._nav_timeout
        if not self._mb_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("SRC3_NAV move_base not available")
            return "FAILED"

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = _quat_from_yaw(yaw)

        self._mb_client.send_goal(goal)
        finished = self._mb_client.wait_for_result(rospy.Duration(timeout))
        if not finished:
            self._mb_client.cancel_goal()
            return "TIMEOUT"
        state = self._mb_client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return "SUCCEEDED"
        return "FAILED"

    # ------------------------------------------------------------------
    # scanning
    # ------------------------------------------------------------------
    def _rotate_step(self, direction, angle):
        """src3 _rotate_qr_step: odometry-closed angular step."""
        with self._lock:
            yaw = self._odom_yaw
            stamp = self._odom_stamp
        if yaw is None or time.monotonic() - stamp > 0.6:
            self._stop(0.3)
            return "NO_ODOM"

        tracker = _YawTracker(direction)
        tracker.reset(yaw)
        deadline = time.monotonic() + angle / max(self._scan_speed, 0.05) + 2.0

        twist = Twist()
        twist.angular.z = self._scan_speed * direction

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._target_locked.is_set():
                self._stop(0.3)
                return "TARGET"
            if time.monotonic() > deadline:
                self._stop(0.3)
                return "TIMEOUT"

            with self._lock:
                yaw = self._odom_yaw
                stamp = self._odom_stamp
                front = self._front_m
                front_s = self._front_stamp

            if yaw is None or time.monotonic() - stamp > 0.6:
                self._stop(0.3)
                return "NO_ODOM"

            # basic safety
            if time.monotonic() - front_s < 0.6 and front < 0.18:
                self._stop(0.3)
                return "BLOCKED"

            tracker.update(yaw)
            if tracker.progress >= angle:
                self._stop(0.3)
                return "SUCCEEDED"

            self._cmd_pub.publish(twist)
            rate.sleep()

        self._stop(0.3)
        return "TIMEOUT"

    def _dwell(self, dwell_s, max_s):
        """Hold zero velocity while OCR runs. Extend if within max."""
        started = time.monotonic()
        deadline = started + dwell_s
        maximum  = started + max_s
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._target_locked.is_set():
                return "TARGET"
            # extend dwell when OCR is consuming frames, up to max
            deadline = min(deadline + 0.02, maximum)
            self._cmd_pub.publish(Twist())
            rate.sleep()
        self._cmd_pub.publish(Twist())
        return "SUCCEEDED"

    def _scan_at_anchor(self, anchor):
        """src3-style stop-and-look: dwell → rotate → dwell → rotate → …"""
        self._temporal.reset()
        self._vote.reset()
        self._target_locked.clear()

        name = anchor.get("name", "?")
        sweeps = anchor.get("sweeps", [])

        self._status("SCAN_BEGIN", anchor=name, sweeps=len(sweeps))

        # initial dwell
        result = self._dwell(self._scan_dwell, self._scan_max)
        if result == "TARGET":
            return "TARGET"

        for si, (signed, _dname) in enumerate(sweeps, 1):
            direction = 1.0 if signed >= 0.0 else -1.0
            steps = _split_rotation_steps(abs(signed), self._scan_step)

            for step_i, angle in enumerate(steps, 1):
                result = self._rotate_step(direction, angle)
                if result == "TARGET":
                    return "TARGET"
                if result != "SUCCEEDED":
                    self._status("SCAN_STEP_FAILED", anchor=name,
                                 sweep=si, step=step_i, result=result)
                    continue

                result = self._dwell(self._scan_dwell, self._scan_max)
                if result == "TARGET":
                    return "TARGET"

            self._status("SWEEP_DONE", anchor=name, sweep=si)

        self._status("SCAN_COMPLETE", anchor=name)
        return "SUCCEEDED"

    # ------------------------------------------------------------------
    # parking (simple wall-relative approach)
    # ------------------------------------------------------------------
    def _park_at_target(self):
        """Simple parking: centre bbox → approach wall → stop."""
        self._status("PARK_BEGIN")

        # centre the sign in the camera view
        for attempt in range(3):
            with self._lock:
                bbox = self._ocr_bbox
                img_w = self._ocr_img_w
            if not bbox or img_w <= 1:
                break
            cx = 0.5 * (bbox[0] + bbox[2])
            error = (cx - 0.5 * img_w) / (0.5 * img_w)  # [-1, 1]
            if abs(error) < 0.08:
                break
            direction = -1.0 if error < 0 else 1.0
            self._rotate_step(direction, math.radians(abs(error) * 15.0))
            rospy.sleep(0.3)

        self._status("PARK_CENTRED")

        # drive forward until close to wall
        deadline = time.monotonic() + 12.0
        twist = Twist()
        twist.linear.x = 0.06
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                front = self._front_m
                front_s = self._front_stamp
            if time.monotonic() - front_s < 0.6 and front < 0.25:
                break
            # watch for OCR loss (sign too close)
            with self._lock:
                bbox_h = (self._ocr_bbox[3] - self._ocr_bbox[1] if self._ocr_bbox else 0)
            if bbox_h / max(self._ocr_img_h, 1) > 0.85:
                break
            self._cmd_pub.publish(twist)
            rate.sleep()

        self._stop(0.5)
        self._status("PARK_COMPLETE")
        return "SUCCEEDED"

    # ------------------------------------------------------------------
    # room coverage
    # ------------------------------------------------------------------
    def _cover_room(self):
        """Walk anchors, scan each, return on TARGET."""
        for idx, anchor in enumerate(self._anchors):
            if rospy.is_shutdown():
                return "SHUTDOWN"

            name = anchor.get("name", str(idx))
            x, y, yaw = anchor["x"], anchor["y"], anchor["yaw"]

            self._status("NAV_TO_ANCHOR", anchor=name, x=x, y=y)
            result = self._navigate(x, y, yaw)
            if result != "SUCCEEDED":
                self._status("ANCHOR_SKIPPED", anchor=name, reason=result)
                continue

            self._status("ANCHOR_ARRIVED", anchor=name)
            result = self._scan_at_anchor(anchor)
            if result == "TARGET":
                self._status("TARGET_FOUND", anchor=name)
                return "TARGET"

        return "COVERAGE_COMPLETE"

    # ------------------------------------------------------------------
    # mission
    # ------------------------------------------------------------------
    def _mission(self):
        """Dual-stage delivery."""
        deadline = time.monotonic() + self._mission_to

        # wait for fusion node to publish a COMPLETE order
        # The fusion publishes partial updates during voice/QR/LLM; we must
        # read until both real and sim workshops are populated.
        self._status("WAITING_ORDER")
        result_topic = rospy.get_param("~result_topic", "/factory/subtask1_result")
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                raw = rospy.wait_for_message(result_topic, String, timeout=2.0)
                if not raw or not raw.data.strip():
                    continue
                order = json.loads(raw.data)
                self._real_wh = str(order.get("pickup_workshop", "") or
                                    order.get("real_warehouse", ""))
                self._sim_wh  = str(order.get("sim_workshop", "") or
                                    order.get("sim_warehouse", ""))
                if self._real_wh and self._sim_wh:
                    break
                self._status("WAITING_ORDER_PARTIAL",
                             real=self._real_wh, sim=self._sim_wh)
            except (rospy.ROSException, json.JSONDecodeError):
                pass
            except Exception:
                rospy.sleep(0.5)
        else:
            raise RuntimeError("no complete order received within {:.0f}s".format(
                self._mission_to))

        self._status("ORDER_ACCEPTED", real=self._real_wh, sim=self._sim_wh)

        # Wait for the first-stage navigator to bring the robot into the room.
        self._status("WAITING_ROOM_ENTRY")
        first_stage_topic = rospy.get_param(
            "~first_stage_status_topic", "/xunfei2026_first_stage/status")
        entry_deadline = time.monotonic() + 180.0
        while not rospy.is_shutdown() and time.monotonic() < entry_deadline:
            try:
                raw = rospy.wait_for_message(first_stage_topic, String, timeout=2.0)
                if raw and "COMPLETE" in (raw.data or ""):
                    break
            except rospy.ROSException:
                pass
        else:
            raise RuntimeError("first-stage room entry did not complete")

        # Start OCR (runs as roslaunch subprocess in the background)
        self._status("STARTING_OCR")
        self._start_ocr()

        # Ensure move_base is alive before sending goals
        self._status("WAITING_MOVE_BASE")
        if not self._mb_client.wait_for_server(rospy.Duration(15.0)):
            raise RuntimeError("move_base not available after room entry")

        rospy.sleep(self._wait_tts)

        # ------ stage 1: real target ------
        self._stage = "real"
        self._search_active = True
        self._target_warehouse = self._real_wh
        self._ocr_ctl_pub.publish(String(data="enable"))

        self._cone_pub.publish(Bool(data=True))
        rospy.sleep(0.5)

        outcome = self._cover_room()
        if outcome == "TARGET":
            self._park_at_target()
            self._status("REAL_PARKED", workshop=self._real_wh)

            # TTS announcement
            item = str(order.get("pickup_item", ""))
            tts = u"已将{}放入{}".format(item, self._real_wh) if item else u"第一次配送完成"
            self._tts_pub.publish(String(data=tts))
            rospy.sleep(self._wait_tts + 1.0)
        else:
            raise RuntimeError("real target not found: {}".format(outcome))

        # ------ stage 2: simulation target ------
        if self._real_wh == self._sim_wh:
            self._status("SAME_WORKSHOP", workshop=self._real_wh)
            rospy.sleep(0.5)
            self._publish_handoff(same_workshop=True)
            return

        # drive away from first parking spot (back up a bit)
        twist = Twist()
        twist.linear.x = -0.08
        for _ in range(30):
            self._cmd_pub.publish(twist)
            rospy.sleep(0.05)
        self._stop(0.3)

        self._stage = "simulation_target"
        self._target_warehouse = self._sim_wh
        self._temporal.reset()
        self._vote.reset()
        self._target_locked.clear()

        outcome = self._cover_room()
        if outcome == "TARGET":
            self._park_at_target()
            self._status("SIM_PARKED", workshop=self._sim_wh)
        else:
            raise RuntimeError("sim target not found: {}".format(outcome))

        self._publish_handoff(same_workshop=False)

    def run(self):
        try:
            self._mission()
        except Exception as exc:
            rospy.logerr("SRC3_NAV FAILED: %s", exc)
            self._status("MISSION_FAILED", error=str(exc))
        finally:
            self._stop(0.5)
            self._cone_pub.publish(Bool(data=False))
            self._ocr_ctl_pub.publish(String(data="disable"))
            self._stop_ocr()
            self._status("SHUTDOWN")


if __name__ == "__main__":
    node = Src3RoomNav()
    worker = threading.Thread(target=node.run, name="mission", daemon=True)
    worker.start()
    rospy.spin()
    worker.join()
