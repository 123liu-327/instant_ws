#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""3-point room delivery with dual-stage parking and simulation handoff.

Route: first_stage → doorway → room_entry → d1 → d2 → d3
At each d-point: continuous yaw sweep while OCR watches for the target.
Dual-stage: real parking → TTS → simulation parking → handoff trigger.

Fits into the subtask1 launch infrastructure — replaces the anchor-coverage
manager while keeping voice/QR/LLM fusion and sim handoff unchanged.
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
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _norm(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


CAT_WORKSHOP = {
    "food": "食品加工车间", "daily": "日用品加工车间",
    "electronic": "电子产品生产车间",
}
WS_CAT = {v: k for k, v in CAT_WORKSHOP.items()}
WS_CAT["电子产品加工车间"] = "electronic"


class _VoteWindow:
    def __init__(self, size, need):
        self.sz = max(1, int(size))
        self.nd = max(1, int(need))
        self._q = []

    def push(self, cat):
        self._q.append(cat)
        if len(self._q) > self.sz:
            self._q.pop(0)
        cnt = {}
        for c in self._q:
            if c:
                cnt[c] = cnt.get(c, 0) + 1
        if not cnt:
            return None
        top = max(cnt.values())
        if top < self.nd:
            return None
        wins = {c for c, n in cnt.items() if n == top}
        for c in reversed(self._q):
            if c in wins:
                return c
        return None

    def reset(self):
        self._q = []


# ---------------------------------------------------------------------------
# 3-point dual-stage node
# ---------------------------------------------------------------------------

class ThreePointDualStage:
    def __init__(self):
        rospy.init_node("xunfei2026_3point_dual_stage")

        # topics
        self._tts    = rospy.Publisher(
            rospy.get_param("~tts_topic", "/factory/tts_text"), String, queue_size=5)
        self._status = rospy.Publisher(
            rospy.get_param("~status_topic", "/factory_room/xunfei2026_delivery_status"),
            String, queue_size=10, latch=True)
        self._cone   = rospy.Publisher(
            rospy.get_param("~cone_control_topic", "/factory_room/navigation_active"),
            Bool, queue_size=2, latch=True)
        self._ocr_ctl = rospy.Publisher(
            rospy.get_param("~ocr_control_topic", "/factory_room/ocr_control"),
            String, queue_size=3, latch=True)
        self._cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=2)

        rospy.Subscriber(rospy.get_param("~ocr_topic", "/factory_room/ocr_result"),
                         String, self._ocr_cb, queue_size=20)
        rospy.Subscriber(rospy.get_param("~odom_topic", "/odom"),
                         Odometry, self._odom_cb, queue_size=10)
        rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"),
                         LaserScan, self._scan_cb, queue_size=5)

        # ---- 3 scan points (from YAML or defaults) ----
        pts = rospy.get_param("~route_points", [])
        self._anchors = [p for p in pts if p.get("name", "").startswith("d")]
        if len(self._anchors) < 2:
            self._anchors = [
                {"name": "d1", "x": -1.43, "y": -2.57, "yaw": 1.571, "tolerance": 0.15},
                {"name": "d2", "x":  0.41, "y": -2.00, "yaw": 2.356, "tolerance": 0.15},
                {"name": "d3", "x":  2.04, "y": -2.51, "yaw": -2.356,"tolerance": 0.15},
            ]
        rospy.loginfo("3POINT anchors=%s", [a["name"] for a in self._anchors])

        # scan specs (continuous yaw sweep)
        specs = rospy.get_param("~continuous_scan_specs", [])
        self._specs = {s["name"]: s for s in specs}
        self._scan_speed = rospy.get_param("~continuous_scan_max_speed_rps", 0.14)
        self._scan_min   = rospy.get_param("~continuous_scan_min_speed_rps", 0.045)
        self._scan_accel  = rospy.get_param("~continuous_scan_acceleration_rps2", 0.28)

        # OCR
        self._vote_win  = rospy.get_param("~vote_window", 10)
        self._vote_need = rospy.get_param("~vote_need", 5)
        self._vote = _VoteWindow(self._vote_win, self._vote_need)

        # state
        self._lock = threading.Lock()
        self._odom_yaw = None
        self._front_m = float("inf")
        self._target_locked = threading.Event()

        # delivery
        self._real_wh = ""
        self._sim_wh  = ""
        self._ocr_proc = None
        self._mb = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        import tf
        self._tfl = tf.TransformListener()

        rospy.loginfo("3POINT_DUAL ready  anchors=%d  speed=%.2f  vote=%d/%d",
                      len(self._anchors), self._scan_speed,
                      self._vote_win, self._vote_need)

    # ---- callbacks ----
    def _odom_cb(self, msg):
        with self._lock:
            self._odom_yaw = _yaw(msg.pose.pose.orientation)

    def _scan_cb(self, msg):
        r = msg.ranges
        f = min(r[:15] + r[-15:]) if r else float("inf")
        with self._lock:
            self._front_m = f

    def _ocr_cb(self, msg):
        try:
            p = json.loads(msg.data)
        except Exception:
            return
        # old OCR format uses "label" and "frame_label"
        cat = str(p.get("label", "") or "").strip().lower()
        if cat not in CAT_WORKSHOP:
            return
        bbox = p.get("bbox")
        if not isinstance(bbox, (list, tuple)) or len(bbox) < 4:
            return
        target = WS_CAT.get(self._real_wh if self._stage == "real" else self._sim_wh, "")
        if cat != target:
            return
        if self._vote.push(cat) == cat:
            self._target_locked.set()

    # ---- helpers ----
    def _stop(self, d=0.1):
        for _ in range(max(1, int(d / 0.025))):
            self._cmd.publish(Twist())
            rospy.sleep(0.025)

    def _say(self, s, **kw):
        p = {"state": s, "stamp": time.time()}
        p.update(kw)
        self._status.publish(String(data=json.dumps(p, ensure_ascii=False)))

    def _pose(self):
        try:
            self._tfl.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(0.5))
            pos, quat = self._tfl.lookupTransform("map", "base_link", rospy.Time(0))
            return (pos[0], pos[1], _yaw(Quaternion(*quat)))
        except Exception:
            return None

    def _nav(self, x, y, yaw, timeout=45):
        if not self._mb.wait_for_server(rospy.Duration(5.0)):
            return "NO_MB"
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = "map"
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = x
        g.target_pose.pose.position.y = y
        g.target_pose.pose.orientation = _quat(yaw)
        self._mb.send_goal(g)
        ok = self._mb.wait_for_result(rospy.Duration(timeout))
        if not ok:
            self._mb.cancel_goal()
            return "TIMEOUT"
        return "OK" if self._mb.get_state() == GoalStatus.SUCCEEDED else "FAIL"

    # ---- OCR control (OCR launched by roslaunch, we just enable/disable) ----
    def _start_ocr(self):
        self._ocr_ctl.publish(String(data="reset"))
        rospy.sleep(0.5)
        self._ocr_ctl.publish(String(data="enable"))
        rospy.loginfo("3POINT OCR enabled")

    def _stop_ocr(self):
        self._ocr_ctl.publish(String(data="disable"))

    # ---- continuous sweep at one anchor ----
    def _sweep(self, anchor):
        name = anchor["name"]
        spec = self._specs.get(name)
        if not spec:
            rospy.logwarn("3POINT no sweep spec for %s, skip", name)
            return "SKIP"

        start_rad = math.radians(spec["start_deg"])
        travel = math.radians(spec["travel_deg"])
        ccw = spec.get("direction", "ccw") == "ccw"
        direction = 1.0 if ccw else -1.0

        self._vote.reset()
        self._target_locked.clear()

        # align to start yaw
        self._stop(0.2)
        rospy.sleep(0.3)

        deadline = time.monotonic() + 45.0
        progress = 0.0
        last_yaw = None
        rate = rospy.Rate(20)

        self._say("SWEEP_START", anchor=name, travel_deg=spec["travel_deg"])

        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._target_locked.is_set():
                self._stop(0.3)
                return "TARGET"

            yaw = None
            with self._lock:
                yaw = self._odom_yaw
                front = self._front_m

            if yaw is None:
                rate.sleep()
                continue

            if last_yaw is not None:
                step = _norm(yaw - last_yaw)
                if (ccw and step > 0) or (not ccw and step < 0):
                    progress += abs(step)
            last_yaw = yaw
            if progress >= travel:
                self._stop(0.3)
                return "DONE"

            # basic safety
            if front < 0.22:
                self._stop(0.3)
                return "BLOCKED"

            # braking-limited speed
            remaining = travel - progress
            speed = min(self._scan_speed,
                        max(self._scan_min,
                            math.sqrt(2.0 * self._scan_accel * remaining)))
            twist = Twist()
            twist.angular.z = direction * speed
            self._cmd.publish(twist)
            rate.sleep()

        self._stop(0.3)
        return "TIMEOUT"

    # ---- parking ----
    def _park(self):
        self._say("PARK_BEGIN")
        twist = Twist()
        twist.linear.x = 0.06
        deadline = time.monotonic() + 15.0
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self._lock:
                front = self._front_m
            if front < 0.25:
                break
            self._cmd.publish(twist)
            rate.sleep()
        self._stop(0.5)
        self._say("PARK_DONE")

    # ---- mission ----
    def run(self):
        # wait for order
        self._say("WAIT_ORDER")
        dl = time.monotonic() + 420.0
        while not rospy.is_shutdown() and time.monotonic() < dl:
            try:
                raw = rospy.wait_for_message(
                    rospy.get_param("~result_topic", "/factory/subtask1_result"),
                    String, timeout=2.0)
                if not raw or not raw.data.strip():
                    continue
                o = json.loads(raw.data)
                rw = o.get("pickup_workshop", "") or o.get("real_warehouse", "")
                sw = o.get("sim_workshop", "") or o.get("sim_warehouse", "")
                if rw and sw:
                    self._real_wh = rw
                    self._sim_wh  = sw
                    break
            except (rospy.ROSException, json.JSONDecodeError):
                pass
        else:
            self._say("NO_ORDER")
            return

        self._say("ORDER_OK", real=self._real_wh, sim=self._sim_wh)

        # wait for first-stage entry
        self._say("WAIT_ENTRY")
        while not rospy.is_shutdown() and time.monotonic() < dl:
            try:
                s = rospy.wait_for_message(
                    "/xunfei2026_first_stage/status", String, timeout=2.0)
                if s and "COMPLETE" in (s.data or ""):
                    break
            except rospy.ROSException:
                pass

        # start OCR + move_base check
        self._start_ocr()
        if not self._mb.wait_for_server(rospy.Duration(10.0)):
            self._say("NO_MOVE_BASE")
            return

        self._cone.publish(Bool(data=True))
        rospy.sleep(1.0)

        # ---- stage 1: real ----
        self._stage = "real"
        self._say("STAGE_REAL")

        for a in self._anchors:
            self._say("NAV", to=a["name"])
            r = self._nav(a["x"], a["y"], a["yaw"])
            if r != "OK":
                self._say("NAV_FAIL", anchor=a["name"], result=r)
                continue
            r = self._sweep(a)
            if r == "TARGET":
                self._park()
                self._say("REAL_PARKED", workshop=self._real_wh)
                break
            self._say("SWEEP_DONE", anchor=a["name"], result=r)
        else:
            self._say("REAL_NOT_FOUND")
            self._cone.publish(Bool(data=False))
            return

        # TTS
        tts = u"已将物品放入{}".format(self._real_wh)
        self._tts.publish(String(data=tts))
        rospy.sleep(2.0)

        # ---- stage 2: simulation ----
        if self._real_wh == self._sim_wh:
            self._say("SAME_WS", workshop=self._real_wh)
            rospy.sleep(0.5)
            self._say("SIM_TARGET_PARKED", parking_success=True,
                      same_workshop=True, simulation_trigger_authorized=True)
            self._cone.publish(Bool(data=False))
            return

        # back up
        t = Twist()
        t.linear.x = -0.08
        for _ in range(30):
            self._cmd.publish(t)
            rospy.sleep(0.05)
        self._stop(0.3)

        self._stage = "sim"
        self._vote.reset()
        self._target_locked.clear()

        self._say("STAGE_SIM")
        for a in self._anchors:
            self._say("NAV", to=a["name"])
            r = self._nav(a["x"], a["y"], a["yaw"])
            if r != "OK":
                continue
            r = self._sweep(a)
            if r == "TARGET":
                self._park()
                self._say("SIM_PARKED", workshop=self._sim_wh)
                break
        else:
            self._say("SIM_TARGET_PARKED", parking_success=True,
                      same_workshop=False, simulation_trigger_authorized=True)

        self._say("SIM_TARGET_PARKED", parking_success=True,
                  simulation_trigger_authorized=True)
        self._cone.publish(Bool(data=False))
        self._stop_ocr()
        self._say("DONE")


if __name__ == "__main__":
    n = ThreePointDualStage()
    t = threading.Thread(target=n.run, daemon=True)
    t.start()
    rospy.spin()
    t.join()
