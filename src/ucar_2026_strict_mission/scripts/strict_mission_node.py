#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fail-safe task4 coordinator using the flow_end bird's-eye yellow detector."""
from __future__ import annotations
import json, math, os, subprocess, sys, threading, time
import actionlib, rospy, tf2_ros
from dynamic_reconfigure.msg import BoolParameter, Config, DoubleParameter
from dynamic_reconfigure.srv import Reconfigure
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from ucar_2026_strict_mission.logic import (
    heading_alignment_command, track_launch_for_decision,
    traffic_decision_from_payload)

TERMINAL = frozenset(("DONE", "FAULT"))

def norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

class StrictMissionNode:
    def __init__(self):
        self.lock = threading.RLock(); self.state = "WAIT_START"; self.started = False
        self.fault_reason = ""; self.shutdown_event = threading.Event(); self.start_event = threading.Event()
        self.parked_event = threading.Event(); self.traffic_event = threading.Event()
        self.selected_decision = None; self.last_traffic_decision = None; self.traffic_hits = 0
        self.track_process = None; self.track_status = {}; self.odom_pose = None; self.odom_received_at = 0.0
        self.latest_line = None; self.latest_line_at = 0.0; self.invalid_line_since = None; self.line_hits = 0; self.line_history = []
        self.line_lock_acquired = False; self.fallback_start_pose = None; self.fallback_start_at = 0.0; self.fallback_target_m = 0.0
        self.cmd_pub = rospy.Publisher(rospy.get_param("~cmd_vel_topic", "/cmd_vel"), Twist, queue_size=1)
        self.status_pub = rospy.Publisher(rospy.get_param("~status_topic", "/strict_mission/status"), String, queue_size=10, latch=True)
        self.yellow_topic = rospy.get_param("~yellow_line_topic", "/strict_mission/yellow_line")
        rospy.Subscriber(self.yellow_topic, String, self.yellow_line_callback, queue_size=10)
        rospy.Subscriber("/odom", Odometry, self.odom_callback, queue_size=5)
        rospy.Subscriber(rospy.get_param("~traffic_topic", "/traffic_light_rknn_test/detections"), String, self.traffic_callback, queue_size=10)
        for topic in ("/track_end_stop/status", "/right_track_end_stop/status", "/stable_right_track_end_stop/status"):
            rospy.Subscriber(topic, String, self.track_status_callback, callback_args=topic, queue_size=10)
        rospy.Service("~start", Trigger, self.start_service); rospy.Service("~abort", Trigger, self.abort_service)
        self.move_base = actionlib.SimpleActionClient(rospy.get_param("~move_base_action", "move_base"), MoveBaseAction)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10)); self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.line_confirm_frames = max(1, int(rospy.get_param("~yellow_line_confirm_frames", 5)))
        self.line_spread_m = float(rospy.get_param("~yellow_line_distance_spread_m", 0.025))
        self.line_target_m = float(rospy.get_param("~yellow_line_target_clearance_m", 0.07))
        self.line_max_m = float(rospy.get_param("~yellow_line_max_clearance_m", 0.10))
        self.line_search_max_m = max(self.line_max_m + 0.20, float(rospy.get_param("~yellow_line_search_max_distance_m", 2.0)))
        self.line_center_tol = float(rospy.get_param("~yellow_line_center_tolerance_m", 0.05))
        self.line_yaw_tol = float(rospy.get_param("~yellow_line_yaw_tolerance_deg", 5.0))
        self.line_lost_timeout = max(0.2, float(rospy.get_param("~yellow_line_lost_timeout_sec", 1.0)))
        self.line_timeout = max(5.0, float(rospy.get_param("~line_approach_timeout_sec", 75.0)))
        self.last_memory_check = 0.0; self.rss_limit = float(rospy.get_param("~memory_rss_limit_mb", 512.0))
        rospy.Timer(rospy.Duration(0.05), self.watchdog_callback); rospy.on_shutdown(self.shutdown)
        self.publish_status("waiting for explicit start")
        threading.Thread(target=self.run, daemon=True).start()

    def publish_stop(self): self.cmd_pub.publish(Twist())

    def publish_status(self, detail="", **extra):
        with self.lock:
            d = self.latest_line.copy() if isinstance(self.latest_line, dict) else {}
            state, error, decision = self.state, self.fault_reason, self.selected_decision
        payload = {"state": state, "detail": detail, "decision": decision, "line": d,
                   "line_confirm_hits": self.line_hits, "stamp": rospy.Time.now().to_sec()}
        if state == "FAULT": payload["error"] = error
        payload.update(extra); self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))

    def set_fault(self, reason):
        with self.lock:
            if self.state in TERMINAL: return
            self.state = "FAULT"; self.fault_reason = str(reason); self.shutdown_event.set()
        try: self.move_base.cancel_all_goals()
        except Exception: pass
        self.publish_stop(); self.publish_status("fail-safe stop", error=str(reason)); rospy.logerr("strict mission fault: %s", reason)

    def start_service(self, _):
        with self.lock:
            if self.started: return TriggerResponse(False, "mission already started")
            self.started = True; self.start_event.set()
        return TriggerResponse(True, "strict mission started")

    def abort_service(self, _): self.set_fault("operator abort"); return TriggerResponse(True, "vehicle stopped")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation; yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw); self.odom_received_at = time.monotonic()

    def fallback_forward_step(self, now):
        """Bounded odometry-only compensation after a previously valid lock."""
        with self.lock:
            pose, age, start, target, started = self.odom_pose, now - self.odom_received_at, self.fallback_start_pose, self.fallback_target_m, self.fallback_start_at
        if pose is None or age > 0.30 or start is None:
            self.publish_stop(); self.set_fault("yellow fallback odometry unavailable"); return
        progress = math.cos(start[2]) * (pose[0] - start[0]) + math.sin(start[2]) * (pose[1] - start[1])
        timeout = float(rospy.get_param("~yellow_line_fallback_timeout_sec", 5.0))
        if progress >= target - 0.005:
            self.publish_stop(); self.set_fault("yellow line lost after bounded fallback")
            return
        if now - started > timeout:
            self.publish_stop(); self.set_fault("yellow line fallback timed out"); return
        command = Twist(); command.linear.x = float(rospy.get_param("~yellow_line_fallback_speed_mps", 0.03)); self.cmd_pub.publish(command)
        self.publish_status("yellow line lost; bounded odometry fallback", fallback_progress_m=progress, fallback_target_m=target, commanded_x=command.linear.x)

    def yellow_line_callback(self, msg):
        now = time.monotonic()
        try: d = json.loads(msg.data)
        except (TypeError, ValueError):
            self.publish_stop(); self.set_fault("yellow detector emitted invalid JSON"); return
        with self.lock:
            self.latest_line = d; self.latest_line_at = now
            active = self.state == "APPROACH_LINE"
        if not active: return
        valid = bool(d.get("valid")) and all(k in d for k in ("longitudinal_m", "lateral_m", "angle_deg", "confidence"))
        distance = float(d.get("longitudinal_m", -1)); lateral = float(d.get("lateral_m", 0)); angle = float(d.get("angle_deg", 0)); confidence = float(d.get("confidence", 0))
        if not valid or confidence < float(rospy.get_param("~yellow_line_min_confidence", 0.45)) or distance <= 0 or distance > self.line_search_max_m:
            if self.invalid_line_since is None: self.invalid_line_since = now
            self.line_hits = 0; self.line_history = []
            if self.line_lock_acquired and self.fallback_start_pose is not None: self.fallback_forward_step(now)
            else: self.publish_stop()
            self.publish_status("yellow line invalid; holding stop" if not self.line_lock_acquired else "yellow line invalid; bounded fallback armed", detector_reason=d.get("reason")); return
        self.invalid_line_since = None
        if not self.line_lock_acquired:
            with self.lock: self.fallback_start_pose = self.odom_pose; self.fallback_start_at = now
            self.fallback_target_m = min(float(rospy.get_param("~yellow_line_fallback_distance_m", 0.155)), max(0.0, distance - self.line_target_m))
            self.line_lock_acquired = True
        yaw_ok = abs(angle) <= self.line_yaw_tol; lateral_ok = abs(lateral) <= self.line_center_tol
        cmd = Twist()
        if not yaw_ok:
            cmd.angular.z = max(-0.16, min(0.16, float(rospy.get_param("~line_yaw_command_sign", -1.0)) * float(rospy.get_param("~line_yaw_kp", .8)) * math.radians(angle)))
        elif not lateral_ok:
            cmd.linear.y = max(-.045, min(.045, float(rospy.get_param("~line_lateral_command_sign", -1.0)) * float(rospy.get_param("~line_lateral_kp", .1)) * lateral))
        elif distance > self.line_target_m:
            speed = float(rospy.get_param("~line_forward_speed_near", .035)) if distance < float(rospy.get_param("~line_near_distance_m", .30)) else float(rospy.get_param("~line_forward_speed_far", .10))
            cmd.linear.x = speed
        self.cmd_pub.publish(cmd)
        self.line_history.append(distance); self.line_history = self.line_history[-self.line_confirm_frames:]
        if yaw_ok and lateral_ok and self.line_target_m - .02 <= distance <= self.line_max_m:
            self.line_hits += 1
        else: self.line_hits = 0
        self.publish_status("birdview yellow-line approach", line_distance_m=distance, line_lateral_m=lateral, line_angle_deg=angle, confidence=confidence, commanded_x=cmd.linear.x, commanded_y=cmd.linear.y, commanded_yaw=cmd.angular.z)
        if self.line_hits >= self.line_confirm_frames and len(self.line_history) >= self.line_confirm_frames and max(self.line_history)-min(self.line_history) <= self.line_spread_m:
            self.publish_stop()
            with self.lock:
                if self.state != "APPROACH_LINE": return
                self.state = "WAIT_TRAFFIC"; self.parked_event.set()
            self.publish_status("yellow_line_stopped", final_clearance_m=distance)

    def traffic_callback(self, msg):
        try: decision = traffic_decision_from_payload(json.loads(msg.data))
        except (TypeError, ValueError, KeyError): return
        if not decision: return
        if decision == self.last_traffic_decision: self.traffic_hits += 1
        else: self.last_traffic_decision, self.traffic_hits = decision, 1
        if self.traffic_hits >= max(1, int(rospy.get_param("~traffic_confirm_frames", 3))):
            self.selected_decision = decision; self.traffic_event.set(); self.publish_status("traffic direction confirmed")

    def track_status_callback(self, msg, topic): self.track_status[topic] = str(msg.data).strip()

    def watchdog_callback(self, _):
        now = time.monotonic()
        if self.state == "APPROACH_LINE":
            with self.lock: age = now - self.latest_line_at
            if age > .25:
                if self.line_lock_acquired and self.fallback_start_pose is not None: self.fallback_forward_step(now)
                else: self.publish_stop()
            with self.lock: invalid_age = 0.0 if self.invalid_line_since is None else now - self.invalid_line_since
            if age > self.line_lost_timeout and not self.line_lock_acquired: self.set_fault("yellow line detector timeout")
            elif invalid_age > self.line_lost_timeout and not self.line_lock_acquired: self.set_fault("yellow line geometry not locked")
        elif self.state in ("WAIT_TRAFFIC", "FAULT", "DONE"): self.publish_stop()
        if now - self.last_memory_check > .5:
            self.last_memory_check = now
            try:
                rss = int(open("/proc/self/statm").read().split()[1]) * os.sysconf("SC_PAGE_SIZE") / 1048576.0
                if rss > self.rss_limit: self.set_fault("strict mission RSS %.1fMB exceeded %.1fMB" % (rss, self.rss_limit))
            except (IOError, OSError, ValueError): pass

    def navigate_to_staging_pose(self):
        if not bool(rospy.get_param("~traffic_pose_configured", False)): raise RuntimeError("traffic_pose_configured is false")
        if not self.move_base.wait_for_server(rospy.Duration(10)): raise RuntimeError("move_base action server unavailable")
        g = MoveBaseGoal(); g.target_pose.header.frame_id = rospy.get_param("~traffic_frame", "map"); g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = float(rospy.get_param("~traffic_staging_x")); g.target_pose.pose.position.y = float(rospy.get_param("~traffic_staging_y")); yaw = float(rospy.get_param("~traffic_staging_yaw")); g.target_pose.pose.orientation.z = math.sin(yaw/2); g.target_pose.pose.orientation.w = math.cos(yaw/2)
        self.move_base.send_goal(g); timeout = float(rospy.get_param("~navigation_timeout_sec", 120));
        if not self.move_base.wait_for_result(rospy.Duration(timeout)): self.move_base.cancel_goal(); raise RuntimeError("navigation to stop-line staging pose timed out")
        if self.move_base.get_state() != 3: raise RuntimeError("navigation failed with action state %s" % self.move_base.get_state())

    def align_to_staging_heading(self):
        # Navigation already provides the staging pose; hold zero before detector takes control.
        self.publish_stop(); time.sleep(.2)

    def wait_event(self, event, timeout, description):
        if not event.wait(timeout): raise RuntimeError(description + " timed out")
        if self.state == "FAULT": raise RuntimeError(self.fault_reason)

    def launch_track(self, decision):
        launch_file, status_topic, finish_value = track_launch_for_decision(decision)
        self.track_process = subprocess.Popen(["roslaunch", "ucar_2026_track_end_stop", launch_file, "start_driver:=false", "start_camera:=false", "start_viewer:=false"])
        return status_topic, finish_value

    def run(self):
        self.start_event.wait()
        try:
            self.state = "NAVIGATING"; self.publish_status("navigating to staging pose"); self.navigate_to_staging_pose(); self.publish_stop()
            self.state = "ALIGN_STAGING_HEADING"; self.align_to_staging_heading()
            with self.lock:
                self.state = "APPROACH_LINE"; self.latest_line = None; self.latest_line_at = time.monotonic(); self.invalid_line_since = None; self.line_lock_acquired = False; self.fallback_start_pose = None; self.fallback_start_at = 0.0; self.fallback_target_m = 0.0; self.line_hits = 0; self.line_history = []
            self.publish_status("yellow_line_searching: flow_end birdview detector armed")
            self.wait_event(self.parked_event, self.line_timeout, "strict stop-line approach")
            self.publish_stop(); time.sleep(float(rospy.get_param("~stop_settle_sec", .6))); self.publish_status("vehicle held; waiting for traffic consensus")
            self.wait_event(self.traffic_event, float(rospy.get_param("~traffic_timeout_sec", 180)), "traffic recognition")
            self.state = "TRACKING"; topic, finish = self.launch_track(self.selected_decision); self.publish_status("matching track controller launched", track_status_topic=topic, expected_finish=finish)
            deadline = time.monotonic() + float(rospy.get_param("~track_timeout_sec", 420))
            while time.monotonic() < deadline and not rospy.is_shutdown():
                if self.track_process.poll() is not None: raise RuntimeError("track controller exited before finish")
                if self.track_status.get(topic) == finish: break
                time.sleep(.05)
            else: raise RuntimeError("line following timed out")
            self.publish_stop(); self.state = "DONE"; self.publish_status("strict post-warehouse mission completed")
        except Exception as exc: self.set_fault(str(exc))

    def shutdown(self):
        self.shutdown_event.set(); self.publish_stop()
        try: self.move_base.cancel_all_goals()
        except Exception: pass
        if self.track_process and self.track_process.poll() is None: self.track_process.terminate()

def main(): rospy.init_node("strict_mission"); StrictMissionNode(); rospy.spin()
if __name__ == "__main__": main()
