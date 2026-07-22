#!/usr/bin/env python3
"""Attach the simulation workflow after the untouched real-car mission.

This node is deliberately passive until the original room-delivery manager
publishes a successful COMPLETE state.  It owns no real-car search or parking
logic.
"""

import json
import math
import os
import socket
import subprocess
import threading
import time
import uuid

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty


PROTOCOL = "xunfei2026_virtual_collaboration_v1"


class SimulationHandoff(object):
    def __init__(self):
        rospy.init_node("xunfei2026_simulation_handoff")

        self.result_topic = rospy.get_param(
            "~result_topic", "/factory/subtask1_result")
        self.real_status_topic = rospy.get_param(
            "~real_status_topic", "/factory_room/xunfei2026_delivery_status")
        self.status_topic = rospy.get_param(
            "~status_topic", "/factory/virtual_collaboration_status")
        self.tts_topic = rospy.get_param("~tts_topic", "/factory/tts_text")
        self.cmd_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.sim_trigger_host = rospy.get_param(
            "~sim_trigger_host", "255.255.255.255")
        self.sim_trigger_port = int(rospy.get_param("~sim_trigger_port", 39026))
        self.sim_completion_port = int(rospy.get_param(
            "~sim_completion_port", 39027))
        self.sim_trigger_repeats = int(rospy.get_param(
            "~sim_trigger_repeats", 8))
        self.sim_trigger_interval = float(rospy.get_param(
            "~sim_trigger_interval_s", 0.15))

        self.post_sim_goal_x = float(rospy.get_param("~post_sim_goal_x", 0.33))
        self.post_sim_goal_y = float(rospy.get_param("~post_sim_goal_y", -3.08))
        self.post_sim_goal_yaw = float(rospy.get_param(
            "~post_sim_goal_yaw", -math.pi / 2.0))
        self.post_sim_goal_timeout = float(rospy.get_param(
            "~post_sim_goal_timeout_s", 55.0))
        self.post_sim_goal_acceptance = max(0.03, float(rospy.get_param(
            "~post_sim_goal_acceptance_m", 0.10)))
        self.post_sim_yaw_acceptance = math.radians(max(
            2.0, float(rospy.get_param(
                "~post_sim_yaw_acceptance_deg", 8.0))))
        self.post_sim_visual_handoff_position = max(
            self.post_sim_goal_acceptance, float(rospy.get_param(
                "~post_sim_visual_handoff_position_m", 0.14)))
        self.post_sim_visual_handoff_yaw = math.radians(max(
            math.degrees(self.post_sim_yaw_acceptance),
            float(rospy.get_param(
                "~post_sim_visual_handoff_yaw_deg", 20.0))))
        self.post_sim_goal_attempts = max(1, int(rospy.get_param(
            "~post_sim_goal_attempts", 2)))
        self.post_sim_planner_tolerance = max(0.03, float(rospy.get_param(
            "~post_sim_planner_xy_tolerance_m", 0.08)))
        self.map_pose_topic = rospy.get_param("~map_pose_topic", "/amcl_pose")
        self.stop_line_launch_file = rospy.get_param(
            "~stop_line_launch_file", "xunfei2026_stop_line_parking_v1.launch")
        self.stop_line_status_topic = rospy.get_param(
            "~stop_line_status_topic", "/factory/stop_line_parking_status")
        self.stop_line_timeout = float(rospy.get_param(
            "~stop_line_timeout_s", 32.0))
        self.stop_line_crossing_speed = float(rospy.get_param(
            "~stop_line_crossing_speed_mps", 0.065))
        self.stop_line_crossing_max_distance = max(0.42, float(
            rospy.get_param("~stop_line_crossing_max_distance_m", 0.68)))
        self.follow_after_stop_line = bool(rospy.get_param(
            "~follow_after_stop_line", False))
        self.follow_mode = str(rospy.get_param(
            "~follow_mode", "middle")).strip().lower()
        if self.follow_mode not in ("left", "middle", "right"):
            raise ValueError("invalid follow_mode: {}".format(
                self.follow_mode))
        self.follow_launch_file = rospy.get_param(
            "~follow_launch_file", "follow_test.launch")
        self.follow_begin_topic = rospy.get_param(
            "~follow_begin_topic", "/follow_begin")
        self.follow_status_topic = rospy.get_param(
            "~follow_status_topic", "/flow_end/follow_test_status")
        self.follow_base_speed = float(rospy.get_param(
            "~follow_base_speed_mps", 0.30))
        self.follow_initial_turn_enabled = bool(rospy.get_param(
            "~follow_initial_turn_enabled", True))
        self.follow_initial_turn_angle = float(rospy.get_param(
            "~follow_initial_turn_angle_deg", 60.0))
        self.follow_initial_turn_speed = float(rospy.get_param(
            "~follow_initial_turn_angular_speed", 0.35))
        self.follow_initial_turn_pause = float(rospy.get_param(
            "~follow_initial_turn_pause_s", 0.5))
        self.follow_parking_enabled = bool(rospy.get_param(
            "~follow_parking_enabled", True))
        self.advance_distance = max(0.0, float(rospy.get_param(
            "~advance_before_follow_distance_m", 0.25)))
        self.advance_speed = max(0.04, float(rospy.get_param(
            "~advance_before_follow_speed_mps", 0.14)))
        self.advance_timeout = max(2.0, float(rospy.get_param(
            "~advance_before_follow_timeout_s", 7.0)))
        self.advance_front_stop = max(0.20, float(rospy.get_param(
            "~advance_before_follow_front_stop_m", 0.30)))
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.lock = threading.RLock()
        self.sim_target_key = ""
        self.sim_selected_item = ""
        self.sim_target_category = ""
        self.sim_target_warehouse = ""
        self.sim_mission_id = ""
        self.handoff_started = False
        self.post_simulation_started = False
        self.stop_line_process = None
        self.follow_process = None
        self.stop_line_event = threading.Event()
        self.stop_line_result = None
        self.follow_status = ""
        self.odom_xy = None
        self.odom_stamp = 0.0
        self.map_pose = None
        self.map_pose_stamp = 0.0
        self.front_clearance = None
        self.scan_stamp = 0.0

        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=10, latch=True)
        self.tts_pub = rospy.Publisher(self.tts_topic, String, queue_size=3)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.follow_begin_pub = rospy.Publisher(
            self.follow_begin_topic, String, queue_size=3)
        rospy.Subscriber(self.result_topic, String, self.result_callback,
                         queue_size=5)
        rospy.Subscriber(self.real_status_topic, String,
                         self.real_status_callback, queue_size=10)
        rospy.Subscriber(self.stop_line_status_topic, String,
                         self.stop_line_status_callback, queue_size=10)
        rospy.Subscriber(self.follow_status_topic, String,
                         self.follow_status_callback, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry,
                         self.odom_callback, queue_size=10)
        rospy.Subscriber(self.map_pose_topic, PoseWithCovarianceStamped,
                         self.map_pose_callback, queue_size=10)
        rospy.Subscriber(self.scan_topic, LaserScan,
                         self.scan_callback, queue_size=5)
        rospy.on_shutdown(self.shutdown)
        self.publish_state("WAITING_REAL_PARKING_COMPLETE")

    def publish_state(self, state, **values):
        payload = {"state": state, "stamp": time.time()}
        payload.update(values)
        text = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(String(data=text))
        rospy.logwarn("XUNFEI2026_SIM_HANDOFF %s", text)

    def result_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if str(payload.get("status", "")).lower() != "success":
            return
        values = (
            str(payload.get("sim_target_key", "")).strip(),
            str(payload.get("sim_selected_item", "")).strip(),
            str(payload.get("sim_target_category", "")).strip(),
            str(payload.get("sim_target_warehouse", "")).strip(),
        )
        if not all(values):
            return
        with self.lock:
            (self.sim_target_key, self.sim_selected_item,
             self.sim_target_category, self.sim_target_warehouse) = values
            if not self.sim_mission_id:
                self.sim_mission_id = uuid.uuid4().hex
        self.publish_state(
            "SIMULATION_ORDER_READY", simulation_item=self.sim_selected_item,
            simulation_warehouse=self.sim_target_warehouse)

    def real_status_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if (str(payload.get("state", "")) != "COMPLETE" or
                not bool(payload.get("parking_success", False))):
            return
        with self.lock:
            if self.handoff_started:
                return
            required = (
                self.sim_target_key, self.sim_selected_item,
                self.sim_target_category, self.sim_target_warehouse,
                self.sim_mission_id)
            if not all(required):
                self.publish_state(
                    "SIMULATION_TRIGGER_SKIPPED",
                    reason="simulation order is incomplete")
                return
            self.handoff_started = True
        worker = threading.Thread(target=self.start_simulation_handoff)
        worker.daemon = True
        worker.start()

    def trigger_payload(self):
        return {
            "protocol": PROTOCOL,
            "type": "start_simulation",
            "mission_id": self.sim_mission_id,
            "sim_target_key": self.sim_target_key,
            "sim_selected_item": self.sim_selected_item,
            "sim_target_category": self.sim_target_category,
            "sim_target_warehouse": self.sim_target_warehouse,
            "stamp": time.time(),
        }

    def start_simulation_handoff(self):
        listener_ready = threading.Event()
        listener = threading.Thread(
            target=self.completion_listener, args=(listener_ready,))
        listener.daemon = True
        listener.start()
        if not listener_ready.wait(3.0):
            self.publish_state(
                "SIMULATION_TRIGGER_FAILED",
                reason="completion listener did not bind")
            return
        payload = json.dumps(
            self.trigger_payload(), ensure_ascii=False).encode("utf-8")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        try:
            for _ in range(max(1, self.sim_trigger_repeats)):
                if rospy.is_shutdown():
                    return
                sock.sendto(payload, (self.sim_trigger_host,
                                      self.sim_trigger_port))
                time.sleep(max(0.02, self.sim_trigger_interval))
            self.publish_state(
                "SIMULATION_TRIGGER_SENT",
                simulation_item=self.sim_selected_item,
                simulation_warehouse=self.sim_target_warehouse,
                udp_host=self.sim_trigger_host,
                udp_port=self.sim_trigger_port)
        except Exception as exc:
            self.publish_state("SIMULATION_TRIGGER_FAILED", reason=str(exc))
        finally:
            sock.close()

    def completion_listener(self, ready_event):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.bind(("", self.sim_completion_port))
            sock.settimeout(1.0)
            ready_event.set()
            self.publish_state(
                "SIMULATION_COMPLETION_LISTENER_READY",
                udp_port=self.sim_completion_port)
            while not rospy.is_shutdown():
                try:
                    data, address = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                try:
                    payload = json.loads(data.decode("utf-8"))
                except Exception:
                    continue
                if (payload.get("protocol") != PROTOCOL or
                        payload.get("type") != "simulation_complete" or
                        payload.get("mission_id") != self.sim_mission_id):
                    continue
                announcement = "仿真任务已完成，已将{}放入{}。".format(
                    self.sim_selected_item, self.sim_target_warehouse)
                self.tts_pub.publish(String(data=announcement))
                self.publish_state(
                    "SIMULATION_COMPLETE_ANNOUNCED",
                    receiver=address[0], announcement=announcement)
                self.wait_for_tts_finished()
                with self.lock:
                    if self.post_simulation_started:
                        return
                    self.post_simulation_started = True
                route = threading.Thread(target=self.post_simulation_route)
                route.daemon = True
                route.start()
                return
        except Exception as exc:
            ready_event.set()
            self.publish_state(
                "SIMULATION_COMPLETION_LISTENER_FAILED", reason=str(exc))
        finally:
            sock.close()

    @staticmethod
    def tts_process_active():
        try:
            pids = [name for name in os.listdir("/proc") if name.isdigit()]
        except OSError:
            return False
        for pid in pids:
            try:
                with open("/proc/{}/cmdline".format(pid), "rb") as stream:
                    command = stream.read().replace(b"\0", b" ").decode(
                        "utf-8", "ignore")
            except (OSError, IOError):
                continue
            if ("xf_tts_stable.py" in command or
                    ("aplay" in command and "tts_result.pcm" in command)):
                return True
        return False

    def wait_for_tts_finished(self, start_timeout=7.0, finish_timeout=25.0):
        deadline = time.monotonic() + start_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.tts_process_active():
                break
            rospy.sleep(0.04)
        else:
            rospy.sleep(0.8)
            return
        deadline = time.monotonic() + finish_timeout
        quiet_since = None
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.tts_process_active():
                quiet_since = None
            elif quiet_since is None:
                quiet_since = time.monotonic()
            elif time.monotonic() - quiet_since >= 0.30:
                return
            rospy.sleep(0.04)

    def stop_robot(self, repeats=12):
        for _ in range(repeats):
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.025)

    def odom_callback(self, msg):
        with self.lock:
            self.odom_xy = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y))
            self.odom_stamp = time.monotonic()

    def map_pose_callback(self, msg):
        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (
            orientation.w * orientation.z +
            orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (
            orientation.y * orientation.y +
            orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        with self.lock:
            self.map_pose = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y), yaw)
            self.map_pose_stamp = time.monotonic()

    def scan_callback(self, msg):
        values = []
        half_angle = math.radians(18.0)
        for index, distance in enumerate(msg.ranges):
            if (not math.isfinite(distance) or distance < msg.range_min or
                    distance > msg.range_max):
                continue
            angle = msg.angle_min + index * msg.angle_increment
            if abs(angle) <= half_angle:
                values.append(float(distance))
        with self.lock:
            self.front_clearance = min(values) if values else None
            self.scan_stamp = time.monotonic()

    def follow_status_callback(self, msg):
        with self.lock:
            self.follow_status = str(msg.data or "").strip()

    def advance_before_line_follow(self):
        if self.advance_distance <= 0.0:
            return
        self.stop_robot(10)
        sensor_deadline = time.monotonic() + 3.0
        start = None
        while not rospy.is_shutdown() and time.monotonic() < sensor_deadline:
            now = time.monotonic()
            with self.lock:
                odom_xy = self.odom_xy
                odom_stamp = self.odom_stamp
                scan_stamp = self.scan_stamp
            if (odom_xy is not None and now - odom_stamp <= 0.6 and
                    now - scan_stamp <= 0.6):
                start = odom_xy
                break
            self.cmd_pub.publish(Twist())
            rospy.sleep(0.04)
        if start is None:
            raise RuntimeError("fresh odom/scan unavailable before 25cm advance")

        self.publish_state(
            "POST_SIM_ADVANCE_BEFORE_FOLLOW_START",
            distance=self.advance_distance, speed=self.advance_speed)
        deadline = time.monotonic() + self.advance_timeout
        rate = rospy.Rate(30)
        travelled = 0.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            now = time.monotonic()
            with self.lock:
                odom_xy = self.odom_xy
                odom_stamp = self.odom_stamp
                clearance = self.front_clearance
                scan_stamp = self.scan_stamp
            if (odom_xy is None or now - odom_stamp > 0.6 or
                    now - scan_stamp > 0.6):
                self.stop_robot(4)
                raise RuntimeError("odom/scan became stale during 25cm advance")
            travelled = math.hypot(
                odom_xy[0] - start[0], odom_xy[1] - start[1])
            remaining = self.advance_distance - travelled
            if remaining <= 0.008:
                self.stop_robot(14)
                self.publish_state(
                    "POST_SIM_ADVANCE_BEFORE_FOLLOW_COMPLETE",
                    requested_distance=self.advance_distance,
                    travelled=travelled)
                return
            if clearance is not None and clearance <= self.advance_front_stop:
                self.stop_robot(14)
                raise RuntimeError(
                    "25cm advance blocked at {:.3f}m front clearance".format(
                        clearance))
            # Brake smoothly over the last few centimetres without inserting
            # a stationary pause before the hard-turn follower takes over.
            speed = min(
                self.advance_speed,
                max(0.05, math.sqrt(max(0.0, 1.2 * remaining))))
            command = Twist()
            command.linear.x = speed
            self.cmd_pub.publish(command)
            rospy.logwarn_throttle(
                0.35, "POST_SIM_ADVANCE travelled=%.3f/%.3fm vx=%.3f front=%s",
                travelled, self.advance_distance, speed,
                "none" if clearance is None else "{:.3f}".format(clearance))
            rate.sleep()
        self.stop_robot(16)
        raise RuntimeError(
            "25cm advance timeout after {:.3f}m".format(travelled))

    @staticmethod
    def ros_bool(value):
        return "true" if value else "false"

    def start_line_following(self):
        self.stop_robot(8)
        command = [
            "roslaunch", "flow_end", self.follow_launch_file,
            "path_select:={}".format(self.follow_mode),
            "base_speed:={}".format(self.follow_base_speed),
            "initial_turn_enabled:={}".format(
                self.ros_bool(self.follow_initial_turn_enabled)),
            "initial_turn_angle_deg:={}".format(
                self.follow_initial_turn_angle),
            "initial_turn_angular_speed:={}".format(
                self.follow_initial_turn_speed),
            "initial_turn_pause_sec:={}".format(
                self.follow_initial_turn_pause),
            "parking_enabled:={}".format(
                self.ros_bool(self.follow_parking_enabled)),
        ]
        with self.lock:
            self.follow_status = ""
        self.follow_process = subprocess.Popen(command)
        deadline = time.monotonic() + 8.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.follow_process.poll() is not None:
                raise RuntimeError("line follower exited before ready")
            if self.follow_begin_pub.get_num_connections() > 0:
                break
            rospy.sleep(0.05)
        else:
            raise RuntimeError("line follower /follow_begin subscriber timeout")

        mode_command = self.follow_mode.capitalize()
        for _ in range(5):
            self.follow_begin_pub.publish(String(data=mode_command))
            rospy.sleep(0.10)
        deadline = time.monotonic() + 3.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                status = self.follow_status
            if (status.startswith("RUNNING_") or
                    status.startswith("ALIGNING_")):
                self.publish_state(
                    "POST_SIM_LINE_FOLLOW_STARTED", mode=self.follow_mode,
                    status=status, hard_turn=(
                        self.follow_initial_turn_enabled and
                        self.follow_mode in ("left", "right")),
                    hard_turn_angle_deg=self.follow_initial_turn_angle)
                return
            if self.follow_process.poll() is not None:
                raise RuntimeError("line follower exited after command")
            rospy.sleep(0.05)
        raise RuntimeError("line follower did not accept {}".format(
            mode_command))

    def navigate_to_stop_line_observation(self):
        self.publish_state(
            "POST_SIM_NAVIGATION_START", x=self.post_sim_goal_x,
            y=self.post_sim_goal_y, yaw=self.post_sim_goal_yaw)
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            raise RuntimeError("post-simulation move_base unavailable")
        try:
            from dynamic_reconfigure.client import Client
            planner_config = Client(
                "/move_base/TebLocalPlannerROS", timeout=2.0)
            planner_config.update_configuration({
                "xy_goal_tolerance": self.post_sim_planner_tolerance})
            self.publish_state(
                "POST_SIM_NAVIGATION_TOLERANCE_SET",
                xy_goal_tolerance_m=self.post_sim_planner_tolerance)
        except Exception as exc:
            # AMCL verification below remains mandatory.  Log this explicitly
            # so a missing dynamic-reconfigure service cannot look like a
            # successfully tightened planner.
            self.publish_state(
                "POST_SIM_NAVIGATION_TOLERANCE_WARNING", reason=str(exc))
        deadline = time.monotonic() + self.post_sim_goal_timeout
        last_error = "navigation did not start"
        for attempt in range(1, self.post_sim_goal_attempts + 1):
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.post_sim_goal_x
            goal.target_pose.pose.position.y = self.post_sim_goal_y
            goal.target_pose.pose.orientation.z = math.sin(
                self.post_sim_goal_yaw / 2.0)
            goal.target_pose.pose.orientation.w = math.cos(
                self.post_sim_goal_yaw / 2.0)
            client.send_goal(goal)
            if not client.wait_for_result(rospy.Duration(remaining)):
                client.cancel_goal()
                last_error = "navigation timeout"
            elif client.get_state() != GoalStatus.SUCCEEDED:
                last_error = "move_base state={}".format(client.get_state())
            else:
                # move_base's action result alone is insufficient: its local
                # planner may report success at the edge of its goal tolerance.
                # Require a fresh AMCL pose and verify the physical endpoint.
                pose_deadline = min(deadline, time.monotonic() + 1.2)
                pose = None
                while (not rospy.is_shutdown() and
                       time.monotonic() < pose_deadline):
                    now = time.monotonic()
                    with self.lock:
                        candidate = self.map_pose
                        candidate_stamp = self.map_pose_stamp
                    if candidate is not None and now - candidate_stamp <= 0.8:
                        pose = candidate
                        break
                    rospy.sleep(0.04)
                if pose is None:
                    last_error = "fresh AMCL pose unavailable"
                else:
                    position_error = math.hypot(
                        pose[0] - self.post_sim_goal_x,
                        pose[1] - self.post_sim_goal_y)
                    yaw_error = abs(math.atan2(
                        math.sin(pose[2] - self.post_sim_goal_yaw),
                        math.cos(pose[2] - self.post_sim_goal_yaw)))
                    strict_endpoint = (
                        position_error <= self.post_sim_goal_acceptance and
                        yaw_error <= self.post_sim_yaw_acceptance)
                    visual_handoff = (
                        position_error <=
                        self.post_sim_visual_handoff_position and
                        yaw_error <= self.post_sim_visual_handoff_yaw)
                    if strict_endpoint or visual_handoff:
                        self.stop_robot(14)
                        acceptance_mode = (
                            "strict_endpoint" if strict_endpoint else
                            "stop_line_visual_handoff")
                        self.publish_state(
                            "POST_SIM_OBSERVATION_POINT_REACHED",
                            actual_x=pose[0], actual_y=pose[1],
                            actual_yaw=pose[2],
                            position_error_m=position_error,
                            yaw_error_deg=math.degrees(yaw_error),
                            navigation_attempt=attempt,
                            acceptance_mode=acceptance_mode)
                        if not strict_endpoint:
                            self.publish_state(
                                "POST_SIM_STOP_LINE_VISUAL_ALIGNMENT_HANDOFF",
                                position_error_m=position_error,
                                yaw_error_deg=math.degrees(yaw_error),
                                position_limit_m=
                                self.post_sim_visual_handoff_position,
                                yaw_limit_deg=math.degrees(
                                    self.post_sim_visual_handoff_yaw))
                        return
                    last_error = (
                        "endpoint outside tolerance: actual=({:.3f}, {:.3f}, "
                        "{:.1f}deg), position_error={:.3f}m, "
                        "yaw_error={:.1f}deg".format(
                            pose[0], pose[1], math.degrees(pose[2]),
                            position_error, math.degrees(yaw_error)))
            self.publish_state(
                "POST_SIM_NAVIGATION_RETRY", attempt=attempt,
                max_attempts=self.post_sim_goal_attempts, reason=last_error)
            try:
                rospy.wait_for_service("/move_base/clear_costmaps", timeout=2.0)
                rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()
            except Exception:
                pass
            rospy.sleep(0.15)
        raise RuntimeError("post-simulation {}".format(last_error))

    def stop_line_status_callback(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        if payload.get("state") not in ("STOP_LINE_PARKED", "STOP_LINE_FAILED"):
            return
        self.stop_line_result = payload
        self.stop_line_event.set()

    def run_stop_line_parking(self):
        self.stop_line_event.clear()
        self.stop_line_result = None
        command = [
            "roslaunch", "iden_controller", self.stop_line_launch_file,
            "status_topic:={}".format(self.stop_line_status_topic),
            "timeout_s:={}".format(self.stop_line_timeout - 2.0),
            "crossing_speed_mps:={}".format(self.stop_line_crossing_speed),
            "crossing_max_distance_m:={}".format(
                self.stop_line_crossing_max_distance),
        ]
        self.stop_line_process = subprocess.Popen(command)
        deadline = time.monotonic() + self.stop_line_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.stop_line_event.wait(0.10):
                break
            if self.stop_line_process.poll() is not None:
                self.stop_line_event.wait(0.5)
                break
        result = self.stop_line_result
        self.stop_process(self.stop_line_process)
        self.stop_line_process = None
        self.stop_robot(16)
        if not result or result.get("state") != "STOP_LINE_PARKED":
            reason = ("stop-line status timeout" if not result else
                      str(result.get("reason", "unknown")))
            raise RuntimeError(reason)
        self.publish_state("POST_SIM_STOP_LINE_PARKED")

    def post_simulation_route(self):
        try:
            self.navigate_to_stop_line_observation()
            self.run_stop_line_parking()
            if self.follow_after_stop_line:
                self.advance_before_line_follow()
                self.start_line_following()
        except Exception as exc:
            self.stop_robot(20)
            self.publish_state("POST_SIM_STOP_LINE_FAILED", reason=str(exc))

    @staticmethod
    def stop_process(process):
        if process is None or process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.kill()

    def shutdown(self):
        self.stop_process(self.stop_line_process)
        self.stop_line_process = None
        self.stop_process(self.follow_process)
        self.follow_process = None
        self.stop_robot(8)


if __name__ == "__main__":
    SimulationHandoff()
    rospy.spin()
