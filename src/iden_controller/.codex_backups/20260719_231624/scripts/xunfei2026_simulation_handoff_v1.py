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
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
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
        self.post_sim_goal_y = float(rospy.get_param("~post_sim_goal_y", -2.74))
        self.post_sim_goal_yaw = float(rospy.get_param(
            "~post_sim_goal_yaw", -math.pi / 2.0))
        self.post_sim_goal_timeout = float(rospy.get_param(
            "~post_sim_goal_timeout_s", 55.0))
        self.stop_line_launch_file = rospy.get_param(
            "~stop_line_launch_file", "xunfei2026_stop_line_parking_v1.launch")
        self.stop_line_status_topic = rospy.get_param(
            "~stop_line_status_topic", "/factory/stop_line_parking_status")
        self.stop_line_timeout = float(rospy.get_param(
            "~stop_line_timeout_s", 32.0))
        self.stop_line_crossing_speed = float(rospy.get_param(
            "~stop_line_crossing_speed_mps", 0.065))

        self.lock = threading.RLock()
        self.sim_target_key = ""
        self.sim_selected_item = ""
        self.sim_target_category = ""
        self.sim_target_warehouse = ""
        self.sim_mission_id = ""
        self.handoff_started = False
        self.post_simulation_started = False
        self.stop_line_process = None
        self.stop_line_event = threading.Event()
        self.stop_line_result = None

        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=10, latch=True)
        self.tts_pub = rospy.Publisher(self.tts_topic, String, queue_size=3)
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        rospy.Subscriber(self.result_topic, String, self.result_callback,
                         queue_size=5)
        rospy.Subscriber(self.real_status_topic, String,
                         self.real_status_callback, queue_size=10)
        rospy.Subscriber(self.stop_line_status_topic, String,
                         self.stop_line_status_callback, queue_size=10)
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

    def navigate_to_stop_line_observation(self):
        self.publish_state(
            "POST_SIM_NAVIGATION_START", x=self.post_sim_goal_x,
            y=self.post_sim_goal_y, yaw=self.post_sim_goal_yaw)
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if not client.wait_for_server(rospy.Duration(5.0)):
            raise RuntimeError("post-simulation move_base unavailable")
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
        if not client.wait_for_result(rospy.Duration(self.post_sim_goal_timeout)):
            client.cancel_goal()
            raise RuntimeError("post-simulation navigation timeout")
        if client.get_state() != GoalStatus.SUCCEEDED:
            try:
                rospy.wait_for_service("/move_base/clear_costmaps", timeout=2.0)
                rospy.ServiceProxy("/move_base/clear_costmaps", Empty)()
            except Exception:
                pass
            raise RuntimeError(
                "post-simulation navigation state={}".format(client.get_state()))
        self.stop_robot(14)
        self.publish_state("POST_SIM_OBSERVATION_POINT_REACHED")

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
        self.stop_robot(8)


if __name__ == "__main__":
    SimulationHandoff()
    rospy.spin()
