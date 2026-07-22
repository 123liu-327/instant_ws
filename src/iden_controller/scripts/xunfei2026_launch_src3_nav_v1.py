#!/usr/bin/python3
"""Launch the src3-inspired room nav in place of the legacy anchor-coverage manager.

Reuses the voice/QR/LLM fusion, first-stage entry, and simulation handoff
from the existing subtask1 launch.  Only the room-delivery manager is swapped
for the new src3-based implementation.
"""

import os
import sys
from xml.dom import minidom

import roslaunch
import rospkg


OLD_MANAGER  = "xunfei2026_room_delivery_anchor_coverage_v1.py"
NEW_MANAGER  = "xunfei2026_room_nav_src3_v1.py"
OLD_HANDOFF  = "xunfei2026_simulation_handoff_tts_trigger_v1.py"
# Keep the proven v3 second-parking handoff.
NEW_HANDOFF  = "xunfei2026_simulation_handoff_second_parking_v3.py"
SOURCE_LAUNCH = "subtask1_xunfei2026_complete_delivery_anchor_coverage_v1.launch"

# Expose these as recognised launch-arg overrides.
KNOWN_OVERRIDES = frozenset((
    "goal_x", "goal_y", "goal_yaw",
    "start_base", "start_lidar", "start_camera",
    "use_spark", "follow_after_stop_line", "follow_mode",
    "post_sim_goal_x", "post_sim_goal_y", "post_sim_goal_yaw",
))


def _set_node_param(node, name, value):
    matches = [
        child for child in node.childNodes
        if child.nodeType == child.ELEMENT_NODE
        and child.tagName == "param"
        and child.getAttribute("name") == name
    ]
    if len(matches) != 1:
        raise RuntimeError("expected one param {} in node {}, found {}".format(
            name, node.getAttribute("name"), len(matches)))
    matches[0].setAttribute("value", str(value))


def _add_node_param(node, name, value):
    existing = [
        child for child in node.childNodes
        if child.nodeType == child.ELEMENT_NODE
        and child.tagName == "param"
        and child.getAttribute("name") == name
    ]
    if existing:
        existing[0].setAttribute("value", str(value))
        return
    el = node.ownerDocument.createElement("param")
    el.setAttribute("name", name)
    el.setAttribute("value", str(value))
    node.appendChild(el)


def load_launch_xml(overrides):
    pkg_dir = rospkg.RosPack().get_path("iden_controller")
    path = os.path.join(pkg_dir, "launch", SOURCE_LAUNCH)
    with open(path, "r", encoding="utf-8") as fh:
        doc = minidom.parseString(fh.read())

    # ---- swap room manager ----
    mgrs = [n for n in doc.getElementsByTagName("node")
            if n.getAttribute("type") == OLD_MANAGER]
    if len(mgrs) != 1:
        raise RuntimeError("expected 1 manager, found {}".format(len(mgrs)))
    mgr = mgrs[0]
    mgr.setAttribute("type", NEW_MANAGER)

    # src3-nav params
    _add_node_param(mgr, "scan_step_deg",          "20.0")
    _add_node_param(mgr, "scan_speed_rps",         "0.35")
    _add_node_param(mgr, "scan_dwell_s",           "0.65")
    _add_node_param(mgr, "scan_max_dwell_s",       "2.0")
    _add_node_param(mgr, "ocr_required_hits",      "2")
    _add_node_param(mgr, "ocr_window_s",           "1.5")
    _add_node_param(mgr, "vote_window_size",       "7")
    _add_node_param(mgr, "vote_min_count",         "3")
    _add_node_param(mgr, "nav_goal_timeout_s",     "40.0")
    _add_node_param(mgr, "nav_xy_tolerance_m",     "0.12")
    _add_node_param(mgr, "nav_yaw_tolerance_deg",  "10.0")
    _add_node_param(mgr, "wait_after_tts_s",       "1.0")
    _add_node_param(mgr, "same_workshop_confirm_s","0.5")
    _add_node_param(mgr, "laser_offset_x_m",       "0.11")
    _add_node_param(mgr, "laser_offset_y_m",       "0.0")
    _add_node_param(mgr, "laser_yaw_rad",          "-0.07")

    # Carry existing params the new node reads
    for name in ("ocr_topic", "ocr_control_topic", "ocr_health_topic",
                 "result_topic", "tts_topic", "cone_control_topic",
                 "odom_topic", "scan_topic", "status_topic",
                 "mission_timeout_s", "ocr_ready_timeout_s"):
        # already present from base launch – just keep them
        pass

    # Load anchor list from base launch's rosparam
    # The base launch embeds anchors as rosparam inside the manager node.
    # We keep that unchanged so the new node reads ~coverage_anchors.

    # ---- swap handoff ----
    hoffs = [n for n in doc.getElementsByTagName("node")
             if n.getAttribute("name") == "xunfei2026_simulation_handoff"]
    if len(hoffs) != 1:
        raise RuntimeError("expected 1 handoff, found {}".format(len(hoffs)))
    hoff = hoffs[0]
    if hoff.getAttribute("type") != OLD_HANDOFF:
        raise RuntimeError("unexpected handoff type: {}".format(
            hoff.getAttribute("type")))
    hoff.setAttribute("type", NEW_HANDOFF)
    _set_node_param(hoff, "post_sim_visual_handoff_position_m", "0.18")
    _set_node_param(hoff, "post_sim_visual_handoff_yaw_deg", "28.0")
    _set_node_param(hoff, "stop_line_launch_file",
                    "xunfei2026_stop_line_parking_visual_handoff_v2.launch")

    # ---- apply CLI overrides ----
    launch_args = {
        n.getAttribute("name"): n
        for n in doc.getElementsByTagName("arg")
        if n.hasAttribute("name")
    }
    for name, value in overrides.items():
        if name not in launch_args:
            raise RuntimeError("unknown launch argument: {}".format(name))
        node = launch_args[name]
        if node.hasAttribute("value"):
            node.removeAttribute("value")
        node.setAttribute("default", value)

    return doc.toxml()


def parse_overrides(args):
    overrides = {}
    for a in args:
        if ":=" not in a:
            raise RuntimeError("arguments must use name:=value: {}".format(a))
        name, value = a.split(":=", 1)
        if not name:
            raise RuntimeError("empty argument name")
        overrides[name] = value
    return overrides


def main():
    overrides = parse_overrides(sys.argv[1:])
    xml_str = load_launch_xml(overrides)
    run_id = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(run_id)
    parent = roslaunch.parent.ROSLaunchParent(
        run_id, [], roslaunch_strs=[xml_str], force_screen=True)
    parent.start()
    try:
        parent.spin()
    finally:
        parent.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        sys.stderr.write("[ERROR] src3-nav launch failed: {}\n".format(exc))
        sys.exit(1)
