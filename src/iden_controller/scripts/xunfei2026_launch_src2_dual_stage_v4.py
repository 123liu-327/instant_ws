#!/usr/bin/python3
"""Launch the existing full flow with only the room manager substituted."""

import os
import sys
from xml.dom import minidom

import roslaunch
import rospkg


OLD_MANAGER = "xunfei2026_room_delivery_anchor_coverage_v1.py"
NEW_MANAGER = "xunfei2026_room_delivery_src2_dual_stage_v4.py"
OLD_HANDOFF = "xunfei2026_simulation_handoff_tts_trigger_v1.py"
NEW_HANDOFF = "xunfei2026_simulation_handoff_second_parking_v3.py"
SOURCE_LAUNCH = "subtask1_xunfei2026_complete_delivery_anchor_coverage_v1.launch"


def set_node_param(node, name, value):
    matches = [
        child for child in node.childNodes
        if (child.nodeType == child.ELEMENT_NODE and
            child.tagName == "param" and child.getAttribute("name") == name)
    ]
    if len(matches) != 1:
        raise RuntimeError("expected one private param {}, found {}".format(
            name, len(matches)))
    matches[0].setAttribute("value", str(value))


def load_launch_xml(overrides):
    package_dir = rospkg.RosPack().get_path("iden_controller")
    path = os.path.join(package_dir, "launch", SOURCE_LAUNCH)
    with open(path, "r", encoding="utf-8") as stream:
        document = minidom.parseString(stream.read())

    managers = [
        node for node in document.getElementsByTagName("node")
        if node.getAttribute("type") == OLD_MANAGER
    ]
    if len(managers) != 1:
        raise RuntimeError("expected one room manager, found {}".format(
            len(managers)))
    managers[0].setAttribute("type", NEW_MANAGER)

    handoffs = [
        node for node in document.getElementsByTagName("node")
        if node.getAttribute("name") == "xunfei2026_simulation_handoff"
    ]
    if len(handoffs) != 1:
        raise RuntimeError("expected one simulation handoff, found {}".format(
            len(handoffs)))
    handoff = handoffs[0]
    if handoff.getAttribute("type") != OLD_HANDOFF:
        raise RuntimeError("unexpected handoff type {}".format(
            handoff.getAttribute("type")))
    handoff.setAttribute("type", NEW_HANDOFF)
    set_node_param(handoff, "post_sim_visual_handoff_position_m", "0.18")
    set_node_param(handoff, "post_sim_visual_handoff_yaw_deg", "28.0")
    set_node_param(
        handoff, "stop_line_launch_file",
        "xunfei2026_stop_line_parking_visual_handoff_v2.launch")

    launch_args = {
        node.getAttribute("name"): node
        for node in document.getElementsByTagName("arg")
        if node.hasAttribute("name")
    }
    for name, value in overrides.items():
        if name not in launch_args:
            raise RuntimeError("unknown launch argument: {}".format(name))
        node = launch_args[name]
        if node.hasAttribute("value"):
            node.removeAttribute("value")
        node.setAttribute("default", value)
    return document.toxml()


def parse_overrides(arguments):
    values = {}
    for argument in arguments:
        if ":=" not in argument:
            raise RuntimeError("launch arguments must use name:=value: {}".format(
                argument))
        name, value = argument.split(":=", 1)
        if not name:
            raise RuntimeError("empty launch argument name")
        values[name] = value
    return values


def main():
    launch_xml = load_launch_xml(parse_overrides(sys.argv[1:]))
    run_id = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(run_id)
    parent = roslaunch.parent.ROSLaunchParent(
        run_id, [], roslaunch_strs=[launch_xml], force_screen=True)
    parent.start()
    try:
        parent.spin()
    finally:
        parent.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        sys.stderr.write("[ERROR] src2 dual-stage v4 launch failed: {}\n".format(exc))
        sys.exit(1)
