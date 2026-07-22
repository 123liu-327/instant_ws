#!/usr/bin/python3
"""Launch the full mission with the proven legacy OCR and parking core."""

import os
import sys
from xml.dom import minidom

import roslaunch
import rospkg

import xunfei2026_launch_src2_dual_stage_v4 as launch_v4


# Keep the complete two-stage delivery and simulation handoff, but restore the
# pre-src2 room OCR, anchor navigation and parking implementation.
launch_v4.NEW_MANAGER = "xunfei2026_room_delivery_dual_stage_v3.py"
launch_v4.NEW_HANDOFF = "xunfei2026_simulation_handoff_second_parking_v3.py"


def add_or_set_node_param(node, name, value):
    matches = [
        child for child in node.childNodes
        if (child.nodeType == child.ELEMENT_NODE and
            child.tagName == "param" and child.getAttribute("name") == name)
    ]
    if len(matches) > 1:
        raise RuntimeError("duplicate private param {}".format(name))
    if matches:
        matches[0].setAttribute("value", str(value))
        return
    parameter = node.ownerDocument.createElement("param")
    parameter.setAttribute("name", name)
    parameter.setAttribute("value", str(value))
    node.appendChild(parameter)


def load_launch_xml(overrides):
    """Apply the current handoff plus legacy room OCR and parking."""
    package_dir = rospkg.RosPack().get_path("iden_controller")
    path = os.path.join(package_dir, "launch", launch_v4.SOURCE_LAUNCH)
    with open(path, "r", encoding="utf-8") as stream:
        document = minidom.parseString(stream.read())

    managers = [
        node for node in document.getElementsByTagName("node")
        if node.getAttribute("type") == launch_v4.OLD_MANAGER
    ]
    if len(managers) != 1:
        raise RuntimeError("expected one room manager, found {}".format(
            len(managers)))
    manager = managers[0]
    manager.setAttribute("type", launch_v4.NEW_MANAGER)
    add_or_set_node_param(
        manager, "ocr_launch_file", "xunfei2026_factory_ocr_v1.launch")
    add_or_set_node_param(manager, "ocr_ready_timeout_s", "30.0")
    add_or_set_node_param(manager, "wait_after_tts_s", "0.35")
    add_or_set_node_param(manager, "dual_stage_real_tts_wait_s", "1.5")
    add_or_set_node_param(manager, "dual_stage_same_workshop_confirm_s", "0.5")
    add_or_set_node_param(manager, "dual_stage_sim_viewpoint_reconfirm_s", "1.4")
    handoffs = [
        node for node in document.getElementsByTagName("node")
        if node.getAttribute("name") == "xunfei2026_simulation_handoff"
    ]
    if len(handoffs) != 1:
        raise RuntimeError("expected one simulation handoff, found {}".format(
            len(handoffs)))
    handoff = handoffs[0]
    if handoff.getAttribute("type") != launch_v4.OLD_HANDOFF:
        raise RuntimeError("unexpected handoff type {}".format(
            handoff.getAttribute("type")))
    handoff.setAttribute("type", launch_v4.NEW_HANDOFF)
    launch_v4.set_node_param(
        handoff, "post_sim_visual_handoff_position_m", "0.18")
    launch_v4.set_node_param(
        handoff, "post_sim_visual_handoff_yaw_deg", "28.0")
    launch_v4.set_node_param(
        handoff, "stop_line_launch_file",
        "xunfei2026_stop_line_parking_visual_handoff_v2.launch")
    launch_v4.set_node_param(handoff, "sim_trigger_repeats", "5")
    launch_v4.set_node_param(handoff, "sim_trigger_interval_s", "0.08")

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


def main():
    launch_xml = load_launch_xml(launch_v4.parse_overrides(sys.argv[1:]))
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
        sys.stderr.write("[ERROR] legacy full launch failed: {}\n".format(exc))
        sys.exit(1)
