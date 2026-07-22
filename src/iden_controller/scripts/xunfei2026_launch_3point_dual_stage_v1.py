#!/usr/bin/python3
"""Launch the 3-point dual-stage room nav inside the subtask1 infrastructure."""

import os, sys
from xml.dom import minidom
import roslaunch, rospkg

OLD_MANAGER  = "xunfei2026_room_delivery_anchor_coverage_v1.py"
NEW_MANAGER  = "xunfei2026_3point_dual_stage_v1.py"
OLD_HANDOFF  = "xunfei2026_simulation_handoff_tts_trigger_v1.py"
NEW_HANDOFF  = "xunfei2026_simulation_handoff_second_parking_v3.py"
SRC  = "subtask1_xunfei2026_complete_delivery_anchor_coverage_v1.launch"


def _set(node, name, val):
    for c in node.childNodes:
        if c.nodeType == c.ELEMENT_NODE and c.tagName == "param" and c.getAttribute("name") == name:
            c.setAttribute("value", str(val))
            return
    el = node.ownerDocument.createElement("param")
    el.setAttribute("name", name)
    el.setAttribute("value", str(val))
    node.appendChild(el)


def load(ov):
    pkg = rospkg.RosPack().get_path("iden_controller")
    with open(os.path.join(pkg, "launch", SRC), encoding="utf-8") as f:
        doc = minidom.parseString(f.read())

    # ---- swap manager ----
    mgrs = [n for n in doc.getElementsByTagName("node")
            if n.getAttribute("type") == OLD_MANAGER]
    if len(mgrs) != 1:
        raise RuntimeError("manager count: {}".format(len(mgrs)))
    m = mgrs[0]
    m.setAttribute("type", NEW_MANAGER)

    # 3-point OCR: old factory_room_ocr_ros (not src2/sr3)
    _set(m, "ocr_launch_file", "")  # not used — OCR is launched inline below

    # ---- add old OCR node directly (same as original fixed_point launch) ----
    ocr_el = doc.createElement("node")
    ocr_el.setAttribute("pkg", "iden_controller")
    ocr_el.setAttribute("type", "run_factory_room_ocr_ros.sh")
    ocr_el.setAttribute("name", "factory_room_ocr")
    ocr_el.setAttribute("output", "screen")
    ocr_el.setAttribute("required", "true")
    for n, v in [
        ("image_topic", "/ucar_camera/image_raw"),
        ("result_topic", "/factory_room/ocr_result"),
        ("control_topic", "/factory_room/ocr_control"),
        ("process_rate_hz", "4.0"),
        ("vote_window", "10"),
        ("vote_need", "5"),
        ("enabled_on_start", "false"),
        ("publish_debug", "true"),
    ]:
        p = doc.createElement("param")
        p.setAttribute("name", n)
        p.setAttribute("value", v)
        ocr_el.appendChild(p)
    m.parentNode.insertBefore(ocr_el, m.nextSibling)  # insert OCR after manager
    _set(m, "ocr_ready_timeout_s", "25.0")
    _set(m, "vote_window", "10")
    _set(m, "vote_need", "5")
    _set(m, "continuous_scan_max_speed_rps", "0.14")
    _set(m, "continuous_scan_min_speed_rps", "0.045")
    _set(m, "continuous_scan_acceleration_rps2", "0.28")

    # Keep anchors and scan specs from the 3-point YAML
    _set(m, "route_points_yaml",
         "fixed_point_continuous_ocr_sweep_route_test_v1.yaml")

    # ---- swap handoff ----
    hs = [n for n in doc.getElementsByTagName("node")
          if n.getAttribute("name") == "xunfei2026_simulation_handoff"]
    if len(hs) != 1:
        raise RuntimeError("handoff count: {}".format(len(hs)))
    h = hs[0]
    if h.getAttribute("type") != OLD_HANDOFF:
        raise RuntimeError("handoff type: {}".format(h.getAttribute("type")))
    h.setAttribute("type", NEW_HANDOFF)
    _set(h, "post_sim_visual_handoff_position_m", "0.18")
    _set(h, "post_sim_visual_handoff_yaw_deg", "28.0")
    _set(h, "stop_line_launch_file",
         "xunfei2026_stop_line_parking_visual_handoff_v2.launch")

    # CLI overrides
    args = {n.getAttribute("name"): n
            for n in doc.getElementsByTagName("arg") if n.hasAttribute("name")}
    for k, v in ov.items():
        if k not in args:
            raise RuntimeError("unknown arg: {}".format(k))
        node = args[k]
        if node.hasAttribute("value"):
            node.removeAttribute("value")
        node.setAttribute("default", v)

    return doc.toxml()


def main():
    ov = {}
    for a in sys.argv[1:]:
        if ":=" not in a:
            raise RuntimeError("bad arg: {}".format(a))
        k, v = a.split(":=", 1)
        ov[k] = v
    xml = load(ov)
    rid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(rid)
    p = roslaunch.parent.ROSLaunchParent(rid, [], roslaunch_strs=[xml], force_screen=True)
    p.start()
    try:
        p.spin()
    finally:
        p.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        sys.stderr.write("[ERROR] 3point launch: {}\n".format(e))
        sys.exit(1)
