#!/usr/bin/env bash
set -eo pipefail

WS="${INSTANT_WS:-/home/ucar/instant_ws}"
SRC="$WS/src"
PURE_DEVEL="$WS/devel_src_pure_runtime_v2"

unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
unset PKG_CONFIG_PATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.bash
source "$PURE_DEVEL/setup.bash"

check_node() {
  local package="$1"
  local executable="$2"
  local resolved
  resolved="$(/usr/bin/python3 -c \
    'import sys; from roslib.packages import find_node; nodes = find_node(sys.argv[1], sys.argv[2]); print(nodes[0] if nodes else "")' \
    "$package" "$executable" 2>/dev/null)"
  if [[ -z "$resolved" || ! -x "$resolved" ]]; then
    echo "[ERROR] Missing executable: $package/$executable" >&2
    exit 1
  fi
  echo "[OK] node $package/$executable -> $resolved"
}

check_launch() {
  local package="$1"
  local launch_file="$2"
  shift 2
  roslaunch --nodes "$package" "$launch_file" "$@" >/dev/null
  echo "[OK] launch $package/$launch_file"
}

check_node move_base move_base
check_node ucar_2026_competition competition_flow_src6_hybrid_v1.py
check_node ucar_2026_competition src3_complete_sentence_cloud_asr_exec_v1.sh
check_node ucar_2026_competition src3_legacy_voice_bridge_complete_sentence_v1.py
check_node ucar_2026_smart_factory_llm reason_pickup_existing_spark_v1.py
check_node yolo qr_collect_and_decode_src6_hybrid_v1.py
check_node factory_sign_ppocr_rknn_test factory_sign_ppocr_rknn_src6_hybrid_v1.py
check_node vision_triggered_navigator vision_triggered_navigator_src6_points_v1.py
check_node ucar_2026_track_end_stop track_end_stop_node
check_node ucar_2026_track_end_stop right_track_end_stop_node
check_node ucar_2026_track_end_stop stable_right_track_end_stop_node
check_node ucar_2026_strict_mission strict_mission_node.py

check_launch ucar_2026_competition full_competition_legacy_src6_hybrid_v1.launch enable_simulation:=false
check_launch simple_navigator navigate.launch
check_launch ucar_2026_competition qr_decoder_src6_hybrid_v1.launch
check_launch factory_sign_ppocr_rknn_test factory_sign_ppocr_rknn_src6_hybrid_v1.launch
check_launch vision_triggered_navigator vision_triggered_navigator_src6_points_v1.launch
check_launch ucar_2026_strict_mission strict_mission_src6_hybrid_v1.launch start_traffic_detector:=false
check_launch ucar_2026_traffic_light_rknn_test traffic_light_rknn_x11_speak_test.launch start_camera:=false start_tts:=false start_competition_speech:=false start_viewer:=false
check_launch ucar_2026_track_end_stop track_end_stop.launch
check_launch ucar_2026_track_end_stop right_track_end_stop.launch
check_launch ucar_2026_track_end_stop stable_right_track_end_stop.launch

/usr/bin/python3 -m py_compile \
  "$SRC/ucar_2026_competition/scripts/competition_flow_src6_hybrid_v1.py" \
  "$SRC/ucar_2026_competition/scripts/src3_legacy_voice_bridge_complete_sentence_v1.py" \
  "$SRC/ucar_2026_strict_mission/scripts/strict_mission_node.py"
/home/ucar/miniconda3/bin/python3 -m py_compile \
  "$SRC/ucar_2026_smart_factory_llm/scripts/reason_pickup_existing_spark_v1.py" \
  "$SRC/factory_sign_ppocr_rknn_test/scripts/factory_sign_ppocr_rknn_src6_hybrid_v1.py"

echo "[OK] Unified src full-flow validation passed."
