#!/usr/bin/env bash
set -eo pipefail

WS="${INSTANT_WS:-/home/ucar/instant_ws}"
SRC="$WS/src"
PURE_WS="$WS/src_pure_runtime_ws_v3"
PURE_SRC="$PURE_WS/src"
BUILD="$WS/build_src_pure_runtime_v3"
DEVEL="$WS/devel_src_pure_runtime_v3"

unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
unset PKG_CONFIG_PATH ROSLISP_PACKAGE_DIRECTORIES
source /opt/ros/noetic/setup.bash

mkdir -p "$PURE_SRC"
ln -sfn /opt/ros/noetic/share/catkin/cmake/toplevel.cmake "$PURE_SRC/CMakeLists.txt"
# Remove the failed v3 draft link; the original flow_end source stays untouched.
rm -f "$PURE_SRC/flow_end"

link_package() {
  local path="$1"
  if [[ ! -f "$path/package.xml" ]]; then
    echo "[ERROR] Package source missing: $path" >&2
    exit 1
  fi
  ln -sfn "$path" "$PURE_SRC/$(basename "$path")"
}

link_package "$SRC/ucar_2026_competition"
link_package "$SRC/ucar_2026_competition_speech"
link_package "$SRC/ucar_2026_smart_factory_llm"
link_package "$SRC/ucar_2026_strict_mission"
link_package "$SRC/ucar_2026_track_end_stop"
link_package "$SRC/ucar_2026_traffic_light_rknn_test"
link_package "$SRC/ucar_2026_qr_speak_test"
link_package "$SRC/factory_sign_ppocr_rknn_test"
link_package "$SRC/flow_end_runtime_v1"
link_package "$SRC/traffic_light_vision"
link_package "$SRC/ucar_camera"
link_package "$SRC/yolo"
link_package "$SRC/ucar_map"
link_package "$SRC/ucar_controller"
link_package "$SRC/ydlidar"
link_package "$SRC/ucar_2026_nav/test_vision2nav/simple_navigator"
link_package "$SRC/ucar_2026_nav/test_vision2nav/vision_triggered_navigator"
link_package "$SRC/ucar_2026_nav/test_vision2nav/map_goal_picker"
link_package "$SRC/ucar_2026_nav/ucar_nav"
link_package "$SRC/ucar_2026_nav/my_planner"
link_package "$SRC/ucar_2026_nav/3rd/fdilink_ahrs"

for path in "$SRC/ucar_2026_nav/3rd/navigation"/*; do
  [[ -f "$path/package.xml" ]] && link_package "$path"
done
for path in "$SRC/ucar_2026_nav/3rd/geometry"/*; do
  [[ -f "$path/package.xml" ]] && link_package "$path"
done
for path in "$SRC/ucar_2026_nav/3rd/geometry2"/*; do
  [[ -f "$path/package.xml" ]] && link_package "$path"
done
link_package "$SRC/ucar_2026_nav/3rd/vision_opencv-noetic/cv_bridge"
link_package "$SRC/ucar_2026_nav/3rd/image_pipeline-noetic/image_view"

packages=(
  ucar_2026_competition
  ucar_2026_competition_speech
  ucar_2026_smart_factory_llm
  ucar_2026_strict_mission
  ucar_2026_track_end_stop
  ucar_2026_traffic_light_rknn_test
  ucar_2026_qr_speak_test
  factory_sign_ppocr_rknn_test
  flow_end_runtime_v1
  traffic_light_vision
  ucar_camera
  yolo
  simple_navigator
  vision_triggered_navigator
  map_goal_picker
  ucar_map
  ucar_nav
  ucar_controller
  ydlidar
  my_planner
  move_base
  amcl
  global_planner
  navfn
  base_local_planner
  dwa_local_planner
  clear_costmap_recovery
  rotate_recovery
  move_slow_and_clear
  costmap_2d
  nav_core
  map_server
  image_view
  cv_bridge
  tf
  tf2
  tf2_msgs
  tf2_geometry_msgs
  tf2_sensor_msgs
  voxel_grid
)

catkin_make -C "$PURE_WS" \
  --source "$PURE_SRC" \
  --build "$BUILD" \
  --force-cmake \
  --only-pkg-with-deps "${packages[@]}" \
  -DCATKIN_DEVEL_PREFIX="$DEVEL" \
  -DCMAKE_BUILD_TYPE=Release \
  -j2

required=(
  "$DEVEL/lib/move_base/move_base"
  "$DEVEL/lib/amcl/amcl"
  "$DEVEL/lib/ucar_controller/base_driver"
  "$DEVEL/lib/ydlidar/ydlidar_node"
  "$DEVEL/lib/ucar_2026_track_end_stop/track_end_stop_node"
  "$DEVEL/lib/ucar_2026_track_end_stop/right_track_end_stop_node"
  "$DEVEL/lib/ucar_2026_track_end_stop/stable_right_track_end_stop_node"
  "$DEVEL/lib/ucar_2026_strict_mission/strict_mission_node.py"
  "$DEVEL/lib/flow_end_runtime_v1/follow_test"
  "$DEVEL/lib/traffic_light_vision/traffic_light_detector.py"
  "$DEVEL/lib/ucar_camera/ucar_camera.py"
  "$DEVEL/lib/ucar_2026_competition/competition_flow_flowend_v1.py"
)
for path in "${required[@]}"; do
  if [[ ! -x "$path" ]]; then
    echo "[ERROR] Pure runtime artifact missing: $path" >&2
    exit 1
  fi
done

echo "[PASS] Pure unified-src runtime built at $DEVEL"
