#!/usr/bin/env bash
set -eo pipefail

WS="${INSTANT_WS:-/home/ucar/instant_ws}"
SRC3="$WS/src3"
BUILD="$SRC3/build_pure"
DEVEL="$SRC3/devel_pure"

# Prevent a caller's sourced legacy workspace from becoming a catkin underlay.
unset CMAKE_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH PKG_CONFIG_PATH
source /opt/ros/noetic/setup.bash

packages=(
  ucar_2026_competition
  ucar_2026_competition_speech
  ucar_2026_smart_factory_llm
  ucar_2026_strict_mission
  ucar_2026_track_end_stop
  ucar_2026_traffic_light_rknn_test
  ucar_2026_line_follow
  ucar_2026_qr_speak_test
  factory_sign_ppocr_rknn_test
  yolo
  simple_navigator
  vision_triggered_navigator
  map_goal_picker
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

catkin_make -C "$SRC3" \
  --source "$SRC3" \
  --build "$BUILD" \
  --force-cmake \
  --only-pkg-with-deps "${packages[@]}" \
  -DCATKIN_DEVEL_PREFIX="$DEVEL" \
  -DCMAKE_BUILD_TYPE=Release \
  -j2

source "$DEVEL/setup.bash"
case ":${CMAKE_PREFIX_PATH:-}:" in
  *":$WS/devel:"*)
    echo "[ERROR] Pure src3 build still contains legacy underlay: $CMAKE_PREFIX_PATH" >&2
    exit 1
    ;;
esac

required=(
  "$DEVEL/lib/move_base/move_base"
  "$DEVEL/lib/amcl/amcl"
  "$DEVEL/lib/ucar_controller/base_driver"
  "$DEVEL/lib/ydlidar/ydlidar_node"
  "$DEVEL/lib/ucar_2026_competition/competition_flow.py"
  "$DEVEL/lib/ucar_2026_smart_factory_llm/reason_pickup_server.py"
)
for path in "${required[@]}"; do
  if [[ ! -e "$path" ]]; then
    echo "[ERROR] Pure src3 artifact missing: $path" >&2
    exit 1
  fi
done

echo "[PASS] Pure src3 build completed without legacy workspace underlay."
echo "[PASS] CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
