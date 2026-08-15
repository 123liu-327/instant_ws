#!/usr/bin/env bash
set -eo pipefail

WS="${INSTANT_WS:-/home/ucar/instant_ws}"
RUNTIME_WS="$WS/runtime_src6_ws"

source /opt/ros/noetic/setup.bash
set -u

catkin_make -C "$RUNTIME_WS" \
  --build "$WS/build_src6_runtime_v2" \
  -DCMAKE_BUILD_TYPE=Release
