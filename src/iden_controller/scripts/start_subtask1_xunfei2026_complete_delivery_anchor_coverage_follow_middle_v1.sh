#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/start_subtask1_xunfei2026_complete_delivery_anchor_coverage_v1.sh" \
  "$@" \
  follow_after_stop_line:=true \
  follow_mode:=middle \
  advance_before_follow_distance_m:=0.25 \
  follow_initial_turn_enabled:=true \
  follow_initial_turn_angle_deg:=60.0 \
  follow_initial_turn_pause_s:=0.5
