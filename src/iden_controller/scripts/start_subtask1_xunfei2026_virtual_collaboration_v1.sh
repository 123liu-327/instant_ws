#!/usr/bin/env bash
set -euo pipefail

# The complete-delivery script is the single owner of the real-car workflow.
# Its launch contains only the explicitly retained voice front-end and the
# passive post-parking simulation handoff.
WS="${INSTANT_WS:-/home/ucar/instant_ws}"
ORIGINAL="$WS/src/iden_controller/scripts/start_subtask1_xunfei2026_complete_delivery_v1.sh"

if [[ ! -x "$ORIGINAL" ]]; then
  echo "[ERROR] Original complete-delivery entry is missing: $ORIGINAL" >&2
  exit 1
fi

echo "[INFO] Starting the original complete-delivery workflow; simulation is post-parking only."
exec "$ORIGINAL" "$@"
