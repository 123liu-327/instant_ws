#!/usr/bin/env bash
set -eo pipefail

WS="${INSTANT_WS:-/home/ucar/instant_ws}"
SPEECH_SOURCE="$WS/src/speech_command"
ASR_BINARY="$WS/src/ucar_2026_competition/scripts/cloud_asr_complete_sentence_v1"

if [[ ! -x "$ASR_BINARY" ]]; then
  echo "[ERROR] Missing complete-sentence ASR binary: $ASR_BINARY" >&2
  exit 1
fi
if [[ ! -f "$SPEECH_SOURCE/package.xml" ]]; then
  echo "[ERROR] Missing legacy speech package resources: $SPEECH_SOURCE" >&2
  exit 1
fi

export ROS_PACKAGE_PATH="$SPEECH_SOURCE${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export LD_LIBRARY_PATH="$WS/devel/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$ASR_BINARY" "$@"
