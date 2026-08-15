#!/usr/bin/env bash
set -eo pipefail

WS="${INSTANT_WS:-/home/ucar/instant_ws}"
SPEECH_SOURCE="$WS/src/speech_command"
SPEECH_BINARY="$WS/devel/lib/speech_command/cloud_asr_test2"

if [[ ! -x "$SPEECH_BINARY" ]]; then
  echo "[ERROR] Missing legacy ASR binary: $SPEECH_BINARY" >&2
  exit 1
fi
if [[ ! -f "$SPEECH_SOURCE/package.xml" ]]; then
  echo "[ERROR] Missing legacy speech package resources: $SPEECH_SOURCE" >&2
  exit 1
fi

# Scope legacy paths to this process only. The parent src3 launch environment
# remains pure and cannot resolve old navigation/localization packages.
export ROS_PACKAGE_PATH="$SPEECH_SOURCE${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export LD_LIBRARY_PATH="$WS/devel/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$SPEECH_BINARY" "$@"
