#!/usr/bin/env bash
set -euo pipefail

WS="${INSTANT_WS:-/home/ucar/instant_ws}"
SRC="$WS/src/ucar_2026_competition/scripts/cloud_asr_complete_sentence_v1.cpp"
OUT="$WS/src/ucar_2026_competition/scripts/cloud_asr_complete_sentence_v1"
STAGED_OUT="${OUT}.new.$$"
FILEUTIL_OBJ="$WS/build/speech_command/CMakeFiles/cloud_asr_test2.dir/src/FileUtil.cpp.o"
OBJ="$(mktemp /tmp/cloud_asr_complete_sentence_v1.XXXXXX.o)"
trap 'rm -f "$OBJ" "$STAGED_OUT"' EXIT

test -f "$SRC"
test -f "$FILEUTIL_OBJ"

/usr/bin/c++ \
  -DROSCONSOLE_BACKEND_LOG4CXX \
  -DROS_BUILD_SHARED_LIBS=1 \
  -DROS_PACKAGE_NAME=\"speech_command\" \
  -I"$WS/src/speech_command/include" \
  -I"$WS/src/speech_command/include/jsoncpp" \
  -I/opt/ros/noetic/include \
  -I/opt/ros/noetic/share/xmlrpcpp/cmake/../../../include/xmlrpcpp \
  -I/usr/lib/libusb/include/libusb-1.0 \
  -std=c++11 \
  -o "$OBJ" \
  -c "$SRC"

/usr/bin/c++ -rdynamic -pthread \
  "$OBJ" "$FILEUTIL_OBJ" \
  -o "$STAGED_OUT" \
  -L"$WS/src/speech_command/lib/arm64" \
  -L/usr/include \
  -Wl,-rpath,"$WS/src/speech_command/lib/arm64:/usr/include:/opt/ros/noetic/lib" \
  /opt/ros/noetic/lib/libroscpp.so \
  /opt/ros/noetic/lib/librosconsole.so \
  /opt/ros/noetic/lib/librosconsole_log4cxx.so \
  /opt/ros/noetic/lib/librosconsole_backend_interface.so \
  -llog4cxx -lboost_regex \
  /opt/ros/noetic/lib/libxmlrpcpp.so \
  /opt/ros/noetic/lib/libroscpp_serialization.so \
  /opt/ros/noetic/lib/librostime.so \
  /opt/ros/noetic/lib/libcpp_common.so \
  -lboost_thread -lpthread -lboost_chrono -lboost_date_time -lboost_atomic \
  /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4 \
  /opt/ros/noetic/lib/libroslib.so \
  /opt/ros/noetic/lib/librospack.so \
  -lpython3.7m -lboost_filesystem -lboost_program_options -lboost_system \
  -ltinyxml2 /opt/ros/noetic/lib/libserial.so \
  -laiui -lmsc -lasound -ljsoncpp -lrt -ldl -lpthread -lstdc++

chmod 755 "$STAGED_OUT"
if ldd "$STAGED_OUT" | grep -q 'not found'; then
  ldd "$STAGED_OUT"
  exit 1
fi
mv -f "$STAGED_OUT" "$OUT"

echo "ASR_COMPLETE_SENTENCE_BUILD_OK $OUT"
