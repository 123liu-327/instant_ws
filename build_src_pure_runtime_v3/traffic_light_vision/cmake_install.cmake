# Install script for directory: /home/ucar/instant_ws/src_pure_runtime_ws_v3/src/traffic_light_vision

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ucar/instant_ws/src_pure_runtime_ws_v3/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ucar/instant_ws/build_src_pure_runtime_v3/traffic_light_vision/catkin_generated/installspace/traffic_light_vision.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/traffic_light_vision/cmake" TYPE FILE FILES
    "/home/ucar/instant_ws/build_src_pure_runtime_v3/traffic_light_vision/catkin_generated/installspace/traffic_light_visionConfig.cmake"
    "/home/ucar/instant_ws/build_src_pure_runtime_v3/traffic_light_vision/catkin_generated/installspace/traffic_light_visionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/traffic_light_vision" TYPE FILE FILES "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/traffic_light_vision/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/traffic_light_vision" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v3/traffic_light_vision/catkin_generated/installspace/traffic_light_detector.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/traffic_light_vision" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v3/traffic_light_vision/catkin_generated/installspace/rknn_backend.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/traffic_light_vision" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v3/traffic_light_vision/catkin_generated/installspace/audit_yolo_dataset.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/traffic_light_vision" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v3/traffic_light_vision/catkin_generated/installspace/convert_onnx_to_rknn.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/traffic_light_vision/launch" TYPE DIRECTORY FILES "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/traffic_light_vision/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/traffic_light_vision/models" TYPE DIRECTORY FILES "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/traffic_light_vision/models/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/traffic_light_vision/training" TYPE DIRECTORY FILES "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/traffic_light_vision/training/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/traffic_light_vision" TYPE FILE FILES "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/traffic_light_vision/YOLO_REBUILD_PLAN.md")
endif()

