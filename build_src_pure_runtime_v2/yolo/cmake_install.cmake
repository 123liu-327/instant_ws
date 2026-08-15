# Install script for directory: /home/ucar/instant_ws/src_pure_runtime_ws/src/yolo

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ucar/instant_ws/src_pure_runtime_ws/install")
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
  include("/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/yolo.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yolo/cmake" TYPE FILE FILES
    "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/yoloConfig.cmake"
    "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/yoloConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yolo" TYPE FILE FILES "/home/ucar/instant_ws/src_pure_runtime_ws/src/yolo/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/camera_mjpeg_server.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/coco2yolo.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/collect_factory_sign.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/keyboard_collect_yolo_images.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/qr_collect_and_decode.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/validate_model.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/init_traffic_dataset.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/build_traffic_yolo_dataset.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/prepare_traffic_cls_dataset.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/train_traffic_resnet18.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/prepare_traffic_rknn_calibration.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/convert_traffic_resnet18_to_rknn.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/compare_traffic_onnx_rknn.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/export_traffic_onnx_reference.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/compare_traffic_rknnlite_reference.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/run_traffic_rknn_gate_on_car.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/deploy_traffic_resnet18_to_car.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/run_traffic_ros_smoke_on_car.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/yolo" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/yolo/catkin_generated/installspace/check_factory_sign_rknn_test.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yolo" TYPE DIRECTORY FILES
    "/home/ucar/instant_ws/src_pure_runtime_ws/src/yolo/launch"
    "/home/ucar/instant_ws/src_pure_runtime_ws/src/yolo/config"
    "/home/ucar/instant_ws/src_pure_runtime_ws/src/yolo/models"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/yolo" TYPE FILE FILES
    "/home/ucar/instant_ws/src_pure_runtime_ws/src/yolo/traffic_yolov5_dataset.md"
    "/home/ucar/instant_ws/src_pure_runtime_ws/src/yolo/traffic_classifier_training.md"
    )
endif()

