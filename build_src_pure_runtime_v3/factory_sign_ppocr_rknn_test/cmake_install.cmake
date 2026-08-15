# Install script for directory: /home/ucar/instant_ws/src_pure_runtime_ws_v3/src/factory_sign_ppocr_rknn_test

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ucar/instant_ws/build_src_pure_runtime_v3/factory_sign_ppocr_rknn_test/catkin_generated/installspace/factory_sign_ppocr_rknn_test.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/factory_sign_ppocr_rknn_test/cmake" TYPE FILE FILES
    "/home/ucar/instant_ws/build_src_pure_runtime_v3/factory_sign_ppocr_rknn_test/catkin_generated/installspace/factory_sign_ppocr_rknn_testConfig.cmake"
    "/home/ucar/instant_ws/build_src_pure_runtime_v3/factory_sign_ppocr_rknn_test/catkin_generated/installspace/factory_sign_ppocr_rknn_testConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/factory_sign_ppocr_rknn_test" TYPE FILE FILES "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/factory_sign_ppocr_rknn_test/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/factory_sign_ppocr_rknn_test" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v3/factory_sign_ppocr_rknn_test/catkin_generated/installspace/factory_sign_ppocr_rknn_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/factory_sign_ppocr_rknn_test" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v3/factory_sign_ppocr_rknn_test/catkin_generated/installspace/check_ppocr_rknn_env.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/factory_sign_ppocr_rknn_test" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v3/factory_sign_ppocr_rknn_test/catkin_generated/installspace/test_image_ppocr_rknn.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/factory_sign_ppocr_rknn_test" TYPE DIRECTORY FILES
    "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/factory_sign_ppocr_rknn_test/launch"
    "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/factory_sign_ppocr_rknn_test/config"
    "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/factory_sign_ppocr_rknn_test/models"
    )
endif()

