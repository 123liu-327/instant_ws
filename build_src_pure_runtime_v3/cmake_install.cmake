# Install script for directory: /home/ucar/instant_ws/src_pure_runtime_ws_v3/src

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
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ucar/instant_ws/src_pure_runtime_ws_v3/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ucar/instant_ws/src_pure_runtime_ws_v3/install" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v3/catkin_generated/installspace/_setup_util.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ucar/instant_ws/src_pure_runtime_ws_v3/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ucar/instant_ws/src_pure_runtime_ws_v3/install" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v3/catkin_generated/installspace/env.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ucar/instant_ws/src_pure_runtime_ws_v3/install/setup.bash;/home/ucar/instant_ws/src_pure_runtime_ws_v3/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ucar/instant_ws/src_pure_runtime_ws_v3/install" TYPE FILE FILES
    "/home/ucar/instant_ws/build_src_pure_runtime_v3/catkin_generated/installspace/setup.bash"
    "/home/ucar/instant_ws/build_src_pure_runtime_v3/catkin_generated/installspace/local_setup.bash"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ucar/instant_ws/src_pure_runtime_ws_v3/install/setup.sh;/home/ucar/instant_ws/src_pure_runtime_ws_v3/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ucar/instant_ws/src_pure_runtime_ws_v3/install" TYPE FILE FILES
    "/home/ucar/instant_ws/build_src_pure_runtime_v3/catkin_generated/installspace/setup.sh"
    "/home/ucar/instant_ws/build_src_pure_runtime_v3/catkin_generated/installspace/local_setup.sh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ucar/instant_ws/src_pure_runtime_ws_v3/install/setup.zsh;/home/ucar/instant_ws/src_pure_runtime_ws_v3/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ucar/instant_ws/src_pure_runtime_ws_v3/install" TYPE FILE FILES
    "/home/ucar/instant_ws/build_src_pure_runtime_v3/catkin_generated/installspace/setup.zsh"
    "/home/ucar/instant_ws/build_src_pure_runtime_v3/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ucar/instant_ws/src_pure_runtime_ws_v3/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/ucar/instant_ws/src_pure_runtime_ws_v3/install" TYPE FILE FILES "/home/ucar/instant_ws/build_src_pure_runtime_v3/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/gtest/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/tf2_msgs/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/tf2/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/map_server/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/tf2_py/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/cv_bridge/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/factory_sign_ppocr_rknn_test/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/image_view/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/tf/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/flow_end_runtime_v1/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/map_goal_picker/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/simple_navigator/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/tf2_geometry_msgs/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/amcl/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/tf2_sensor_msgs/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/traffic_light_vision/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/ucar_2026_competition_speech/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/ucar_2026_qr_speak_test/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/ucar_2026_smart_factory_llm/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/ucar_2026_competition/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/ucar_2026_strict_mission/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/ucar_2026_track_end_stop/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/ucar_2026_traffic_light_rknn_test/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/ucar_camera/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/ucar_controller/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/ucar_map/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/ucar_nav/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/vision_triggered_navigator/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/voxel_grid/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/costmap_2d/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/nav_core/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/base_local_planner/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/clear_costmap_recovery/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/dwa_local_planner/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/move_slow_and_clear/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/my_planner/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/navfn/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/global_planner/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/rotate_recovery/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/move_base/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/ydlidar/cmake_install.cmake")
  include("/home/ucar/instant_ws/build_src_pure_runtime_v3/yolo/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/ucar/instant_ws/build_src_pure_runtime_v3/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
