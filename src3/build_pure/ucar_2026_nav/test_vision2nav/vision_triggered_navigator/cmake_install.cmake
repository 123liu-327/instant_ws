# Install script for directory: /home/ucar/instant_ws/src3/ucar_2026_nav/test_vision2nav/vision_triggered_navigator

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ucar/instant_ws/src3/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ucar/instant_ws/src3/build_pure/ucar_2026_nav/test_vision2nav/vision_triggered_navigator/catkin_generated/installspace/vision_triggered_navigator.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vision_triggered_navigator/cmake" TYPE FILE FILES
    "/home/ucar/instant_ws/src3/build_pure/ucar_2026_nav/test_vision2nav/vision_triggered_navigator/catkin_generated/installspace/vision_triggered_navigatorConfig.cmake"
    "/home/ucar/instant_ws/src3/build_pure/ucar_2026_nav/test_vision2nav/vision_triggered_navigator/catkin_generated/installspace/vision_triggered_navigatorConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vision_triggered_navigator" TYPE FILE FILES "/home/ucar/instant_ws/src3/ucar_2026_nav/test_vision2nav/vision_triggered_navigator/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vision_triggered_navigator" TYPE PROGRAM FILES "/home/ucar/instant_ws/src3/build_pure/ucar_2026_nav/test_vision2nav/vision_triggered_navigator/catkin_generated/installspace/vision_triggered_navigator.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/vision_triggered_navigator" TYPE FILE FILES "/home/ucar/instant_ws/src3/ucar_2026_nav/test_vision2nav/vision_triggered_navigator/scripts/navigator_logic.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/vision_triggered_navigator" TYPE DIRECTORY FILES
    "/home/ucar/instant_ws/src3/ucar_2026_nav/test_vision2nav/vision_triggered_navigator/launch"
    "/home/ucar/instant_ws/src3/ucar_2026_nav/test_vision2nav/vision_triggered_navigator/config"
    )
endif()

