# Install script for directory: /home/ucar/instant_ws/src_pure_runtime_ws/src/ucar_2026_competition_speech

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
  include("/home/ucar/instant_ws/build_src_pure_runtime_v2/ucar_2026_competition_speech/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_2026_competition_speech/srv" TYPE FILE FILES "/home/ucar/instant_ws/src_pure_runtime_ws/src/ucar_2026_competition_speech/srv/Announce.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_2026_competition_speech/cmake" TYPE FILE FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/ucar_2026_competition_speech/catkin_generated/installspace/ucar_2026_competition_speech-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ucar/instant_ws/devel_src_pure_runtime_v2/include/ucar_2026_competition_speech")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ucar/instant_ws/devel_src_pure_runtime_v2/share/roseus/ros/ucar_2026_competition_speech")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ucar/instant_ws/devel_src_pure_runtime_v2/share/common-lisp/ros/ucar_2026_competition_speech")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ucar/instant_ws/devel_src_pure_runtime_v2/share/gennodejs/ros/ucar_2026_competition_speech")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ucar/instant_ws/devel_src_pure_runtime_v2/lib/python3/dist-packages/ucar_2026_competition_speech")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ucar/instant_ws/devel_src_pure_runtime_v2/lib/python3/dist-packages/ucar_2026_competition_speech" REGEX "/\\_\\_init\\_\\_\\.py$" EXCLUDE REGEX "/\\_\\_init\\_\\_\\.pyc$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ucar/instant_ws/devel_src_pure_runtime_v2/lib/python3/dist-packages/ucar_2026_competition_speech" FILES_MATCHING REGEX "/home/ucar/instant_ws/devel_src_pure_runtime_v2/lib/python3/dist-packages/ucar_2026_competition_speech/.+/__init__.pyc?$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/ucar_2026_competition_speech/catkin_generated/installspace/ucar_2026_competition_speech.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_2026_competition_speech/cmake" TYPE FILE FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/ucar_2026_competition_speech/catkin_generated/installspace/ucar_2026_competition_speech-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_2026_competition_speech/cmake" TYPE FILE FILES
    "/home/ucar/instant_ws/build_src_pure_runtime_v2/ucar_2026_competition_speech/catkin_generated/installspace/ucar_2026_competition_speechConfig.cmake"
    "/home/ucar/instant_ws/build_src_pure_runtime_v2/ucar_2026_competition_speech/catkin_generated/installspace/ucar_2026_competition_speechConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_2026_competition_speech" TYPE FILE FILES "/home/ucar/instant_ws/src_pure_runtime_ws/src/ucar_2026_competition_speech/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ucar_2026_competition_speech" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/ucar_2026_competition_speech/catkin_generated/installspace/competition_announcer.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ucar_2026_competition_speech" TYPE PROGRAM FILES "/home/ucar/instant_ws/build_src_pure_runtime_v2/ucar_2026_competition_speech/catkin_generated/installspace/announce_once.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_2026_competition_speech" TYPE DIRECTORY FILES "/home/ucar/instant_ws/src_pure_runtime_ws/src/ucar_2026_competition_speech/launch")
endif()

