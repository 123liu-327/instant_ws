# Install script for directory: /home/ucar/instant_ws/src3/ucar_2026_smart_factory_llm

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_2026_smart_factory_llm/srv" TYPE FILE FILES "/home/ucar/instant_ws/src3/ucar_2026_smart_factory_llm/srv/ReasonPickupOrder.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_2026_smart_factory_llm/cmake" TYPE FILE FILES "/home/ucar/instant_ws/src3/build_pure/ucar_2026_smart_factory_llm/catkin_generated/installspace/ucar_2026_smart_factory_llm-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ucar/instant_ws/src3/devel_pure/include/ucar_2026_smart_factory_llm")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ucar/instant_ws/src3/devel_pure/share/roseus/ros/ucar_2026_smart_factory_llm")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ucar/instant_ws/src3/devel_pure/share/common-lisp/ros/ucar_2026_smart_factory_llm")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ucar/instant_ws/src3/devel_pure/share/gennodejs/ros/ucar_2026_smart_factory_llm")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/ucar/instant_ws/src3/devel_pure/lib/python3/dist-packages/ucar_2026_smart_factory_llm")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/ucar/instant_ws/src3/devel_pure/lib/python3/dist-packages/ucar_2026_smart_factory_llm")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ucar/instant_ws/src3/build_pure/ucar_2026_smart_factory_llm/catkin_generated/installspace/ucar_2026_smart_factory_llm.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_2026_smart_factory_llm/cmake" TYPE FILE FILES "/home/ucar/instant_ws/src3/build_pure/ucar_2026_smart_factory_llm/catkin_generated/installspace/ucar_2026_smart_factory_llm-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_2026_smart_factory_llm/cmake" TYPE FILE FILES
    "/home/ucar/instant_ws/src3/build_pure/ucar_2026_smart_factory_llm/catkin_generated/installspace/ucar_2026_smart_factory_llmConfig.cmake"
    "/home/ucar/instant_ws/src3/build_pure/ucar_2026_smart_factory_llm/catkin_generated/installspace/ucar_2026_smart_factory_llmConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_2026_smart_factory_llm" TYPE FILE FILES "/home/ucar/instant_ws/src3/ucar_2026_smart_factory_llm/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ucar_2026_smart_factory_llm" TYPE PROGRAM FILES "/home/ucar/instant_ws/src3/build_pure/ucar_2026_smart_factory_llm/catkin_generated/installspace/reason_pickup_server.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ucar_2026_smart_factory_llm" TYPE PROGRAM FILES "/home/ucar/instant_ws/src3/build_pure/ucar_2026_smart_factory_llm/catkin_generated/installspace/reason_and_speak_once.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ucar_2026_smart_factory_llm" TYPE PROGRAM FILES "/home/ucar/instant_ws/src3/build_pure/ucar_2026_smart_factory_llm/catkin_generated/installspace/qr_to_llm_speak_once.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ucar_2026_smart_factory_llm" TYPE PROGRAM FILES "/home/ucar/instant_ws/src3/build_pure/ucar_2026_smart_factory_llm/catkin_generated/installspace/task1_full_once.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ucar_2026_smart_factory_llm" TYPE DIRECTORY FILES
    "/home/ucar/instant_ws/src3/ucar_2026_smart_factory_llm/launch"
    "/home/ucar/instant_ws/src3/ucar_2026_smart_factory_llm/config"
    )
endif()

