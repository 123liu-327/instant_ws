cmake_minimum_required(VERSION 3.0.2)
project(flow_end)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  include/flow_test
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 鏄庣‘鍒楀嚭婧愭枃浠讹紝閬垮厤 GLOB 涓嶇敓鏁?
set(SOURCES
  src/Laser_linear.cpp
  src/follow_line.cpp
  src/PID.cpp
  src/Callback.cpp
  src/corner_move.cpp
  src/Findline_Adaptive.cpp
  src/follow.cpp
  src/generateLookupTable.cpp
  src/ImagePerspectiveInit.cpp
  src/MatTransform.cpp
  src/Point_Process.cpp
  src/process_image.cpp
  src/Signal.cpp
  src/track_line.cpp

)

message(STATUS "Sources: ${SOURCES}")

add_executable(follow_end ${SOURCES})

target_link_libraries(follow_end
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES}
)

add_executable(image_process_debug
  src/image_process_debug.cpp
  src/Findline_Adaptive.cpp
  src/ImagePerspectiveInit.cpp
  src/MatTransform.cpp
  src/Point_Process.cpp
  src/process_image.cpp
  src/track_line.cpp
)

target_link_libraries(image_process_debug
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)





add_executable(follow_test
  src/follow_test.cpp
  src/Callback_test.cpp
  src/follow_line_test.cpp
  src/follow_motion_controller.cpp
  src/PID.cpp
  src/corner_move.cpp
  src/Findline_Adaptive.cpp
  src/ImagePerspectiveInit.cpp
  src/MatTransform.cpp
  src/Point_Process.cpp
  src/process_image.cpp
  src/track_line.cpp
)

target_link_libraries(follow_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
