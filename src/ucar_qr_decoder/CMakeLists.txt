cmake_minimum_required(VERSION 3.0.2)
project(ucar_qr_decoder)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)

catkin_python_setup()

add_message_files(FILES
  ZBarDecodeRequest.msg
  ZBarDecodeResult.msg
)
generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS cv_bridge message_runtime nav_msgs roscpp rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
)

add_executable(zbar_decoder_node src/zbar_decoder_node.cpp)
add_dependencies(zbar_decoder_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(zbar_decoder_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${ZBAR_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/qr_decoder_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS zbar_decoder_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_decoder_logic.py)
endif()
