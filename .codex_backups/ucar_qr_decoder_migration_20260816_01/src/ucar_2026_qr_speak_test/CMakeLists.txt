cmake_minimum_required(VERSION 3.0.2)
project(ucar_2026_qr_speak_test)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/audio_setup_node.py
  scripts/qr_rotate_scan_speak_test_node.py
  scripts/qr_speak_test_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_qr_rotate_scan_speak_logic.py)
endif()
