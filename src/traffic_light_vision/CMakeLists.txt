cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_vision)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/traffic_light_detector.py
  scripts/rknn_backend.py
  tools/audit_yolo_dataset.py
  tools/convert_onnx_to_rknn.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(DIRECTORY training/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/training
)

install(FILES YOLO_REBUILD_PLAN.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_rknn_backend.py)
endif()
