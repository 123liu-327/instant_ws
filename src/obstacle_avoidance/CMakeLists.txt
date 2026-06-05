cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoidance)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/collision_avoider.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
