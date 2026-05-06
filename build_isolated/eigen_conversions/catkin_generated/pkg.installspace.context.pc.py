# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include/eigen3".split(';') if "${prefix}/include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "geometry_msgs;std_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-leigen_conversions;-lorocos-kdl".split(';') if "-leigen_conversions;-lorocos-kdl" != "" else []
PROJECT_NAME = "eigen_conversions"
PROJECT_SPACE_DIR = "/home/ucar/ucar_ws/install_isolated"
PROJECT_VERSION = "1.12.0"
