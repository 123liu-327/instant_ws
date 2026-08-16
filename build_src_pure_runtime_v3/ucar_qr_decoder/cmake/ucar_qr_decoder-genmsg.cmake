# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ucar_qr_decoder: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iucar_qr_decoder:/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ucar_qr_decoder_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeRequest.msg" NAME_WE)
add_custom_target(_ucar_qr_decoder_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucar_qr_decoder" "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeRequest.msg" "std_msgs/Header:sensor_msgs/Image"
)

get_filename_component(_filename "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeResult.msg" NAME_WE)
add_custom_target(_ucar_qr_decoder_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ucar_qr_decoder" "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeResult.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ucar_qr_decoder
  "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucar_qr_decoder
)
_generate_msg_cpp(ucar_qr_decoder
  "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucar_qr_decoder
)

### Generating Services

### Generating Module File
_generate_module_cpp(ucar_qr_decoder
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucar_qr_decoder
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ucar_qr_decoder_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ucar_qr_decoder_generate_messages ucar_qr_decoder_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeRequest.msg" NAME_WE)
add_dependencies(ucar_qr_decoder_generate_messages_cpp _ucar_qr_decoder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeResult.msg" NAME_WE)
add_dependencies(ucar_qr_decoder_generate_messages_cpp _ucar_qr_decoder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_qr_decoder_gencpp)
add_dependencies(ucar_qr_decoder_gencpp ucar_qr_decoder_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_qr_decoder_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ucar_qr_decoder
  "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ucar_qr_decoder
)
_generate_msg_eus(ucar_qr_decoder
  "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ucar_qr_decoder
)

### Generating Services

### Generating Module File
_generate_module_eus(ucar_qr_decoder
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ucar_qr_decoder
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ucar_qr_decoder_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ucar_qr_decoder_generate_messages ucar_qr_decoder_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeRequest.msg" NAME_WE)
add_dependencies(ucar_qr_decoder_generate_messages_eus _ucar_qr_decoder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeResult.msg" NAME_WE)
add_dependencies(ucar_qr_decoder_generate_messages_eus _ucar_qr_decoder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_qr_decoder_geneus)
add_dependencies(ucar_qr_decoder_geneus ucar_qr_decoder_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_qr_decoder_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ucar_qr_decoder
  "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucar_qr_decoder
)
_generate_msg_lisp(ucar_qr_decoder
  "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucar_qr_decoder
)

### Generating Services

### Generating Module File
_generate_module_lisp(ucar_qr_decoder
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucar_qr_decoder
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ucar_qr_decoder_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ucar_qr_decoder_generate_messages ucar_qr_decoder_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeRequest.msg" NAME_WE)
add_dependencies(ucar_qr_decoder_generate_messages_lisp _ucar_qr_decoder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeResult.msg" NAME_WE)
add_dependencies(ucar_qr_decoder_generate_messages_lisp _ucar_qr_decoder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_qr_decoder_genlisp)
add_dependencies(ucar_qr_decoder_genlisp ucar_qr_decoder_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_qr_decoder_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ucar_qr_decoder
  "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ucar_qr_decoder
)
_generate_msg_nodejs(ucar_qr_decoder
  "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ucar_qr_decoder
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ucar_qr_decoder
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ucar_qr_decoder
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ucar_qr_decoder_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ucar_qr_decoder_generate_messages ucar_qr_decoder_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeRequest.msg" NAME_WE)
add_dependencies(ucar_qr_decoder_generate_messages_nodejs _ucar_qr_decoder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeResult.msg" NAME_WE)
add_dependencies(ucar_qr_decoder_generate_messages_nodejs _ucar_qr_decoder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_qr_decoder_gennodejs)
add_dependencies(ucar_qr_decoder_gennodejs ucar_qr_decoder_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_qr_decoder_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ucar_qr_decoder
  "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_qr_decoder
)
_generate_msg_py(ucar_qr_decoder
  "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_qr_decoder
)

### Generating Services

### Generating Module File
_generate_module_py(ucar_qr_decoder
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_qr_decoder
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ucar_qr_decoder_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ucar_qr_decoder_generate_messages ucar_qr_decoder_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeRequest.msg" NAME_WE)
add_dependencies(ucar_qr_decoder_generate_messages_py _ucar_qr_decoder_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ucar/instant_ws/src_pure_runtime_ws_v3/src/ucar_qr_decoder/msg/ZBarDecodeResult.msg" NAME_WE)
add_dependencies(ucar_qr_decoder_generate_messages_py _ucar_qr_decoder_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ucar_qr_decoder_genpy)
add_dependencies(ucar_qr_decoder_genpy ucar_qr_decoder_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ucar_qr_decoder_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucar_qr_decoder)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ucar_qr_decoder
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(ucar_qr_decoder_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ucar_qr_decoder_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ucar_qr_decoder)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ucar_qr_decoder
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(ucar_qr_decoder_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ucar_qr_decoder_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucar_qr_decoder)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ucar_qr_decoder
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(ucar_qr_decoder_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ucar_qr_decoder_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ucar_qr_decoder)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ucar_qr_decoder
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(ucar_qr_decoder_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ucar_qr_decoder_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_qr_decoder)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_qr_decoder\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_qr_decoder
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_qr_decoder")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ucar_qr_decoder
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(ucar_qr_decoder_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ucar_qr_decoder_generate_messages_py std_msgs_generate_messages_py)
endif()
