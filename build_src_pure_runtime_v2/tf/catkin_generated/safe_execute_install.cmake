execute_process(COMMAND "/home/ucar/instant_ws/build_src_pure_runtime_v2/tf/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ucar/instant_ws/build_src_pure_runtime_v2/tf/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
