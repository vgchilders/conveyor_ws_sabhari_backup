execute_process(COMMAND "/home/recycling-robot-mqp/conveyor_ws_sabhari_backup/build/arm/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/recycling-robot-mqp/conveyor_ws_sabhari_backup/build/arm/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
