file(GLOB robot_module_dirs "*")
foreach(robot_module_dir ${robot_module_dirs})
  if(EXISTS ${robot_module_dir}/CMakeLists.txt)
    add_subdirectory(${robot_module_dir})
  endif()
endforeach()

