file(GLOB controller_dirs "*")
foreach(controller_dir ${controller_dirs})
  if(EXISTS ${controller_dir}/CMakeLists.txt)
    add_subdirectory(${controller_dir})
  endif()
endforeach()

