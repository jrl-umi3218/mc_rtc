file(GLOB observer_dirs "*")
foreach(observer_dir ${observer_dirs})
  if(EXISTS ${observer_dir}/CMakeLists.txt)
    add_subdirectory(${observer_dir})
  endif()
endforeach()

