# ----------------------------------------------------------------
# Generated CMake target import file.
# ----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mc_rtc::mc_rtc_ros" for configuration ""
set_property(
  TARGET mc_rtc::mc_rtc_ros
  APPEND
  PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG
)
set_target_properties(
  mc_rtc::mc_rtc_ros
  PROPERTIES IMPORTED_LOCATION_NOCONFIG
             "/home/celian/workspace/install/lib/libmc_rtc_ros.so.1.0.0"
             IMPORTED_SONAME_NOCONFIG "libmc_rtc_ros.so.1"
)

list(APPEND _cmake_import_check_targets mc_rtc::mc_rtc_ros)
list(APPEND _cmake_import_check_files_for_mc_rtc::mc_rtc_ros
     "/home/celian/workspace/install/lib/libmc_rtc_ros.so.1.0.0"
)

# Import target "mc_rtc::mc_tasks_ros" for configuration ""
set_property(
  TARGET mc_rtc::mc_tasks_ros
  APPEND
  PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG
)
set_target_properties(
  mc_rtc::mc_tasks_ros
  PROPERTIES IMPORTED_LOCATION_NOCONFIG
             "/home/celian/workspace/install/lib/libmc_tasks_ros.so.1.0.0"
             IMPORTED_SONAME_NOCONFIG "libmc_tasks_ros.so.1"
)

list(APPEND _cmake_import_check_targets mc_rtc::mc_tasks_ros)
list(APPEND _cmake_import_check_files_for_mc_rtc::mc_tasks_ros
     "/home/celian/workspace/install/lib/libmc_tasks_ros.so.1.0.0"
)

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
