#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

find_package(Boost REQUIRED COMPONENTS program_options timer)

add_executable(mc_bin_to_log mc_bin_to_log.cpp mc_bin_to_log_main.cpp)
target_link_libraries(mc_bin_to_log PUBLIC mc_control)

add_executable(mc_bin_to_flat mc_bin_to_flat.cpp mc_bin_to_flat_main.cpp)
target_link_libraries(mc_bin_to_flat PUBLIC mc_control)

if(${ROSCPP_FOUND})
  add_executable(mc_bin_to_rosbag mc_bin_to_rosbag.cpp mc_bin_to_rosbag_main.cpp)
  target_link_libraries(mc_bin_to_rosbag PUBLIC mc_control)
  install(TARGETS mc_bin_to_rosbag DESTINATION bin)
endif()

set(mc_bin_utils_SRC
  mc_bin_utils.cpp
  mc_bin_to_log.cpp
  mc_bin_to_flat.cpp
)
if(${ROSCPP_FOUND})
  list(APPEND mc_bin_utils_SRC mc_bin_to_rosbag.cpp)
endif()
add_executable(mc_bin_utils ${mc_bin_utils_SRC})
target_link_libraries(mc_bin_utils PUBLIC mc_control Boost::program_options Boost::disable_autolinking)

install(TARGETS mc_bin_to_log mc_bin_to_flat mc_bin_utils DESTINATION bin)

set(UTILS_SCRIPT
  mc_rtc_new_controller
  mc_rtc_new_fsm_controller
)
install(PROGRAMS ${UTILS_SCRIPT} DESTINATION bin)

add_subdirectory(mc_log_gui)
