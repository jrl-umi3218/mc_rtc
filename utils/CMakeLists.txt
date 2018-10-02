add_executable(mc_bin_to_log mc_bin_to_log.cpp)
target_link_libraries(mc_bin_to_log PUBLIC mc_control)
pkg_config_use_dependency(mc_bin_to_log Tasks)

add_executable(mc_bin_to_flat mc_bin_to_flat.cpp)
target_link_libraries(mc_bin_to_flat PUBLIC mc_control)

set(UTILS_SCRIPT
  mc_rtc_new_controller
  mc_rtc_new_fsm_controller
)

install(TARGETS mc_bin_to_log mc_bin_to_flat DESTINATION bin)
install(PROGRAMS ${UTILS_SCRIPT} DESTINATION bin)

if(${ROSCPP_FOUND})
  add_executable(mc_bin_to_rosbag mc_bin_to_rosbag.cpp)
  target_link_libraries(mc_bin_to_rosbag PUBLIC mc_control)
  pkg_config_use_dependency(mc_bin_to_rosbag rosbag)
  pkg_config_use_dependency(mc_bin_to_rosbag sensor_msgs)
  install(TARGETS mc_bin_to_rosbag DESTINATION bin)
endif()
