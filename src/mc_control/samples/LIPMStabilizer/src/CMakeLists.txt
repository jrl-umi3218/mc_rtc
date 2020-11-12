set(controller_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_lipm_stabilizer.cpp
)

set(controller_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_lipm_stabilizer.h
)

add_fsm_controller(lipm_stabilizer ${controller_SRC} ${controller_HDR})
