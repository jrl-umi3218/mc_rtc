set(controller_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_admittance_sample_controller.cpp
)

set(controller_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_admittance_sample_controller.h
)

add_fsm_controller(admittance_sample ${controller_SRC} ${controller_HDR})

add_subdirectory(states)
