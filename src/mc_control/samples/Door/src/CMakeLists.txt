set(controller_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_door_sample_controller.cpp
)

set(controller_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_door_sample_controller.h
)

add_fsm_controller(mc_door_sample ${controller_SRC} ${controller_HDR})

add_subdirectory(states)
