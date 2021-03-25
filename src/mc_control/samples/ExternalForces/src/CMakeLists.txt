set(controller_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_external_forces_controller.cpp
)

set(controller_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_external_forces_controller.h
)

add_fsm_controller(external_forces ${controller_SRC} ${controller_HDR})

add_subdirectory(states)
