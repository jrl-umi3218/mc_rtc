set(controller_SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_fsm_controller.cpp
)

set(controller_HDR
  ${CMAKE_CURRENT_SOURCE_DIR}/mc_fsm_controller.h
)

add_fsm_controller(fsm_sample ${controller_SRC} ${controller_HDR})
