set(controller_SRC
  mc_fsm_controller.cpp
)

set(controller_HDR
  mc_fsm_controller.h
)

set(CONTROLLER_NAME fsm_sample)
add_library(${CONTROLLER_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${CONTROLLER_NAME} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS")
target_link_libraries(${CONTROLLER_NAME} PUBLIC mc_control_fsm)
install(TARGETS ${CONTROLLER_NAME} DESTINATION ${MC_CONTROLLER_INSTALL_PREFIX}/..)

add_controller(${CONTROLLER_NAME}_controller lib.cpp "")
set_target_properties(${CONTROLLER_NAME}_controller PROPERTIES FOLDER controllers/fsm)
target_link_libraries(${CONTROLLER_NAME}_controller PUBLIC mc_control_fsm)

target_link_libraries(${CONTROLLER_NAME}_controller PUBLIC ${CONTROLLER_NAME})
