set(controller_SRC
  mc_lipm_stabilizer.cpp
)

set(controller_HDR
  mc_lipm_stabilizer.h
)

set(CONTROLLER_NAME lipm_stabilizer)
add_library(${CONTROLLER_NAME} SHARED ${controller_SRC} ${controller_HDR})
target_link_libraries(${CONTROLLER_NAME} PUBLIC mc_control_fsm)
install(TARGETS ${CONTROLLER_NAME} DESTINATION ${MC_CONTROLLER_INSTALL_PREFIX}/..)

add_controller(${CONTROLLER_NAME}_controller lib.cpp "")
set_target_properties(${CONTROLLER_NAME}_controller PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS" PREFIX "")
set_target_properties(${CONTROLLER_NAME}_controller PROPERTIES OUTPUT_NAME "${CONTROLLER_NAME}")
target_link_libraries(${CONTROLLER_NAME}_controller PUBLIC ${CONTROLLER_NAME})

