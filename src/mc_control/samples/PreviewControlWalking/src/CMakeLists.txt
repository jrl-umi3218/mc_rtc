set(controller_SRC
  PreviewControlWalking.cpp
)

set(controller_HDR
  PreviewControlWalking.h
)

set(CONTROLLER_NAME PreviewControlWalking)
add_library(${CONTROLLER_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${CONTROLLER_NAME} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS")
target_link_libraries(${CONTROLLER_NAME} PUBLIC mc_rtc::mc_control_fsm mc_rtc::mc_planning)
install(TARGETS ${CONTROLLER_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_controller(${CONTROLLER_NAME}Controller lib.cpp "")
set_target_properties(${CONTROLLER_NAME}Controller PROPERTIES FOLDER controllers/fsm)
target_link_libraries(${CONTROLLER_NAME}Controller PUBLIC ${CONTROLLER_NAME})

add_subdirectory(states)
