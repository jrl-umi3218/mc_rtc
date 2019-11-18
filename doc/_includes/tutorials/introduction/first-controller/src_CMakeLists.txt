set(controller_SRC
  MyFirstController.cpp
)

set(controller_HDR
  MyFirstController.h
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMyFirstController_EXPORTS")

