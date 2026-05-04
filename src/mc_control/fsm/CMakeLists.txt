#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(fsm_controller_SRC Controller.cpp Executor.cpp State.cpp StateFactory.cpp
                       TransitionMap.cpp
)

set(fsm_controller_HDR_DIR ${PROJECT_SOURCE_DIR}/include/mc_control/fsm)
set(fsm_controller_HDR
    ${fsm_controller_HDR_DIR}/Controller.h ${fsm_controller_HDR_DIR}/Executor.h
    ${fsm_controller_HDR_DIR}/State.h ${fsm_controller_HDR_DIR}/StateFactory.h
    ${fsm_controller_HDR_DIR}/Transition.h ${fsm_controller_HDR_DIR}/TransitionMap.h
)

if(${PYTHON_BINDING})
  list(APPEND fsm_controller_HDR ${fsm_controller_HDR_DIR}/PythonState.h)
  list(APPEND fsm_controller_SRC PythonState.cpp)
endif()

add_library(mc_control_fsm SHARED ${fsm_controller_SRC} ${fsm_controller_HDR})
jrl_target_generate_config_header(
  mc_control_fsm PUBLIC VERSION "${PROJECT_VERSION}" FILENAME "mc_control/fsm/api.h"
)
set_target_properties(
  mc_control_fsm PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_FSM_EXPORTS"
)
target_link_libraries(mc_control_fsm PUBLIC mc_control mc_planning)
install_mc_rtc_lib(mc_control_fsm)

add_subdirectory(states)
