#pragma once

#include <string>

#include "@CMAKE_CURRENT_SOURCE_DIR@/utils.h"

#include "@CMAKE_CURRENT_SOURCE_DIR@/fsm_states/ConfigureState.h"

const std::string FSM_STATES_JSON_DIR = "@CMAKE_CURRENT_SOURCE_DIR@/fsm_states/data/";

static const std::string SingleState_DIR = "$<TARGET_FILE_DIR:SingleState>";
static const std::string MultipleStates_DIR = "$<TARGET_FILE_DIR:MultipleStates>";
static const std::string ConfigureState_DIR = "$<TARGET_FILE_DIR:ConfigureState>";