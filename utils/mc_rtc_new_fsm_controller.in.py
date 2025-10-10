#!/usr/bin/env @MC_LOG_UI_PYTHON_EXECUTABLE@
# -*- coding: utf-8 -*-

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

import argparse
from importlib import util, machinery
import os


def load_source(modname, filename):
    loader = machinery.SourceFileLoader(modname, filename)
    spec = util.spec_from_file_location(modname, filename, loader=loader)
    module = util.module_from_spec(spec)
    loader.exec_module(module)
    return module


try:
    mod = load_source(
        "mc_rtc_new_controller", os.path.dirname(__file__) + "/mc_rtc_new_controller"
    )
except FileNotFoundError:
    mod = load_source(
        "mc_rtc_new_controller", os.path.dirname(__file__) + "/mc_rtc_new_controller.py"
    )
new_controller = mod.new_controller


def new_fsm_controller(project_dir, controller_class_name, controller_name):
    if not os.path.isabs(project_dir):
        return new_fsm_controller(
            os.path.abspath(project_dir), controller_class_name, controller_name
        )
    if controller_name is None:
        return new_fsm_controller(
            project_dir, controller_class_name, controller_class_name
        )
    repo = new_controller(
        project_dir,
        controller_class_name,
        controller_name,
        base_class="mc_control::fsm::Controller",
        base_use_conf=True,
        extra_includes=["mc_control/fsm/Controller.h"],
        extra_libs="mc_rtc::mc_control_fsm",
        min_code=False,
        controller_constructor=False,
    )
    with open(project_dir + "/etc/{}.in.yaml".format(controller_name), "w") as fd:
        fd.write(
            """---
# If true, the FSM transitions are managed by an external tool
Managed: false
# If true and the FSM is self-managed, transitions should be triggered
StepByStep: true
# Change idle behaviour, if true the state is kept until transition,
# otherwise the FSM holds the last state until transition
IdleKeepState: false
# Where to look for state libraries
StatesLibraries:
- "@AROBASE@MC_STATES_DEFAULT_RUNTIME_INSTALL_PREFIX@AROBASE@"
- "@AROBASE@MC_STATES_RUNTIME_INSTALL_PREFIX@AROBASE@"
# Where to look for state files
StatesFiles:
- "@AROBASE@MC_STATES_DEFAULT_RUNTIME_INSTALL_PREFIX@AROBASE@/data"
- "@AROBASE@MC_STATES_RUNTIME_INSTALL_PREFIX@AROBASE@/data"
# If true, state factory will be more verbose
VerboseStateFactory: false
# Additional robots to load
robots:
  ground:
    module: env/ground
# General constraints, always on
constraints:
- type: contact
- type: dynamics
  damper: [0.1, 0.01, 0.5]
- type: compoundJoint
# Collision constraint
collisions:
- type: collision
  useMinimal: true
# Initial set of contacts
contacts:
- r2: ground
  r1Surface: LeftFoot
  r2Surface: AllGround
- r2: ground
  r1Surface: RightFoot
  r2Surface: AllGround

# Some options for a specific robot
jvrc1:
  posture:
    stiffness: 1
    weight: 10
  ff:
    stiffness: 2
    weight: 100
# Implement some additional text states
states: {{}}
# Transitions map
transitions:
- [{controller_name}_Initial, OK, {controller_name}_Initial, Strict]
# Initial state
init: {controller_name}_Initial
""".format(
                controller_name=controller_name,
                controller_class_name=controller_class_name,
            )
        )
    with open(project_dir + "/src/CMakeLists.txt", "w") as fd:
        fd.write(
            """set(controller_SRC
  {controller_class_name}.cpp
)

set(controller_HDR
  {controller_class_name}.h
)

add_library(${{PROJECT_NAME}} SHARED ${{controller_SRC}} ${{controller_HDR}})
set_target_properties(${{PROJECT_NAME}} PROPERTIES COMPILE_FLAGS "-D{controller_class_name}_EXPORTS")
target_link_libraries(${{PROJECT_NAME}} PUBLIC mc_rtc::mc_control_fsm)
install(TARGETS ${{PROJECT_NAME}}
  ARCHIVE DESTINATION "${{MC_RTC_LIBDIR}}"
  LIBRARY DESTINATION "${{MC_RTC_LIBDIR}}"
  RUNTIME DESTINATION bin)

add_controller(${{PROJECT_NAME}}_controller lib.cpp "")
target_link_libraries(${{PROJECT_NAME}}_controller PUBLIC ${{PROJECT_NAME}})
""".format(
                controller_class_name=controller_class_name
            )
        )
    with open(project_dir + "/src/lib.cpp", "w") as fd:
        fd.write(
            """#include "{controller_class_name}.h"

CONTROLLER_CONSTRUCTOR("{controller_name}", {controller_class_name})
""".format(
                controller_name=controller_name,
                controller_class_name=controller_class_name,
            )
        )
    os.makedirs(project_dir + "/src/states/data")
    with open(project_dir + "/src/CMakeLists.txt", "a") as fd:
        fd.write("\nadd_subdirectory(states)")
    with open(project_dir + "/src/states/data/states.json", "w") as fd:
        fd.write("{\n}")
    with open(project_dir + "/src/states/CMakeLists.txt", "w") as fd:
        fd.write(
            """add_fsm_state_simple({controller_name}_Initial)

add_fsm_data_directory(data)
""".format(
                controller_name=controller_name
            )
        )
    with open(
        project_dir + "/src/states/{}_Initial.h".format(controller_name), "w"
    ) as fd:
        fd.write(
            """#pragma once

#include <mc_control/fsm/State.h>

struct {controller_name}_Initial : mc_control::fsm::State
{{
  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;
}};
""".format(
                controller_name=controller_name
            )
        )
    with open(
        project_dir + "/src/states/{}_Initial.cpp".format(controller_name), "w"
    ) as fd:
        fd.write(
            """#include "{controller_name}_Initial.h"

#include "../{controller_class_name}.h"

void {controller_name}_Initial::configure(const mc_rtc::Configuration & config)
{{
}}

void {controller_name}_Initial::start(mc_control::fsm::Controller & ctl_)
{{
  auto & ctl = static_cast<{controller_class_name} &>(ctl_);
}}

bool {controller_name}_Initial::run(mc_control::fsm::Controller & ctl_)
{{
  auto & ctl = static_cast<{controller_class_name} &>(ctl_);
  output("OK");
  return true;
}}

void {controller_name}_Initial::teardown(mc_control::fsm::Controller & ctl_)
{{
  auto & ctl = static_cast<{controller_class_name} &>(ctl_);
}}

EXPORT_SINGLE_STATE("{controller_name}_Initial", {controller_name}_Initial)
""".format(
                controller_name=controller_name,
                controller_class_name=controller_class_name,
            )
        )
    # TODO: vscode intellisense
    # os.makedirs(project_dir + "/.vscode")
    with open(project_dir + "/.nvim.lua", "w") as fd:
        fd.write(
            """-- Project-specific Neovim configuration

-- Set up YAML schema association for this project
local lspconfig = require('lspconfig')

lspconfig.yamlls.setup{
  settings = {
    yaml = {
      schemas = {
        ["https://arntanguy.github.io/mc_rtc/schemas/mc_rtc/mc_rtc.json"] = "**/mc_rtc.yaml",
        ["https://arntanguy.github.io/mc_rtc/schemas/mc_control/FSMController.json"] = "etc/{controller_name}.in.yaml"
        }},
        ["https://arntanguy.github.io/mc_rtc/schemas/mc_control/FSMStates.json"] = {{ "src/states/data/*.yaml" }},

      },
      validate = true,
      format = {{ enable = true }},
      hover = true,
      completion = true,
    }
  }
}
-- You can add more project-specific Neovim or LSP settings below
""".format(
                controller_name=controller_name
            )
        )

    repo.index.add(
        [
            f.format(controller_name)
            for f in [
                "etc/{}.in.yaml",
                "src/CMakeLists.txt",
                "src/lib.cpp",
                "src/states/data/states.json",
                "src/states/CMakeLists.txt",
                "src/states/{}_Initial.h",
                "src/states/{}_Initial.cpp",
            ]
        ]
    )
    return repo


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Create a new mc_rtc FSM project")
    parser.add_argument(
        "project_dir",
        metavar="[project directory]",
        type=str,
        help="Path of the project",
    )
    parser.add_argument(
        "controller_class_name",
        metavar="[controller class name]",
        type=str,
        help="Name of the controller class",
    )
    parser.add_argument(
        "controller_name",
        metavar="[controller name]",
        nargs="?",
        type=str,
        help="Name of the controller, defaults to controller class name",
        default=None,
    )
    args = parser.parse_args()
    repo = new_fsm_controller(
        args.project_dir, args.controller_class_name, args.controller_name
    )
    repo.index.commit("Initial commit")
