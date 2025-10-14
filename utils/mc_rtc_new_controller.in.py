#!/usr/bin/env @MC_LOG_UI_PYTHON_EXECUTABLE@
# -*- coding: utf-8 -*-

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from __future__ import print_function

import argparse
import git
import os
import re
import sys
import json


def new_controller(
    project_dir,
    controller_class_name,
    controller_name,
    base_class="mc_control::MCController",
    base_use_conf=False,
    extra_includes=[],
    extra_libs="",
    min_code=True,
    controller_constructor=True,
):
    if not os.path.isabs(project_dir):
        return new_controller(
            os.path.abspath(project_dir),
            controller_class_name,
            controller_name,
            base_class,
            base_use_conf,
            extra_includes,
            extra_libs,
            min_code,
            controller_constructor,
        )
    extra_includes = "\n".join(["#include <{}>".format(i) for i in extra_includes])
    if len(extra_libs):
        extra_libs = "target_link_libraries(${{PROJECT_NAME}} PUBLIC {})".format(
            extra_libs
        )
    if controller_name is None:
        return new_controller(
            project_dir,
            controller_class_name,
            controller_class_name,
            base_class,
            base_use_conf,
            extra_includes,
            extra_libs,
            min_code,
            controller_constructor,
        )
    if os.path.exists(project_dir):
        print("{} already exists, aborting project creation".format(project_dir))
        sys.exit(1)
    if re.match("[A-Z][A-z0-9]*", controller_class_name) is None:
        print(
            "Controller class name must start with a capital letter and be composed of letters and numbers only"
        )
        sys.exit(1)
    os.makedirs(project_dir)
    repo = git.Repo.init(project_dir)
    os.makedirs(project_dir + "/src")
    os.makedirs(project_dir + "/etc")
    with open(project_dir + "/CMakeLists.txt", "w") as fd:
        configure_config = """set(AROBASE "@")
configure_file(etc/{controller_name}.in.yaml "${{CMAKE_CURRENT_BINARY_DIR}}/etc/{controller_name}.yaml")
install(FILES "${{CMAKE_CURRENT_BINARY_DIR}}/etc/{controller_name}.yaml" DESTINATION "${{MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}}/etc")
""".format(
            controller_name=controller_name
        )
        fd.write(
            """cmake_minimum_required(VERSION 3.10)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME {controller_class_name})
set(PROJECT_DESCRIPTION "{controller_name}")
set(PROJECT_URL "")

project(${{PROJECT_NAME}} CXX)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

{configure_config}
""".format(
                controller_name=controller_name,
                controller_class_name=controller_class_name,
                configure_config=configure_config,
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

add_controller(${{PROJECT_NAME}} ${{controller_SRC}} ${{controller_HDR}})
set_target_properties(${{PROJECT_NAME}} PROPERTIES COMPILE_FLAGS "-D{controller_class_name}_EXPORTS")
{extra_libs}
""".format(
                controller_class_name=controller_class_name, extra_libs=extra_libs
            )
        )
    with open(project_dir + "/src/" + "api.h", "w") as fd:
        fd.write(
            """#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define {controller_class_name}_DLLIMPORT __declspec(dllimport)
#  define {controller_class_name}_DLLEXPORT __declspec(dllexport)
#  define {controller_class_name}_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define {controller_class_name}_DLLIMPORT __attribute__((visibility("default")))
#    define {controller_class_name}_DLLEXPORT __attribute__((visibility("default")))
#    define {controller_class_name}_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define {controller_class_name}_DLLIMPORT
#    define {controller_class_name}_DLLEXPORT
#    define {controller_class_name}_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef {controller_class_name}_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define {controller_class_name}_DLLAPI
#  define {controller_class_name}_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef {controller_class_name}_EXPORTS
#    define {controller_class_name}_DLLAPI {controller_class_name}_DLLEXPORT
#  else
#    define {controller_class_name}_DLLAPI {controller_class_name}_DLLIMPORT
#  endif // {controller_class_name}_EXPORTS
#  define {controller_class_name}_LOCAL {controller_class_name}_DLLLOCAL
#endif // {controller_class_name}_STATIC""".format(
                controller_class_name=controller_class_name
            )
        )
    with open(project_dir + "/src/" + controller_class_name + ".h", "w") as fd:
        fd.write(
            """#pragma once

#include <mc_control/mc_controller.h>
{extra_includes}

#include "api.h"

struct {controller_class_name}_DLLAPI {controller_class_name} : public {base_class}
{{
  {controller_class_name}(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;
}};""".format(
                controller_class_name=controller_class_name,
                base_class=base_class,
                extra_includes=extra_includes,
            )
        )

    with open(project_dir + "/src/" + controller_class_name + ".cpp", "w") as fd:
        base_init = "{}(rm, dt".format(base_class)
        if base_use_conf:
            base_init += ", config"
        base_init += ")"
        if min_code:
            min_code = """  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({{}});
"""
        else:
            min_code = ""
        if controller_constructor:
            controller_constructor = """CONTROLLER_CONSTRUCTOR("{controller_name}", {controller_class_name})""".format(
                controller_name=controller_name,
                controller_class_name=controller_class_name,
            )
        else:
            controller_constructor = ""
        fd.write(
            """#include "{controller_class_name}.h"

{controller_class_name}::{controller_class_name}(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: {base_init}
{{
{min_code}
  mc_rtc::log::success("{controller_name} init done ");
}}

bool {controller_class_name}::run()
{{
  return {base_class}::run();
}}

void {controller_class_name}::reset(const mc_control::ControllerResetData & reset_data)
{{
  {base_class}::reset(reset_data);
}}

{controller_constructor}
""".format(
                controller_class_name=controller_class_name,
                controller_name=controller_name,
                base_class=base_class,
                base_init=base_init,
                min_code=min_code,
                controller_constructor=controller_constructor,
            )
        )
    with open(project_dir + "/etc/" + controller_name + ".in.yaml", "w") as fd:
        fd.write("---\n")

    # VSCode config
    vscode_dir = os.path.join(project_dir, ".vscode")
    os.makedirs(vscode_dir, exist_ok=True)
    vscode_settings = {
        "yaml.schemas": {
            "https://arntanguy.github.io/mc_rtc/schemas/mc_rtc/mc_rtc.json": "**/mc_rtc.yaml",
            "https://arntanguy.github.io/mc_rtc/schemas/mc_control/FSMController.json": "etc/{}.yaml".format(
                controller_name
            ),
        },
        "yaml.validate": True,
        "yaml.format.enable": False,
        "yaml.hover": True,
        "yaml.completion": True,
    }
    with open(os.path.join(vscode_dir, "settings.json"), "w") as f:
        json.dump(vscode_settings, f, indent=2)

    # Neovim config
    with open(os.path.join(project_dir, ".nvim.lua"), "w") as fd:
        fd.write(
            """-- Project-specific Neovim configuration

-- Set up YAML schema association for this project
vim.lsp.config('yamlls',
{{
  settings = {{
    yaml = {{
      schemas = {{
        ["https://arntanguy.github.io/mc_rtc/schemas/mc_rtc/mc_rtc.json"] = "**/mc_rtc.yaml",
        ["https://arntanguy.github.io/mc_rtc/schemas/mc_control/FSMController.json"] = "etc/{controller_name}.yaml",
        ["https://arntanguy.github.io/mc_rtc/schemas/mc_control/FSMStates.json"] = "src/states/data/*.yaml"
      }},
      validate = true,
      format = {{ enable = false }},
      hover = true,
      completion = true,
    }}
  }}
}}
""".format(
                controller_name=controller_name
            )
        )
    repo.index.add(
        [
            s.format(controller_class_name)
            for s in [
                "CMakeLists.txt",
                "src/CMakeLists.txt",
                "src/api.h",
                "etc/" + controller_name + ".in.yaml",
                "src/{}.h",
                "src/{}.cpp",
                ".vscode/",
                ".nvim.lua",
            ]
        ]
    )
    return repo


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Create a new mc_rtc Controller project"
    )
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
    repo = new_controller(
        args.project_dir, args.controller_class_name, args.controller_name
    )
    repo.index.commit("Initial commit")
