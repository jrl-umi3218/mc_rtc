# Copyright 2012-2017 CNRS-UM LIRMM, CNRS-AIST JRL
#
# This file is part of mc_rtc.
#
# mc_rtc is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# mc_rtc is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with mc_rtc.  If not, see <http://www.gnu.org/licenses/>.

set(SETUP_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIGURATION>")
if(DEFINED CMAKE_BUILD_TYPE)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}/mc_rtc")
else()
  foreach(CFG ${CMAKE_CONFIGURATION_TYPES})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${CFG}/mc_rtc")
  endforeach()
endif()
configure_file(setup.in.py "${CMAKE_CURRENT_BINARY_DIR}/setup.in.py")
file(GENERATE OUTPUT "${SETUP_LOCATION}/setup.py" INPUT "${CMAKE_CURRENT_BINARY_DIR}/setup.in.py")

if(${ROSCPP_FOUND})
  set(DEFINE_HAS_ROS_PYTHON_MACRO "DEF MC_RTC_HAS_ROS = 1")
else()
  set(DEFINE_HAS_ROS_PYTHON_MACRO "DEF MC_RTC_HAS_ROS = 0")
endif()

configure_file(mc_rtc/mc_rtc_config.pxi.in "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc_config.pxi.in")
file(GENERATE OUTPUT "${SETUP_LOCATION}/mc_rtc_config.pxi" INPUT "${CMAKE_CURRENT_BINARY_DIR}/mc_rtc_config.pxi.in")

# Build the bindings locally at build time for test purposes
add_custom_target(mc_rtc-python-bindings ALL
  COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" python setup.py build_ext --inplace
  COMMENT "Generating local mc_rtc Python bindings"
  DEPENDS include/mc_control_wrapper.hpp include/mc_rbdyn_wrapper.hpp include/mc_solver_wrapper.hpp
          mc_control/c_mc_control.pxd mc_control/mc_control.pxd mc_control/mc_control.pyx
          mc_rbdyn/c_mc_rbdyn.pxd mc_rbdyn/mc_rbdyn.pxd mc_rbdyn/mc_rbdyn.pyx
          mc_rtc/c_mc_rtc.pxd mc_rtc/mc_rtc.pyx
          mc_solver/c_mc_solver.pxd mc_solver/mc_solver.pxd mc_solver/mc_solver.pyx
          mc_tasks/c_mc_tasks.pxd mc_tasks/mc_tasks.pxd mc_tasks/mc_tasks.pyx
  SOURCES include/mc_control_wrapper.hpp include/mc_rbdyn_wrapper.hpp include/mc_solver_wrapper.hpp
          mc_control/c_mc_control.pxd mc_control/mc_control.pxd mc_control/mc_control.pyx
          mc_rbdyn/c_mc_rbdyn.pxd mc_rbdyn/mc_rbdyn.pxd mc_rbdyn/mc_rbdyn.pyx
          mc_rtc/c_mc_rtc.pxd mc_rtc/mc_rtc.pyx
          mc_solver/c_mc_solver.pxd mc_solver/mc_solver.pxd mc_solver/mc_solver.pyx
          mc_tasks/c_mc_tasks.pxd mc_tasks/mc_tasks.pxd mc_tasks/mc_tasks.pyx
)
add_dependencies(mc_rtc-python-bindings mc_control mc_control_fsm)

set(BINDINGS_GENERATED_SOURCES
  ${SETUP_LOCATION}/mc_control/mc_control.cpp
  ${SETUP_LOCATION}/mc_rbdyn/mc_rbdyn.cpp
  ${SETUP_LOCATION}/mc_rtc/mc_rtc.cpp
  ${SETUP_LOCATION}/mc_solver/mc_solver.cpp
  ${SETUP_LOCATION}/mc_tasks/mc_tasks.cpp
  ${SETUP_LOCATION}/mc_rtc/gui/gui.cpp
  ${SETUP_LOCATION}/mc_control/fsm/fsm.cpp
)

# Force rebuild
add_custom_target(force-mc-rtc-python-bindings
  COMMAND ${CMAKE_COMMAND} -E remove -f ${BINDINGS_GENERATED_SOURCES}
  COMMAND ${CMAKE_MAKE_PROGRAM} mc_rtc-python-bindings
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  COMMENT "Remove Cython generated files"
)

if(NOT ${DISABLE_TESTS})
  if(NOT DEFINED PYTHON_DEB_ROOT)
    ADD_TEST(NAME PythonBindingsTest
      COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" nosetests
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
  endif()
endif()

set(PIP_EXTRA_OPTIONS "")
if(${PYTHON_BINDING_USER_INSTALL})
  set(PIP_EXTRA_OPTIONS "--user")
endif()
# Install the bindings
if(DEFINED PYTHON_DEB_ROOT)
  add_custom_target(install-mc_rtc-python-bindings
    COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" python setup.py install --root=${PYTHON_DEB_ROOT} --install-layout=deb && python3 setup.py install --root=${PYTHON_DEB_ROOT} --install-layout=deb
    COMMENT "Install mc_rtc Python bindings (Debian package)"
  )
else()
  add_custom_target(install-mc_rtc-python-bindings
    COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" pip install . ${PIP_EXTRA_OPTIONS}
    COMMENT "Install mc_rtc Python bindings"
  )
  add_custom_target(force-install-mc_rtc-python-bindings
    COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" pip install . ${PIP_EXTRA_OPTIONS} --upgrade
    COMMENT "Install mc_rtc Python bindings (forced)"
  )
endif()
install(CODE "EXECUTE_PROCESS(COMMAND \"${CMAKE_COMMAND}\" --build \"${CMAKE_BINARY_DIR}\" --config \${CMAKE_INSTALL_CONFIG_NAME} --target install-mc_rtc-python-bindings)")
