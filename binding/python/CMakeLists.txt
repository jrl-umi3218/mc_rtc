#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

if(${ROSCPP_FOUND})
  set(DEFINE_HAS_ROS_PYTHON_MACRO "DEF MC_RTC_HAS_ROS = 1")
else()
  set(DEFINE_HAS_ROS_PYTHON_MACRO "DEF MC_RTC_HAS_ROS = 0")
endif()

add_cython_bindings(mc_rtc
  TARGETS mc_control_fsm mc_control mc_rbdyn mc_rtc_loader mc_rtc_utils mc_rtc_gui mc_solver mc_tasks mc_trajectory mc_rtc_ros mc_control_client
  VERSION 1.0.0
  MODULES mc_control.mc_control
          mc_control.fsm.fsm
          mc_rbdyn.mc_rbdyn
          mc_rtc.mc_rtc
          mc_rtc.gui.gui
          mc_solver.mc_solver
          mc_tasks.mc_tasks
          mc_tasks.force.force
  EXPORT_SOURCES
    mc_control/fsm/__init__.py
    mc_control/fsm/c_fsm.pxd
    mc_control/fsm/fsm.pxd
    mc_control/__init__.py
    mc_control/c_mc_control.pxd
    mc_control/mc_control.pxd
    mc_rbdyn/__init__.py
    mc_rbdyn/c_mc_rbdyn.pxd
    mc_rbdyn/mc_rbdyn.pxd
    mc_rtc/gui/__init__.py
    mc_rtc/gui/c_gui.pxd
    mc_rtc/gui/gui.pxd
    mc_rtc/__init__.py
    mc_rtc/c_mc_rtc.pxd
    mc_rtc/mc_rtc.pxd
    mc_solver/__init__.py
    mc_solver/c_mc_solver.pxd
    mc_solver/mc_solver.pxd
    mc_tasks/force/__init__.py
    mc_tasks/force/c_force.pxd
    mc_tasks/force/force.pxd
    mc_tasks/__init__.py
    mc_tasks/mc_tasks.pxd
    mc_tasks/c_mc_tasks.pxd
  PRIVATE_SOURCES
    include/mc_control_fsm_wrapper.hpp
    include/mc_control_wrapper.hpp
    include/mc_rtc_gui_wrapper.hpp
    include/mc_rtc_wrapper.hpp
    include/mc_solver_wrapper.hpp
    include/mc_rbdyn_wrapper.hpp
    mc_control/fsm/fsm.pyx
    mc_control/mc_control.pyx
    mc_rbdyn/mc_rbdyn.pyx
    mc_rtc/gui/gui.pyx
    mc_rtc/mc_rtc.pyx
    mc_solver/mc_solver.pyx
    mc_tasks/force/force.pyx
    mc_tasks/mc_tasks.pyx
    mc_tasks/com_trajectory_task.pxi
    mc_tasks/position_trajectory_task.pxi
    mc_tasks/orientation_trajectory_task.pxi
    mc_tasks/vector_orientation_trajectory_task.pxi
    tests/test_mc_rtc.py
  GENERATE_SOURCES
    mc_rtc/mc_rtc_config.pxi
    tests/test_mc_tasks.py
)

