#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport c_fsm

from mc_control.mc_control cimport MCController

cdef class Controller(MCController):
  cdef c_fsm.Controller * impl

cdef Controller ControllerFromPtr(c_fsm.Controller * ctl)

cdef public class PythonState(object)[object PythonStateObject, type PythonStateType]:
  cdef c_fsm.PythonState * impl
