# distutils: language = c++

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport c_fsm

cimport eigen.eigen as eigen
cimport mc_rtc.c_mc_rtc as c_mc_rtc
cimport mc_rtc.mc_rtc as mc_rtc
cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_rbdyn.mc_rbdyn as mc_rbdyn
cimport mc_solver.mc_solver as mc_solver

from mc_control.mc_control cimport MCController
cimport mc_control.mc_control as mc_control

from cython.operator cimport dereference as deref

from libcpp cimport bool as cppbool
from libcpp.set cimport set as cppset
from libcpp.string cimport string
from libcpp.vector cimport vector

Contact = mc_control.Contact

cdef class Controller(MCController):
  def __cinit__(self):
    self.impl = self.base = NULL
  def contactConstraint(self):
    return mc_solver.ContactConstraintFromPtr(&(self.impl.contactConstraint()))

cdef Controller ControllerFromPtr(c_fsm.Controller * ctl):
  cdef Controller ret = Controller()
  ret.impl = ret.base = ctl
  return ret

cdef void python_configure_cb(PythonState s, c_mc_rtc.Configuration & config) with gil:
  s.configure(mc_rtc.ConfigurationFromRef(config))

cdef void python_start_cb(PythonState s, c_fsm.Controller & ctl) with gil:
  s.start(ControllerFromPtr(&ctl))

cdef cppbool python_run_cb(PythonState s, c_fsm.Controller & ctl) with gil:
  return s.run(ControllerFromPtr(&ctl))

cdef void python_teardown_cb(PythonState s, c_fsm.Controller & ctl) with gil:
  s.teardown(ControllerFromPtr(&ctl))

cdef void python_stop_cb(PythonState s, c_fsm.Controller & ctl) with gil:
  s.stop(ControllerFromPtr(&ctl))

cdef class PythonState(object):
  def __dealloc__(self):
    del self.impl
  def __cinit__(self):
    self.impl = new c_fsm.PythonState()
    try:
      self.configure
      self.impl.configure_ = c_fsm.make_configure_cb(python_configure_cb, self)
    except AttributeError:
      pass
    try:
      self.start
      self.impl.start_ = c_fsm.make_controller_cb(python_start_cb, self)
    except AttributeError:
      pass
    try:
      self.run
      self.impl.run_ = c_fsm.make_run_cb(python_run_cb, self)
    except AttributeError:
      pass
    try:
      self.teardown
      self.impl.teardown_ = c_fsm.make_controller_cb(python_teardown_cb, self)
    except AttributeError:
      pass
    try:
      self.stop
      self.impl.stop_ = c_fsm.make_controller_cb(python_stop_cb, self)
    except AttributeError:
      pass
