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

from cython.operator cimport dereference as deref

from libcpp cimport bool as cppbool
from libcpp.set cimport set as cppset
from libcpp.string cimport string
from libcpp.vector cimport vector

cdef class Contact(object):
  def __ctor__(self, r1, r2, r1Surface, r2Surface, friction = mc_rbdyn.Contact.defaultFriction, eigen.Vector6d dof = None):
    if isinstance(r1, unicode):
      r1 = r1.encode(u'ascii')
    if isinstance(r1Surface, unicode):
      r1Surface = r1Surface.encode(u'ascii')
    if isinstance(r2, unicode):
      r2 = r2.encode(u'ascii')
    if isinstance(r2Surface, unicode):
      r2Surface = r2Surface.encode(u'ascii')
    if dof is None:
      self.impl = c_fsm.Contact(r1, r2, r1Surface, r2Surface, friction)
    else:
      self.impl = c_fsm.Contact(r1, r2, r1Surface, r2Surface, friction, dof.impl)
  def __cinit__(self, *args):
    if len(args) > 0:
      self.__ctor__(*args)
  property r1:
    def __get__(self):
      return self.impl.r1
    def __set__(self, r1):
      if isinstance(r1, unicode):
        r1 = r1.encode(u'ascii')
      self.impl.r1 = r1
  property r1Surface:
    def __get__(self):
      return self.impl.r1Surface
    def __set__(self, r1Surface):
      if isinstance(r1Surface, unicode):
        r1Surface = r1Surface.encode(u'ascii')
      self.impl.r1Surface = r1Surface
  property r2:
    def __get__(self):
      return self.impl.r2
    def __set__(self, r2):
      if isinstance(r2, unicode):
        r2 = r2.encode(u'ascii')
      self.impl.r2 = r2
  property r2Surface:
    def __get__(self):
      return self.impl.r2Surface
    def __set__(self, r2Surface):
      if isinstance(r2Surface, unicode):
        r2Surface = r2Surface.encode(u'ascii')
      self.impl.r2Surface = r2Surface
  property friction:
    def __get__(self):
      return self.impl.friction
    def __set__(self, friction):
      self.impl.friction = friction
  property dof:
    def __get__(self):
      return eigen.Vector6dFromC(self.impl.dof)
    def __set__(self, dof):
      if isinstance(dof, eigen.Vector6d):
        self.impl.dof = (<eigen.Vector6d>dof).impl
      else:
        self.dof = eigen.Vector6d(dof)

cdef Contact ContactFromC(const c_fsm.Contact & c):
  cdef Contact ret = Contact()
  ret.impl = c
  return ret

cdef class Controller(MCController):
  def __cinit__(self):
    self.impl = self.base = NULL
  def addCollisions(self, r1, r2, collisions):
    assert(all([isinstance(col, mc_rbdyn.Collision) for col in collisions]))
    cdef vector[c_mc_rbdyn.Collision] cols
    if isinstance(r1, unicode):
      r1 = r1.encode(u'ascii')
    if isinstance(r2, unicode):
      r2 = r2.encode(u'ascii')
    for col in collisions:
      cols.push_back((<mc_rbdyn.Collision>col).impl)
    self.impl.addCollisions(r1, r2, cols)
  def removeCollisions(self, r1, r2, collisions = None):
    cdef vector[c_mc_rbdyn.Collision] cols
    if isinstance(r1, unicode):
      r1 = r1.encode(u'ascii')
    if isinstance(r2, unicode):
      r2 = r2.encode(u'ascii')
    if collisions is None:
      self.impl.removeCollisions(r1, r2)
    else:
      for col in collisions:
        cols.push_back((<mc_rbdyn.Collision>col).impl)
      self.impl.removeCollisions(r1, r2, cols)
  def hasRobot(self, name):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.impl.hasRobot(name)
  def robot(self, name = None):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    if name is None:
      return MCController.robot(self)
    else:
      return mc_rbdyn.RobotFromC(self.impl.robot(name))
  def addContact(self, Contact c):
    self.impl.addContact(c.impl)
  def removeContact(self, Contact c):
    self.impl.removeContact(c.impl)
  def contacts(self):
    cdef c_fsm.ContactSet cs = self.impl.contacts()
    return [ContactFromC(c) for c in cs]
  def hasContact(self, Contact c):
    self.impl.hasContact(c.impl)
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
