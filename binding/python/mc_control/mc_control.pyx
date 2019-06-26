# distutils: language = c++

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_control.c_mc_control as c_mc_control

cimport eigen.eigen as eigen

cimport sva.c_sva as c_sva
cimport sva.sva as sva

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_rbdyn.mc_rbdyn as mc_rbdyn

cimport mc_solver.mc_solver as mc_solver

cimport mc_tasks.mc_tasks as mc_tasks

cimport mc_rtc.mc_rtc as mc_rtc
cimport mc_rtc.gui.gui as mc_rtc_gui

from cython.operator cimport preincrement as preinc
from cython.operator cimport dereference as deref
from libcpp.map cimport map as cppmap
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef class Gripper(object):
  def __cinit__(self):
    self.impl = c_mc_control.GripperPtr(NULL)
  cdef c_mc_control.Gripper * _impl(self):
    assert(self.impl.get() != NULL)
    return self.impl.get()
  property names:
    def __get__(self):
      return self._impl().names
  property q:
    def __get__(self):
      return self._impl()._q

cdef Gripper GripperFromShPtr(c_mc_control.shared_ptr[c_mc_control.Gripper] p):
  cdef Gripper ret = Gripper()
  ret.impl = p
  return ret

cdef class ControllerResetData(object):
  def __cinit__(self):
    pass
  property q:
    def __get__(self):
      return self.impl.q

cdef ControllerResetData ControllerResetDataFromPtr(c_mc_control.ControllerResetData * p):
    cdef ControllerResetData ret = ControllerResetData()
    ret.impl = p
    return ret

cdef class MCController(object):
  def __cinit__(self):
    pass
  def run(self):
    return self.base.run()
  def reset(self, ControllerResetData data):
    self.base.reset(deref(data.impl))
  def robot(self):
    return mc_rbdyn.RobotFromC(self.base.robot())
  def env(self):
    return mc_rbdyn.RobotFromC(self.base.env())
  def robots(self):
    return mc_rbdyn.RobotsFromRawPtr(&(self.base.robots()))
  def set_joint_pos(self, jname, pos):
    if isinstance(jname, unicode):
      jname = jname.encode(u'ascii')
    return self.base.set_joint_pos(jname, pos)
  def play_next_stance(self):
    return self.base.play_next_stance()
  def read_msg(self, msg):
    if isinstance(msg, unicode):
      msg = msg.encode(u'ascii')
    return self.base.read_msg(msg)
  def read_write_msg(self, msg, out):
    if isinstance(msg, unicode):
      msg = msg.encode(u'ascii')
    return self.base.read_write_msg(msg, out)
  def supported_robots(self):
    return self.base.supported_robots()
  def logger(self):
    return mc_rtc.LoggerFromRef(self.base.logger())
  def gui(self):
    return mc_rtc_gui.StateBuilderFromShPtr(self.base.gui())
  property timeStep:
    def __get__(self):
      return self.base.timeStep
  property contactConstraint:
    def __get__(self):
      return mc_solver.ContactConstraintFromPtr(&self.base.contactConstraint)
  property dynamicsConstraint:
    def __get__(self):
      return mc_solver.DynamicsConstraintFromPtr(&self.base.dynamicsConstraint)
  property kinematicsConstraint:
    def __get__(self):
      return mc_solver.KinematicsConstraintFromPtr(&self.base.kinematicsConstraint)
  property selfCollisionConstraint:
    def __get__(self):
      return mc_solver.CollisionsConstraintFromPtr(&self.base.selfCollisionConstraint)
  property postureTask:
    def __get__(self):
      return mc_tasks.PostureTaskFromPtr(self.base.postureTask.get())
  property qpsolver:
    def __get__(self):
      return mc_solver.QPSolverFromRef(self.base.solver())
  property grippers:
    def __get__(self):
      return {g.first: GripperFromShPtr(g.second) for g in self.base.grippers}

cdef MCController MCControllerFromPtr(c_mc_control.MCController * p):
    cdef MCController ret = MCController()
    ret.base = p
    return ret

cdef class PythonRWCallback(object):
  def __cinit__(self, succ, out):
    self.impl.success = succ
    self.impl.out = out
  property success:
    def __get__(self):
      return self.impl.success
    def __set__(self, value):
      self.impl.success = value
  property out:
    def __get__(self):
      return self.impl.out
    def __set__(self, value):
      self.impl.out = value

cdef cppbool python_to_run_callback(void * f) with gil:
  return (<object>f).run_callback()

cdef void python_to_reset_callback(const c_mc_control.ControllerResetData & crd, void * f) with gil:
  (<object>f).reset_callback(ControllerResetDataFromPtr(&(c_mc_control.const_cast_crd(crd))))

cdef cppbool python_to_read_msg_callback(string & msg, void * f) with gil:
  return (<object>f).read_msg_callback(msg)

cdef c_mc_control.PythonRWCallback python_to_read_write_msg_callback(string & msg, void * f) with gil:
  cdef PythonRWCallback ret = PythonRWCallback(*(<object>f).read_write_msg_callback(msg))
  return ret.impl

cdef class MCPythonController(MCController):
  def __dealloc__(self):
    del self.impl
    self.impl = self.base = NULL
  def __cinit__(self, robot_modules, double dt):
    cdef mc_rbdyn.RobotModuleVector rmv = mc_rbdyn.RobotModuleVector(robot_modules)
    self.impl = self.base = new c_mc_control.MCPythonController(rmv.v, dt)
    try:
      self.run_callback
      c_mc_control.set_run_callback(deref(self.impl), &python_to_run_callback, <void*>(self))
    except AttributeError:
      raise TypeError("You need to implement a run_callback method in your object")
    try:
      self.reset_callback
      c_mc_control.set_reset_callback(deref(self.impl), &python_to_reset_callback, <void*>(self))
    except AttributeError:
      pass
    try:
      self.read_msg_callback
      c_mc_control.set_read_msg_callback(deref(self.impl), &python_to_read_msg_callback, <void*>(self))
    except AttributeError:
      pass
    try:
      self.read_write_msg_callback
      c_mc_control.set_read_write_msg_callback(deref(self.impl), &python_to_read_write_msg_callback, <void*>(self))
    except AttributeError:
      pass

cdef class MCGlobalController(object):
  def __dealloc__(self):
    del self.impl
    self.impl = NULL
  def __cinit_simple__(self):
    self.impl = new c_mc_control.MCGlobalController()
  def __cinit_conf__(self, conf):
    if isinstance(conf, unicode):
      conf = conf.encode(u'ascii')
    self.impl = new c_mc_control.MCGlobalController(conf)
  def __cinit_full__(self, conf, mc_rbdyn.RobotModule rm):
    if isinstance(conf, unicode):
      conf = conf.encode(u'ascii')
    self.impl = new c_mc_control.MCGlobalController(conf, rm.impl)
  def __cinit__(self, *args):
    if len(args) == 0:
      self.__cinit_simple__()
    elif len(args) == 1:
      self.__cinit_conf__(args[0])
    elif len(args) == 2:
      self.__cinit_full__(args[0], args[1])
    else:
      raise TypeError("Wrong arguments passed to MCGlobalController ctor")
  def init(self, q, pos = None):
    cdef c_mc_control.array7d p = c_mc_control.array7d()
    if pos is None:
      self.impl.init(q)
    else:
      assert(len(pos) == 7)
      for i,pi in enumerate(pos):
        p[i] = pos[i]
      self.impl.init(q, p)
  def setSensorPosition(self, eigen.Vector3d p):
    self.impl.setSensorPosition(p.impl)
  def setSensorOrientation(self, eigen.Quaterniond q):
    self.impl.setSensorOrientation(q.impl)
  def setSensorLinearVelocity(self, eigen.Vector3d lv):
    self.impl.setSensorLinearVelocity(lv.impl)
  def setSensorAngularVelocity(self, eigen.Vector3d av):
    self.impl.setSensorAngularVelocity(av.impl)
  def setSensorAcceleration(self, eigen.Vector3d a):
    self.impl.setSensorAcceleration(a.impl)
  def setEncoderValues(self, q):
    self.impl.setEncoderValues(q)
  def setEncoderVelocities(self, alpha):
    self.impl.setEncoderVelocities(alpha)
  def setFlexibilityValues(self, flex):
    self.impl.setFlexibilityValues(flex)
  def setJointTorques(self, tau):
    self.impl.setJointTorques(tau)
  def setWrenches(self, wrenchesIn):
    cdef cppmap[string, c_sva.ForceVecd] wrenches = cppmap[string, c_sva.ForceVecd]()
    for sensor,w in wrenchesIn.iteritems():
      if not isinstance(w, sva.ForceVecd):
        w = sva.ForceVecd(w)
      wrenches[sensor] = deref((<sva.ForceVecd>(w)).impl)
    self.impl.setWrenches(wrenches)
  def setActualGripperQ(self, gripperQ):
    self.impl.setActualGripperQ(gripperQ)

  def run(self):
    return self.impl.run()

  def timestep(self):
    return self.impl.timestep()
  def controller(self):
    return MCControllerFromPtr(&(self.impl.controller()))
  def ref_joint_order(self):
    return self.impl.ref_joint_order()
  def robot(self):
    return mc_rbdyn.RobotFromC(self.impl.robot())
  property running:
    def __get__(self):
      return self.impl.running
    def __set__(self, b):
      self.impl.running = b
