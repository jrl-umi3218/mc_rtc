# distutils: language = c++

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_solver.c_mc_solver as c_mc_solver

cimport tasks.qp.c_qp as c_qp
cimport tasks.qp.qp as qp

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_rbdyn.mc_rbdyn as mc_rbdyn

cimport mc_tasks.mc_tasks as mc_tasks

cimport eigen.eigen as eigen

from cython.operator cimport preincrement as preinc
from cython.operator cimport dereference as deref
from libcpp.map cimport map as cppmap
from libcpp.pair cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef class ConstraintSet(object):
  def __cinit__(self):
    self.cs_base = NULL

cdef class ContactConstraint(ConstraintSet):
  Acceleration = c_mc_solver.ContactTypeAcceleration
  Velocity = c_mc_solver.ContactTypeVelocity
  Position = c_mc_solver.ContactTypePosition
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, timeStep, int cType = ContactConstraint.Velocity, dynamics = True, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      assert(cType >= ContactConstraint.Acceleration and cType <= ContactConstraint.Position)
      self.impl = self.cs_base = new c_mc_solver.ContactConstraint(timeStep, <c_mc_solver.ContactConstraintContactType>cType)
  property contactConstr:
    def __get__(self):
      cdef c_mc_solver.ContactConstrCastResult cr = c_mc_solver.get_contact_constr(deref(self.impl))
      if cr.acc:
        return qp.ContactAccConstrFromPtr(cr.acc)
      elif cr.speed:
        return qp.ContactSpeedConstrFromPtr(cr.speed)
      elif cr.pos:
        return qp.ContactPosConstrFromPtr(cr.pos)
      else:
        raise TypeError("Impossible (ContactConstraint)")

cdef ContactConstraint ContactConstraintFromPtr(c_mc_solver.ContactConstraint * p):
    cdef ContactConstraint ret = ContactConstraint(0.0, skip_alloc = True)
    ret.__own_impl = False
    ret.impl = ret.cs_base = p
    return ret

cdef class KinematicsConstraint(ConstraintSet):
  def __dealloc__(self):
    if type(self) is KinematicsConstraint and self.__own_impl:
      del self.impl
  def __cinit__(self, mc_rbdyn.Robots robots, robotIndex, timeStep, damper = None, velocityPercent = 1.0, infTorque = False, skip_alloc = False):
    cdef c_mc_solver.array3d damp = c_mc_solver.array3d()
    self.__own_impl = True
    if type(self) is KinematicsConstraint and not skip_alloc:
      if damper is None:
        self.impl = self.cs_base = new c_mc_solver.KinematicsConstraint(deref(robots.impl), robotIndex, timeStep)
      else:
        assert(len(damper) == 3)
        for i in xrange(3):
          damp[i] = damper[i]
        self.impl = self.cs_base = new c_mc_solver.KinematicsConstraint(deref(robots.impl), robotIndex, timeStep, damp, velocityPercent)

cdef KinematicsConstraint KinematicsConstraintFromPtr(c_mc_solver.KinematicsConstraint*p):
    cdef KinematicsConstraint ret = KinematicsConstraint(None, 0, 0.0, skip_alloc = True)
    ret.__own_impl = False
    ret.impl = ret.cs_base = p
    return ret

cdef class DynamicsConstraint(KinematicsConstraint):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, mc_rbdyn.Robots robots, robotIndex, timeStep, isStatic = False, damper = None, velocityPercent = 1.0, infTorque = False, skip_alloc = False):
    cdef c_mc_solver.array3d damp = c_mc_solver.array3d()
    self.__own_impl = True
    if not skip_alloc:
      if damper is None:
        self.d_impl = self.impl = self.cs_base = new c_mc_solver.DynamicsConstraint(deref(robots.impl), robotIndex, timeStep, infTorque)
      else:
        assert(len(damper) == 3)
        for i in xrange(3):
          damp[i] = damper[i]
        self.d_impl = self.impl = self.cs_base = new c_mc_solver.DynamicsConstraint(deref(robots.impl), robotIndex, timeStep, damp, velocityPercent, infTorque)

cdef DynamicsConstraint DynamicsConstraintFromPtr(c_mc_solver.DynamicsConstraint * p):
    cdef DynamicsConstraint ret = DynamicsConstraint(None, 0, 0.0, skip_alloc = True)
    ret.__own_impl = False
    ret.d_impl = ret.impl = ret.cs_base = p
    return ret

cdef class CollisionsConstraint(ConstraintSet):
  defaultDampingOffset = c_mc_solver.CollisionsConstraintDefaultDampingOffset
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, mc_rbdyn.Robots robots, r1Index, r2Index, timeStep, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = self.cs_base = new c_mc_solver.CollisionsConstraint(deref(robots.impl), r1Index, r2Index, timeStep)
  def removeCollision(self, QPSolver solver, b1Name, b2Name):
    if isinstance(b1Name, unicode):
        b1Name = b1Name.encode(u'ascii')
    if isinstance(b2Name, unicode):
        b2Name = b2Name.encode(u'ascii')
    return self.impl.removeCollision(deref(solver.impl), b1Name, b2Name)
  def removeCollisionByBody(self, QPSolver solver, b1Name, b2Name):
    if isinstance(b1Name, unicode):
        b1Name = b1Name.encode(u'ascii')
    if isinstance(b2Name, unicode):
        b2Name = b2Name.encode(u'ascii')
    return self.impl.removeCollisionByBody(deref(solver.impl), b1Name, b2Name)
  def addCollision(self, QPSolver solver, mc_rbdyn.Collision col):
    self.impl.addCollision(deref(solver.impl), col.impl)
  def addCollisions(self, QPSolver solver, cols):
    for c in cols:
      self.addCollision(solver, c)
  def reset(self):
    self.impl.reset()
  property r1Index:
    def __get__(self):
      return self.impl.r1Index
  property r2Index:
    def __get__(self):
      return self.impl.r2Index
  property cols:
    def __get__(self):
      end = deref(self.impl).cols.end()
      it = deref(self.impl).cols.begin()
      ret = []
      while it != end:
        ret.append(mc_rbdyn.CollisionFromC(deref(it)))
        preinc(it)
      return ret

cdef CollisionsConstraint CollisionsConstraintFromPtr(c_mc_solver.CollisionsConstraint * p):
    cdef CollisionsConstraint ret = CollisionsConstraint(None, None, None, None, skip_alloc = True)
    ret.__own_impl = False
    ret.impl = ret.cs_base = p
    return ret

cdef qp.BilateralContact BilateralContactFromC(const c_qp.BilateralContact & bc):
  cdef qp.BilateralContact ret = qp.BilateralContact()
  ret.impl = bc
  return ret

cdef class QPSolver(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, mc_rbdyn.Robots robots, timeStep, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = new c_mc_solver.TasksQPSolver(robots.impl, timeStep)
  def addConstraintSet(self, ConstraintSet cs):
    self.impl.addConstraintSet(deref(cs.cs_base))
  def removeConstraintSet(self, ConstraintSet cs):
    self.impl.removeConstraintSet(deref(cs.cs_base))
  def setContacts(self, contacts):
    self.impl.setContacts(deref(mc_rbdyn.ContactVector(contacts).v))
  def contacts(self):
    ret = []
    for i in range(self.impl.contacts().size()):
      ret.append(mc_rbdyn.ContactFromC(self.impl.contacts()[i]))
    return ret
  def addTask(self, mc_tasks.MetaTask task):
    self.impl.addTask(task.mt_base)
  def removeTask(self, mc_tasks.MetaTask task):
    self.impl.removeTask(task.mt_base)
  def run(self):
    return self.impl.run()
  property robots:
    def __get__(self):
      return mc_rbdyn.RobotsFromRef(self.impl.robots())
  property timeStep:
    def __get__(self):
      return self.impl.dt()

cdef QPSolver QPSolverFromPtr(c_mc_solver.QPSolver * p):
    cdef QPSolver ret = QPSolver(None, 0, skip_alloc = True)
    ret.__own_impl = False
    ret.impl = p
    return ret

cdef QPSolver QPSolverFromRef(c_mc_solver.QPSolver & p):
    return QPSolverFromPtr(&p)
