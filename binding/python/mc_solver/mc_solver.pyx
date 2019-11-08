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
      self.impl = self.cs_base = new c_mc_solver.ContactConstraint(timeStep, <c_mc_solver.ContactConstraintContactType>cType, dynamics)
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
  property posLambdaConstr:
    def __get__(self):
      if self.impl.posLambdaConstr.get() != NULL:
        return qp.PositiveLambdaFromPtr(self.impl.posLambdaConstr.get())
      else:
        return None

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
  property jointLimitsConstr:
    def __get__(self):
      if self.impl.jointLimitsConstr.get() != NULL:
        return qp.JointLimitsConstrFromPtr(self.impl.jointLimitsConstr.get())
      else:
        return None
  property damperJointLimitsConstr:
    def __get__(self):
      cdef c_qp.DamperJointLimitsConstr * p = self.impl.damperJointLimitsConstr.get()
      if p != NULL:
        return qp.DamperJointLimitsConstrFromPtr(p)
      else:
        return None

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
  property motionConstr:
    def __get__(self):
      cdef c_qp.MotionConstr * p = self.d_impl.motionConstr.get()
      if p != NULL:
        return qp.MotionConstrFromPtr(p)
      else:
        return None

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
  property collConstr:
    def __get__(self):
      return qp.CollisionConstrFromPtr(self.impl.collConstr.get())
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

cdef class RobotEnvCollisionsConstraint(ConstraintSet):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, mc_rbdyn.Robots robots, timeStep, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = self.cs_base = new c_mc_solver.RobotEnvCollisionsConstraint(deref(robots.impl), timeStep)
  def removeEnvCollision(self, QPSolver solver, b1, b2):
    if isinstance(b1, unicode):
        b1 = b1.encode(u'ascii')
    if isinstance(b2, unicode):
        b2 = b2.encode(u'ascii')
    return self.impl.removeEnvCollision(deref(solver.impl), b1, b2)
  def removeEnvCollisionByBody(self, QPSolver solver, b1, b2):
    if isinstance(b1, unicode):
        b1 = b1.encode(u'ascii')
    if isinstance(b2, unicode):
        b2 = b2.encode(u'ascii')
    return self.impl.removeEnvCollisionByBody(deref(solver.impl), b1, b2)
  def removeSelfCollision(self, QPSolver solver, b1, b2):
    if isinstance(b1, unicode):
        b1 = b1.encode(u'ascii')
    if isinstance(b2, unicode):
        b2 = b2.encode(u'ascii')
    return self.impl.removeSelfCollision(deref(solver.impl), b1, b2)
  def addEnvCollision(self, QPSolver solver, mc_rbdyn.Collision col):
    self.impl.addEnvCollision(deref(solver.impl), col.impl)
  def addSelfCollision(self, QPSolver solver, mc_rbdyn.Collision col):
    self.impl.addSelfCollision(deref(solver.impl), col.impl)
  def setEnvCollisions(self, QPSolver solver, contacts, cols_in):
    cdef vector[c_mc_rbdyn.Collision] cols = vector[c_mc_rbdyn.Collision]()
    for c in cols_in:
      cols.push_back((<mc_rbdyn.Collision>(c)).impl)
    self.impl.setEnvCollisions(deref(solver.impl), deref(mc_rbdyn.ContactVector(contacts).v), cols)
  def setSelfCollisions(self, QPSolver solver, contacts, cols_in):
    cdef vector[c_mc_rbdyn.Collision] cols = vector[c_mc_rbdyn.Collision]()
    for c in cols_in:
      cols.push_back((<mc_rbdyn.Collision>(c)).impl)
    self.impl.setSelfCollisions(deref(solver.impl), deref(mc_rbdyn.ContactVector(contacts).v), cols)
  property selfCollConstrMng:
    def __get__(self):
      return CollisionsConstraintFromPtr(&self.impl.selfCollConstrMng)
  property envCollConstrMng:
    def __get__(self):
      return CollisionsConstraintFromPtr(&self.impl.envCollConstrMng)

cdef RobotEnvCollisionsConstraint RobotEnvCollisionsConstraintFromPtr(c_mc_solver.RobotEnvCollisionsConstraint * p):
    cdef RobotEnvCollisionsConstraint ret = RobotEnvCollisionsConstraint(None, 0, skip_alloc = True)
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
      self.impl = new c_mc_solver.QPSolver(robots.impl, timeStep)
  def addConstraintSet(self, ConstraintSet cs):
    self.impl.addConstraintSet(deref(cs.cs_base))
  def addConstraint(self, qp.Constraint c):
    if isinstance(c, qp.Equality):
      self.impl.addConstraint((<qp.Equality>c).cf_base)
    elif isinstance(c, qp.Inequality):
      self.impl.addConstraint((<qp.Inequality>c).cf_base)
    elif isinstance(c, qp.GenInequality):
      self.impl.addConstraint((<qp.GenInequality>c).cf_base)
    elif isinstance(c, qp.Bound):
      self.impl.addConstraint((<qp.Bound>c).cf_base)
    else:
      raise TypeError("Cannot handle this type of native constraint")
  def removeConstraint(self, qp.Constraint c):
    if isinstance(c, qp.Equality):
      self.impl.removeConstraint((<qp.Equality>c).cf_base)
    elif isinstance(c, qp.Inequality):
      self.impl.removeConstraint((<qp.Inequality>c).cf_base)
    elif isinstance(c, qp.GenInequality):
      self.impl.removeConstraint((<qp.GenInequality>c).cf_base)
    elif isinstance(c, qp.Bound):
      self.impl.removeConstraint((<qp.Bound>c).cf_base)
    else:
      raise TypeError("Cannot handle this type of native constraint")
  def removeConstraintSet(self, ConstraintSet cs):
    self.impl.removeConstraintSet(deref(cs.cs_base))
  def updateConstrSize(self):
    self.impl.updateConstrSize()
  def updateNrVars(self):
    self.impl.updateNrVars()
  def setContacts(self, contacts):
    self.impl.setContacts(deref(mc_rbdyn.ContactVector(contacts).v))
  def contacts(self):
    ret = []
    for i in range(self.impl.contacts().size()):
      ret.append(mc_rbdyn.ContactFromC(self.impl.contacts()[i]))
    return ret
  def addTask(self, task):
    if isinstance(task, qp.Task):
      self.impl.addTask((<qp.Task>task).base)
    elif isinstance(task, mc_tasks.MetaTask):
      self.impl.addTask((<mc_tasks.MetaTask>task).mt_base)
    else:
      raise TypeError("Cannot add a Task of this type")
  def removeTask(self, task):
    if isinstance(task, qp.Task):
      self.impl.removeTask((<qp.Task>task).base)
    elif isinstance(task, mc_tasks.MetaTask):
      self.impl.removeTask((<mc_tasks.MetaTask>task).mt_base)
  def updateConstrSize(self):
    self.impl.updateConstrSize()
  def updateNrVars(self):
    self.impl.updateNrVars()
  def run(self):
    return self.impl.run()
  def contactById(self, qp.ContactId cId):
    return BilateralContactFromC(self.impl.contactById(cId.impl).second)
  def lambdaVec(self, cIndex):
    return eigen.VectorXdFromC(self.impl.lambdaVec(cIndex))
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
