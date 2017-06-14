# distutils: language = c++

cimport mc_tasks.c_mc_tasks as c_mc_tasks

cimport eigen.c_eigen as c_eigen
cimport eigen.eigen as eigen

cimport sva.sva as sva

cimport tasks.qp.c_qp as c_qp
cimport tasks.qp.qp as qp

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_rbdyn.mc_rbdyn as mc_rbdyn

cimport mc_solver.c_mc_solver as c_mc_solver
cimport mc_solver.mc_solver as mc_solver

from cython.operator cimport preincrement as preinc
from cython.operator cimport dereference as deref
from libcpp.map cimport map as cppmap
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef class MetaTask(object):
  def __cinit__(self):
    self.mt_base = NULL
  def reset(self):
    assert(self.mt_base)
    self.mt_base.reset()
  def dimWeight(self, eigen.VectorXd dimW = None):
    assert(self.mt_base)
    if dimW is None:
      return eigen.VectorXdFromC(self.mt_base.dimWeight())
    else:
      self.mt_base.dimWeight(dimW.impl)
  def selectActiveJoints(self, mc_solver.QPSolver solver, joints):
    assert(self.mt_base)
    self.mt_base.selectActiveJoints(deref(solver.impl), joints)
  def selectUnactiveJoints(self, mc_solver.QPSolver solver, joints):
    assert(self.mt_base)
    self.mt_base.selectUnactiveJoints(deref(solver.impl), joints)
  def resetJointsSelector(self, mc_solver.QPSolver solver):
    assert(self.mt_base)
    self.mt_base.resetJointsSelector(deref(solver.impl))
  def eval(self):
    assert(self.mt_base)
    return eigen.VectorXdFromC(self.mt_base.eval())
  def speed(self):
    assert(self.mt_base)
    return eigen.VectorXdFromC(self.mt_base.speed())

include "com_trajectory_task.pxi"
include "position_trajectory_task.pxi"
include "orientation_trajectory_task.pxi"
include "vector_orientation_trajectory_task.pxi"

cdef class CoMTask(_CoMTrajectoryTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, mc_rbdyn.Robots robots, robotIndex,
               stiffness = 2.0, weight = 500.0):
    self.__own_impl = True
    self.impl = self.ttg_base = self.mt_base = new c_mc_tasks.CoMTask(deref(robots.impl), robotIndex, stiffness, weight)
  def __cinit__(self, *args, skip_alloc = False):
    if skip_alloc:
      self.__own_impl = False
      self.impl = self.ttg_base = self.mt_base = NULL
      return
    elif len(args) >= 3:
      self.__ctor__(*args)
    else:
      raise TypeError("Not enough arguments passed to CoMTask ctor")
  def com(self, eigen.Vector3d com = None):
    assert(self.impl)
    if com is None:
      return eigen.Vector3dFromC(self.impl.com())
    else:
      self.impl.com(com.impl)
  def move_com(self, eigen.Vector3d d):
    assert(self.impl)
    self.impl.move_com(d.impl)

cdef class PositionTask(_PositionTrajectoryTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, bodyName, mc_rbdyn.Robots robots,
                robotIndex, stiffness = 2.0, weight = 500.0):
    self.__own_impl = True
    self.impl = self.ttg_base = self.mt_base = new c_mc_tasks.PositionTask(bodyName, deref(robots.impl), robotIndex, stiffness, weight)
  def __cinit__(self, *args, skip_alloc = False):
    if skip_alloc:
      self.__own_impl = False
      self.impl = self.ttg_base = self.mt_base = NULL
      return
    elif len(args) >= 3:
      self.__ctor__(*args)
    else:
      raise TypeError("Not enough arguments passed to PositionTask ctor")
  def position(self, eigen.Vector3d pos = None):
    assert(self.impl)
    if pos is None:
      return eigen.Vector3dFromC(self.impl.position())
    else:
      self.impl.position(pos.impl)

cdef PositionTask PositionTaskFromSharedPtr(shared_ptr[c_mc_tasks.PositionTask] ptr):
  cdef PositionTask ret = PositionTask(skip_alloc = True)
  ret.impl = ret.mt_base = ptr.get()
  return ret

cdef class OrientationTask(_OrientationTrajectoryTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, bodyName, mc_rbdyn.Robots robots,
                robotIndex, stiffness = 2.0, weight = 500.0):
    self.__own_impl = True
    self.impl = self.ttg_base = self.mt_base = new c_mc_tasks.OrientationTask(bodyName, deref(robots.impl), robotIndex, stiffness, weight)
  def __cinit__(self, *args, skip_alloc = False):
    if skip_alloc:
      self.__own_impl = False
      self.impl = self.ttg_base = self.mt_base = NULL
      return
    elif len(args) >= 3:
      self.__ctor__(*args)
    else:
      raise TypeError("Not enough arguments passed to OrientationTask ctor")
  def orientation(self, eigen.Matrix3d ori = None):
    assert(self.impl)
    if ori is None:
      return eigen.Matrix3dFromC(self.impl.orientation())
    else:
      self.impl.orientation(ori.impl)

cdef OrientationTask OrientationTaskFromSharedPtr(shared_ptr[c_mc_tasks.OrientationTask] ptr):
  cdef OrientationTask ret = OrientationTask(skip_alloc = True)
  ret.impl = ret.mt_base = ptr.get()
  return ret

cdef class VectorOrientationTask(_VectorOrientationTrajectoryTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, bodyName, eigen.Vector3d bodyVector,
               eigen.Vector3d targetVector, mc_rbdyn.Robots robots,
                robotIndex, stiffness = 2.0, weight = 500.0):
    self.__own_impl = True
    self.impl = self.ttg_base = self.mt_base = new c_mc_tasks.VectorOrientationTask(bodyName, bodyVector.impl, targetVector.impl, deref(robots.impl), robotIndex, stiffness, weight)
  def __cinit__(self, *args, skip_alloc = False):
    if skip_alloc:
      self.__own_impl = False
      self.impl = self.ttg_base = self.mt_base = NULL
      return
    elif len(args) >= 5:
      self.__ctor__(*args)
    else:
      raise TypeError("Not enough arguments passed to VectorOrientationTask ctor")
  def bodyVector(self, eigen.Vector3d ori = None):
    assert(self.impl)
    if ori is None:
      return eigen.Vector3dFromC(self.impl.bodyVector())
    else:
      self.impl.bodyVector(ori.impl)

cdef class EndEffectorTask(MetaTask):
  def __dealloc__(self):
    if self.__own_impl and type(self) is EndEffectorTask:
      del self.impl
  def __ctor__(self, bodyName, mc_rbdyn.Robots robots,
                robotIndex, stiffness = 2.0, weight = 1000.0):
    self.__own_impl = True
    self.impl = self.mt_base = new c_mc_tasks.EndEffectorTask(bodyName, deref(robots.impl), robotIndex, stiffness, weight)
  def __cinit__(self, *args, skip_alloc = False):
    if skip_alloc:
      self.__own_impl = False
      self.impl = self.mt_base = NULL
      return
    elif len(args) >= 3:
      self.__ctor__(*args)
    else:
      raise TypeError("Not enough arguments passed to EndEffectorTask ctor")
  def add_ef_pose(self, sva.PTransformd pt):
    assert(self.impl)
    self.impl.add_ef_pose(deref(pt.impl))
  def set_ef_pose(self, sva.PTransformd pt):
    assert(self.impl)
    self.impl.set_ef_pose(deref(pt.impl))
  def get_ef_pose(self):
    assert(self.impl)
    return sva.PTransformdFromC(self.impl.get_ef_pose())
  property positionTask:
    def __get__(self):
      assert(self.impl)
      return PositionTaskFromSharedPtr(self.impl.positionTask)
  property orientationTask:
    def __get__(self):
      assert(self.impl)
      return OrientationTaskFromSharedPtr(self.impl.orientationTask)

cdef class RelativeEndEffectorTask(EndEffectorTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.rel_impl
  def __ctor__(self, bodyName, mc_rbdyn.Robots robots,
                robotIndex, relBodyIdx = 0, stiffness = 2.0, weight = 1000.0):
    self.__own_impl = True
    self.rel_impl = self.impl = self.mt_base = new c_mc_tasks.RelativeEndEffectorTask(bodyName, deref(robots.impl), robotIndex, relBodyIdx, stiffness, weight)
  def __cinit__(self, *args, skip_alloc = False):
    if skip_alloc:
      self.__own_impl = False
      self.rel_impl = self.impl = self.mt_base = NULL
      return
    elif len(args) >= 3:
      self.__ctor__(*args)
    else:
      raise TypeError("Not enough arguments passed to RelativeEndEffectorTask ctor")
