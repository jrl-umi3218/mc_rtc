# distutils: language = c++

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

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
from libcpp.pair cimport pair
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
  property name:
    def __get__(self):
      assert(self.mt_base)
      return self.mt_base.name()
    def __set__(self, value):
      assert(self.mt_base)
      if isinstance(value, unicode):
        value = value.encode(u'ascii')
      self.mt_base.name(value)

include "com_trajectory_task.pxi"
include "position_trajectory_task.pxi"
include "orientation_trajectory_task.pxi"
include "vector_orientation_trajectory_task.pxi"

def genericTTGNull(AnyTTG self):
  self.impl = self.ttg_base = self.mt_base = NULL

def genericMTNull(AnyTask self):
  self.impl = self.mt_base = NULL

def genericInit(AnyTask self, size, name, *args, skip_alloc=False, **kwargs):
  if skip_alloc:
    if len(args) + len(kwargs) > 0:
      raise TypeError("Cannot pass skip_alloc=True and other arguments to {0} ctor".format(name))
    self.__own_impl = False
    if AnyTask in AnyTTG:
      genericTTGNull(self)
    else:
      genericMTNull(self)
    return
  elif len(args) >= size:
    self.__ctor__(*args, **kwargs)
  else:
    raise TypeError("Not enough arguments passed to {0} ctor".format(name))

cdef class CoMTask(_CoMTrajectoryTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, mc_rbdyn.Robots robots, robotIndex,
               stiffness = 2.0, weight = 500.0):
    self.__own_impl = True
    self.impl = self.ttg_base = self.mt_base = new c_mc_tasks.CoMTask(deref(robots.impl), robotIndex, stiffness, weight)

  def __cinit__(self, *args, **kwargs):
    genericInit[CoMTask](self, 2, 'CoMTask', *args, **kwargs)

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
    if isinstance(bodyName, unicode):
      bodyName = bodyName.encode(u'ascii')
    self.__own_impl = True
    self.impl = self.ttg_base = self.mt_base = new c_mc_tasks.PositionTask(bodyName, deref(robots.impl), robotIndex, stiffness, weight)

  def __cinit__(self, *args, **kwargs):
    genericInit[PositionTask](self, 3, 'PositionTask', *args, **kwargs)

  def position(self, eigen.Vector3d pos = None):
    assert(self.impl)
    if pos is None:
      return eigen.Vector3dFromC(self.impl.position())
    else:
      self.impl.position(pos.impl)

cdef PositionTask PositionTaskFromSharedPtr(shared_ptr[c_mc_tasks.PositionTask] ptr):
  cdef PositionTask ret = PositionTask(skip_alloc = True)
  ret.impl = ret.mt_base = ret.ttg_base = ptr.get()
  return ret

cdef class OrientationTask(_OrientationTrajectoryTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, bodyName, mc_rbdyn.Robots robots,
                robotIndex, stiffness = 2.0, weight = 500.0):
    if isinstance(bodyName, unicode):
      bodyName = bodyName.encode(u'ascii')
    self.__own_impl = True
    self.impl = self.ttg_base = self.mt_base = new c_mc_tasks.OrientationTask(bodyName, deref(robots.impl), robotIndex, stiffness, weight)
  def __cinit__(self, *args, **kwargs):
    genericInit[OrientationTask](self, 3, 'OrientationTask', *args, **kwargs)
  def orientation(self, eigen.Matrix3d ori = None):
    assert(self.impl)
    if ori is None:
      return eigen.Matrix3dFromC(self.impl.orientation())
    else:
      self.impl.orientation(ori.impl)

cdef OrientationTask OrientationTaskFromSharedPtr(shared_ptr[c_mc_tasks.OrientationTask] ptr):
  cdef OrientationTask ret = OrientationTask(skip_alloc = True)
  ret.impl = ret.mt_base = ret.ttg_base = ptr.get()
  return ret

cdef class VectorOrientationTask(_VectorOrientationTrajectoryTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, bodyName, eigen.Vector3d bodyVector,
               eigen.Vector3d targetVector, mc_rbdyn.Robots robots,
                robotIndex, stiffness = 2.0, weight = 500.0):
    if isinstance(bodyName, unicode):
      bodyName = bodyName.encode(u'ascii')
    self.__own_impl = True
    self.impl = self.ttg_base = self.mt_base = new c_mc_tasks.VectorOrientationTask(bodyName, bodyVector.impl, targetVector.impl, deref(robots.impl), robotIndex, stiffness, weight)
  def __cinit__(self, *args, **kwargs):
    genericInit[VectorOrientationTask](self, 5, 'VectorOrientationTask', *args, **kwargs)

cdef class SurfaceTransformTask(_SurfaceTransformTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, surfaceName, mc_rbdyn.Robots robots,
          robotIndex, stiffness = 2.0, weight = 500.0):
    if isinstance(surfaceName, unicode):
      surfaceName = surfaceName.encode(u'ascii')
    self.__own_impl = True
    self.impl = self.ttg_base = self.mt_base = new c_mc_tasks.SurfaceTransformTask(surfaceName, deref(robots.impl), robotIndex, stiffness, weight)
  def __cinit__(self, *args, **kwargs):
    genericInit[SurfaceTransformTask](self, 3, 'SurfaceTransformTask', *args, **kwargs)
  def target(self, pos = None):
    if pos is None:
      return sva.PTransformdFromC(self.impl.target())
    else:
      if isinstance(pos, sva.PTransformd):
        self.impl.target(deref((<sva.PTransformd>pos).impl))
      else:
        self.target(sva.PTransformd(pos))

cdef class PairDoubleMatrix3d(object):
  cdef pair[double, c_eigen.Matrix3d] impl
  def __cinit__(self, *args):
    if len(args) == 1 and isinstance(args[0], tuple) and len(args[0]) == 2:
      tup = args[0]
      self.impl = pair[double, c_eigen.Matrix3d](tup[0], eigen.Matrix3d(tup[1]).impl)

cdef class VectorPairDoubleMatrix3d(object):
  cdef vector[pair[double, c_eigen.Matrix3d]] impl
  def __cinit__(self, *args):
    if len(args) == 1 and isinstance(args[0], list):
      for p in args[0]:
        self.impl.push_back(PairDoubleMatrix3d(p).impl)

cdef class PairDoubleVector3d(object):
  cdef pair[double, c_eigen.Vector3d] impl
  def __cinit__(self, *args):
    if len(args) == 1 and isinstance(args[0], tuple) and len(args[0]) == 2:
      tup = args[0]
      self.impl = pair[double, c_eigen.Vector3d](tup[0], eigen.Vector3d(tup[1]).impl)
      print "pair {}".format(tup[0])

cdef class VectorPairDoubleVector3d(object):
  cdef vector[pair[double, c_eigen.Vector3d]] impl
  def __cinit__(self, *args):
    if len(args) == 1 and isinstance(args[0], list):
      for p in args[0]:
        self.impl.push_back(PairDoubleVector3d(p).impl)

cdef class BSplineTrajectoryTask(_SplineTrajectoryTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, mc_rbdyn.Robots robots, robotIndex,
          surfaceName, duration, stiffness, weight, sva.PTransformd target, posWp, oriWp):
    if isinstance(surfaceName, unicode):
      surfaceName = surfaceName.encode(u'ascii')
    self.__own_impl = True
    self.impl = self.ttg_base = self.mt_base = new c_mc_tasks.BSplineTrajectoryTask(deref(robots.impl), robotIndex, surfaceName, duration, stiffness, weight, deref(target.impl))
    self.posWaypoints(posWp)
    self.oriWaypoints(oriWp)
  def __cinit__(self, *args, **kwargs):
    genericInit[BSplineTrajectoryTask](self, 9, 'BSplineTrajectoryTask', *args, **kwargs)
  def posWaypoints(self, posWp):
    assert(self.impl)
    self.impl.posWaypoints(eigen.Vector3dVector(posWp).v)
  def oriWaypoints(self, oriWp):
    assert(self.impl)
    self.impl.oriWaypoints(VectorPairDoubleMatrix3d(oriWp).impl)
  def target(self, pos = None):
    if pos is None:
      return sva.PTransformdFromC((<c_mc_tasks.SplineTrajectoryTask[c_mc_tasks.BSplineTrajectoryTask]*>self.impl).target())
    else:
      if isinstance(pos, sva.PTransformd):
        (<c_mc_tasks.SplineTrajectoryTask[c_mc_tasks.BSplineTrajectoryTask]*>self.impl).target(deref((<sva.PTransformd>pos).impl))
      else:
        self.target(sva.PTransformd(pos))
  def evalTracking(self):
    assert(self.impl)
    return eigen.VectorXdFromC(self.impl.evalTracking())
  def displaySamples(self, samples = None):
    assert(self.impl)
    if samples is None:
      return (<c_mc_tasks.SplineTrajectoryTask[c_mc_tasks.BSplineTrajectoryTask]*>self.impl).displaySamples()
    else:
      (<c_mc_tasks.SplineTrajectoryTask[c_mc_tasks.BSplineTrajectoryTask]*>self.impl).displaySamples(samples)

cdef class ExactCubicTrajectoryTask(_SplineTrajectoryTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, mc_rbdyn.Robots robots, robotIndex,
          surfaceName, duration, stiffness, weight, sva.PTransformd target):
    if isinstance(surfaceName, unicode):
      surfaceName = surfaceName.encode(u'ascii')
    self.__own_impl = True
    self.impl = self.ttg_base = self.mt_base = new c_mc_tasks.ExactCubicTrajectoryTask(deref(robots.impl), robotIndex, surfaceName, duration, stiffness, weight, deref(target.impl))
  def __cinit__(self, *args, **kwargs):
    genericInit[ExactCubicTrajectoryTask](self, 7, 'ExactCubicTrajectoryTask', *args, **kwargs)
  def posWaypoints(self, posWp):
    assert(self.impl)
    self.impl.posWaypoints(VectorPairDoubleVector3d(posWp).impl)
  def oriWaypoints(self, oriWp):
    assert(self.impl)
    self.impl.oriWaypoints(VectorPairDoubleMatrix3d(oriWp).impl)
  def constraints(self, eigen.Vector3d initVel, eigen.Vector3d initAcc, eigen.Vector3d endVel, eigen.Vector3d endAcc):
    self.impl.constraints(initVel.impl, initAcc.impl, endVel.impl, endAcc.impl)

  def target(self, pos = None):
    if pos is None:
      return sva.PTransformdFromC((<c_mc_tasks.SplineTrajectoryTask[c_mc_tasks.ExactCubicTrajectoryTask]*>self.impl).target())
    else:
      if isinstance(pos, sva.PTransformd):
        (<c_mc_tasks.SplineTrajectoryTask[c_mc_tasks.ExactCubicTrajectoryTask]*>self.impl).target(deref((<sva.PTransformd>pos).impl))
      else:
        self.target(sva.PTransformd(pos))
  def evalTracking(self):
    assert(self.impl)
    return eigen.VectorXdFromC(self.impl.evalTracking())
  def displaySamples(self, samples = None):
    assert(self.impl)
    if samples is None:
      return (<c_mc_tasks.SplineTrajectoryTask[c_mc_tasks.ExactCubicTrajectoryTask]*>self.impl).displaySamples()
    else:
      (<c_mc_tasks.SplineTrajectoryTask[c_mc_tasks.ExactCubicTrajectoryTask]*>self.impl).displaySamples(samples)

cdef class EndEffectorTask(MetaTask):
  def __dealloc__(self):
    if self.__own_impl and type(self) is EndEffectorTask:
      del self.impl
  def __ctor__(self, bodyName, mc_rbdyn.Robots robots,
                robotIndex, stiffness = 2.0, weight = 1000.0):
    if isinstance(bodyName, unicode):
      bodyName = bodyName.encode(u'ascii')
    self.__own_impl = True
    self.impl = self.mt_base = new c_mc_tasks.EndEffectorTask(bodyName, deref(robots.impl), robotIndex, stiffness, weight)
  def __cinit__(self, *args, **kwargs):
    genericInit[EndEffectorTask](self, 3, 'EndEffectorTask', *args, **kwargs)
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
      del self.impl
  def __ctor__(self, bodyName, mc_rbdyn.Robots robots,
                robotIndex, relBodyName = "", stiffness = 2.0, weight = 1000.0):
    if isinstance(bodyName, unicode):
      bodyName = bodyName.encode(u'ascii')
    if isinstance(relBodyName, unicode):
      relBodyName = relBodyName.encode(u'ascii')
    self.__own_impl = True
    self.impl = self.mt_base = new c_mc_tasks.RelativeEndEffectorTask(bodyName, deref(robots.impl), robotIndex, relBodyName, stiffness, weight)
  def __cinit__(self, *args, **kwargs):
    genericInit[RelativeEndEffectorTask](self, 3, 'RelativeEndEffectorTask', *args, **kwargs)

cdef class PostureTask(MetaTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, mc_solver.QPSolver solver, rIndex, stiffness, weight):
    self.__own_impl = True
    self.impl = self.mt_base = new c_mc_tasks.PostureTask(deref(solver.impl), rIndex, stiffness, weight)
  def __cinit__(self, *args, **kwargs):
    genericInit[PostureTask](self, 4, 'PostureTask', *args, **kwargs)
  def posture(self, target = None):
    assert(self.impl)
    if target is None:
      return self.impl.posture()
    else:
      self.impl.posture(target)
  def jointGains(self, mc_solver.QPSolver solver, jgs):
    assert(self.impl)
    self.impl.jointGains(deref(solver.impl), qp.JointGainsVector(jgs).v)
  def jointStiffness(self, mc_solver.QPSolver solver, jss):
    assert(self.impl)
    self.impl.jointStiffness(deref(solver.impl), qp.JointStiffnessVector(jss).v)
  def target(self, in_):
    assert(self.impl)
    self.impl.target(in_)
  def stiffness(self, s = None):
    assert(self.impl)
    if s is None:
      return self.impl.stiffness()
    else:
      self.impl.stiffness(s)
  def weight(self, w = None):
    assert(self.impl)
    if w is None:
      return self.impl.weight()
    else:
      self.impl.weight(w)

cdef PostureTask PostureTaskFromPtr(c_mc_tasks.PostureTask * p):
  cdef PostureTask ret = PostureTask(skip_alloc = True)
  ret.__own_impl = False
  ret.impl = ret.mt_base = p
  return ret
