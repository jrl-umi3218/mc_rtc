# distutils: language = c++

#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_tasks.force.c_force as c_force

cimport mc_rbdyn.mc_rbdyn as mc_rbdyn
cimport mc_tasks.mc_tasks as mc_tasks

from mc_tasks.mc_tasks cimport MetaTask, SurfaceTransformTask

cimport sva.sva as sva

cimport eigen.eigen as eigen

from cython.operator cimport preincrement as preinc
from cython.operator cimport dereference as deref
from libcpp.map cimport map as cppmap
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef class AdmittanceTask(SurfaceTransformTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.adm_impl
      self.adm_impl = self.impl = self.ttg_base = self.mt_base = NULL
  def __ctor__(self, surfaceName, mc_rbdyn.Robots robots, robotIndex,
               stiffness = 5.0, weight = 1000.0):
    if isinstance(surfaceName, unicode):
      surfaceName = surfaceName.encode(u'ascii')
    self.__own_impl = True
    self.adm_impl = self.impl = self.ttg_base = self.mt_base = new c_force.AdmittanceTask(surfaceName, deref(robots.impl), robotIndex, stiffness, weight)
  def __cinit__(self, *args, skip_alloc = False, **kwargs):
    if skip_alloc:
      if len(args) + len(kwargs) > 0:
        raise TypeError("Cannot pass skip_alloc = True and other arguments to AdmittanceTask ctor")
      self.__own_impl = False
      self.adm_impl = self.impl = self.ttg_base = self.mt_base = NULL
    elif len(args) >= 3:
      self.__ctor__(*args, **kwargs)
    else:
      raise TypeError("Not enough arguments passed to AdmittanceTask ctor")
  def targetPose(self, pos = None):
    if pos is None:
      return sva.PTransformdFromC(self.adm_impl.targetPose())
    else:
      if isinstance(pos, sva.PTransformd):
        self.adm_impl.targetPose(deref((<sva.PTransformd>pos).impl))
      else:
        self.targetPose(sva.PTransformd(pos))
  def targetWrench(self, wrench = None):
    if wrench is None:
      return sva.ForceVecdFromC(self.adm_impl.targetWrench())
    else:
      if isinstance(wrench, sva.ForceVecd):
        self.adm_impl.targetWrench(deref((<sva.ForceVecd>wrench).impl))
      else:
        self.targetWrench(sva.ForceVecd(wrench))

cdef class ComplianceTask(MetaTask):
  defaultFGain = c_force.defaultFGain
  defaultTGain = c_force.defaultTGain
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, mc_rbdyn.Robots robots, robotIndex, body,
                     timestep, stiffness = 5.0, weight = 1000.0,
                     forceThresh = 3., torqueThresh = 1.,
                     forceGain = defaultFGain, torqueGain = defaultTGain):
    if isinstance(body, unicode):
      body = body.encode(u'ascii')
    self.__own_impl = True
    self.impl = self.mt_base = new c_force.ComplianceTask(deref(robots.impl), robotIndex, body, timestep, stiffness, weight, forceThresh, torqueThresh, forceGain, torqueGain)
  def __cinit__(self, *args, skip_alloc = False, **kwargs):
    if skip_alloc:
      if len(args) + len(kwargs) > 0:
        raise TypeError("Cannot pass skip_alloc = True and other arguments to ComplianceTask ctor")
      self.__own_impl = False
      self.impl = self.mt_base = NULL
    elif len(args) >= 4:
      self.__ctor__(*args, **kwargs)
    else:
      raise TypeError("Not enough arguments passed to ComplianceTask ctor")

  def setTargetWrench(self, wrench):
    if isinstance(wrench, sva.ForceVecd):
      self.impl.setTargetWrench(deref((<sva.ForceVecd>(wrench)).impl))
    else:
      self.setTargetWrench(sva.ForceVecd(wrench))

cdef class DampingTask(AdmittanceTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.adm_impl
      self.damping_impl = self.adm_impl = self.impl = self.ttg_base = self.mt_base = NULL
  def __ctor__(self, surfaceName, mc_rbdyn.Robots robots, robotIndex,
               stiffness = 5.0, weight = 1000.0):
    if isinstance(surfaceName, unicode):
      surfaceName = surfaceName.encode(u'ascii')
    self.__own_impl = True
    self.damping_impl = self.adm_impl = self.impl = self.ttg_base = self.mt_base = new c_force.DampingTask(surfaceName, deref(robots.impl), robotIndex, stiffness, weight)
  def __cinit__(self, *args, skip_alloc = False, **kwargs):
    if skip_alloc:
      if len(args) + len(kwargs) > 0:
        raise TypeError("Cannot pass skip_alloc = True and other arguments to DampingTask ctor")
      self.__own_impl = False
      self.damping_impl = self.adm_impl = self.impl = self.ttg_base = self.mt_base = NULL
    elif len(args) >= 3:
      self.__ctor__(*args, **kwargs)
    else:
      raise TypeError("Not enough arguments passed to DampingTask ctor")

cdef class CoPTask(DampingTask):
  def __dealloc__(self):
    if self.__own_impl:
      del self.adm_impl
      self.cop_impl = self.damping_impl = self.adm_impl = self.impl = self.ttg_base = self.mt_base = NULL
  def __ctor__(self, surfaceName, mc_rbdyn.Robots robots, robotIndex,
               stiffness = 5.0, weight = 1000.0):
    if isinstance(surfaceName, unicode):
      surfaceName = surfaceName.encode(u'ascii')
    self.__own_impl = True
    self.cop_impl = self.damping_impl = self.adm_impl = self.impl = self.ttg_base = self.mt_base = new c_force.CoPTask(surfaceName, deref(robots.impl), robotIndex, stiffness, weight)
  def __cinit__(self, *args, skip_alloc = False, **kwargs):
    if skip_alloc:
      if len(args) + len(kwargs) > 0:
        raise TypeError("Cannot pass skip_alloc = True and other arguments to CoPTask ctor")
      self.__own_impl = False
      self.cop_impl = self.damping_impl = self.adm_impl = self.impl = self.ttg_base = self.mt_base = NULL
    elif len(args) >= 3:
      self.__ctor__(*args, **kwargs)
    else:
      raise TypeError("Not enough arguments passed to CoPTask ctor")
  def measuredCoP(self):
    return eigen.Vector2dFromC(self.cop_impl.measuredCoP())
  def measuredCoPW(self):
    return eigen.Vector3dFromC(self.cop_impl.measuredCoPW())
  def setZeroTargetWrench(self):
    self.cop_impl.setZeroTargetWrench()
  def targetCoP(self, eigen.Vector2d target = None):
    if target is None:
      return eigen.Vector2dFromC(self.cop_impl.targetCoP())
    else:
      self.cop_impl.targetCoP(target.impl)
  def targetCoPW(self):
    return eigen.Vector3dFromC(self.cop_impl.targetCoPW())
  def targetForce(self, eigen.Vector3d force = None):
    if force is None:
      return eigen.Vector3dFromC(self.cop_impl.targetForce())
    else:
      self.cop_impl.targetForce(force.impl)
