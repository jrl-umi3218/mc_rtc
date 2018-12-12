# distutils: language = c++

cimport mc_tasks.force.c_force as c_force

cimport mc_rbdyn.mc_rbdyn as mc_rbdyn
cimport mc_tasks.mc_tasks as mc_tasks

from mc_tasks.mc_tasks cimport MetaTask

cimport sva.sva as sva

from cython.operator cimport preincrement as preinc
from cython.operator cimport dereference as deref
from libcpp.map cimport map as cppmap
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

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

