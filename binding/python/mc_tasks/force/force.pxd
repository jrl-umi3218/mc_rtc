#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_tasks.force.c_force as c_force
from mc_tasks.mc_tasks cimport MetaTask, SurfaceTransformTask

from libcpp cimport bool as cppbool

cdef class AdmittanceTask(SurfaceTransformTask):
  cdef c_force.AdmittanceTask * adm_impl

cdef class ComplianceTask(MetaTask):
  cdef c_force.ComplianceTask * impl
  cdef cppbool __own_impl

cdef class DampingTask(AdmittanceTask):
  cdef c_force.DampingTask * damping_impl

cdef class CoPTask(DampingTask):
  cdef c_force.CoPTask * cop_impl
