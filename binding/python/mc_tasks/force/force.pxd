cimport mc_tasks.force.c_force as c_force
from mc_tasks.mc_tasks cimport MetaTask

from libcpp cimport bool as cppbool

cdef class ComplianceTask(MetaTask):
  cdef c_force.ComplianceTask * impl
  cdef cppbool __own_impl

