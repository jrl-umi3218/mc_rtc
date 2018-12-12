cimport eigen.c_eigen as c_eigen
cimport sva.c_sva as c_sva
cimport rbdyn.c_rbdyn as c_rbdyn
cimport sch.c_sch as c_sch
cimport tasks.qp.c_qp as c_qp
cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_solver.c_mc_solver as c_mc_solver
cimport mc_tasks.c_mc_tasks as c_mc_tasks

from libcpp.map cimport map as cppmap
from libcpp.pair cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef extern from "<mc_tasks/ComplianceTask.h>" namespace "mc_tasks::force":
  pair[double, double] defaultFGain
  pair[double, double] defaultTGain

  cdef cppclass ComplianceTask(c_mc_tasks.MetaTask):
    ComplianceTask(const c_mc_rbdyn.Robots &,
                   unsigned int, const string &,
                   double, double, double, double, double,
                   pair[double, double], pair[double, double])
    void setTargetWrench(const c_sva.ForceVecd&)
