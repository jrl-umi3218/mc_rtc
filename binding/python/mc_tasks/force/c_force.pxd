#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

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

cdef extern from "<mc_tasks/AdmittanceTask.h>" namespace "mc_tasks::force":
  cdef cppclass AdmittanceTask(c_mc_tasks.SurfaceTransformTask):
    AdmittanceTask() # Does not exist but fix an error on Cython 0.2
    AdmittanceTask(const string &,
                   const c_mc_rbdyn.Robots &,
                   unsigned int, double, double)
    c_sva.PTransformd targetPose()
    void targetPose(const c_sva.PTransformd &)
    void targetWrench(const c_sva.ForceVecd &)
    const c_sva.ForceVecd & targetWrench()


cdef extern from "<mc_tasks/ComplianceTask.h>" namespace "mc_tasks::force":
  pair[double, double] defaultFGain
  pair[double, double] defaultTGain

  cdef cppclass ComplianceTask(c_mc_tasks.MetaTask):
    ComplianceTask(const c_mc_rbdyn.Robots &,
                   unsigned int, const string &,
                   double, double, double, double, double,
                   pair[double, double], pair[double, double])
    void setTargetWrench(const c_sva.ForceVecd&)

cdef extern from "<mc_tasks/DampingTask.h>" namespace "mc_tasks::force":
  cdef cppclass DampingTask(AdmittanceTask):
    DampingTask() # Does not exist but fix an error on Cython 0.2
    DampingTask(const string &,
                const c_mc_rbdyn.Robots &, unsigned int,
                double, double)

cdef extern from "<mc_tasks/CoPTask.h>" namespace "mc_tasks::force":
  cdef cppclass CoPTask(DampingTask):
    CoPTask(const string &,
            const c_mc_rbdyn.Robots &, unsigned int,
            double, double)
    c_eigen.Vector2d measuredCoP()
    c_eigen.Vector3d measuredCoPW()
    void setZeroTargetWrench()
    c_eigen.Vector2d targetCoP()
    c_eigen.Vector3d targetCoPW()
    void targetCoP(const c_eigen.Vector2d &)
    c_eigen.Vector3d targetForce()
    void targetForce(const c_eigen.Vector3d &)
