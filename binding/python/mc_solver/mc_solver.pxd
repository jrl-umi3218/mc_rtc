cimport c_mc_solver

from libcpp cimport bool as cppbool

cdef class ConstraintSet(object):
  cdef c_mc_solver.ConstraintSet * cs_base

cdef class ContactConstraint(ConstraintSet):
  cdef c_mc_solver.ContactConstraint * impl
  cdef cppbool __own_impl

cdef ContactConstraint ContactConstraintFromPtr(c_mc_solver.ContactConstraint *)

cdef class KinematicsConstraint(ConstraintSet):
  cdef c_mc_solver.KinematicsConstraint * impl
  cdef cppbool __own_impl

cdef KinematicsConstraint KinematicsConstraintFromPtr(c_mc_solver.KinematicsConstraint *)

cdef class DynamicsConstraint(KinematicsConstraint):
  cdef c_mc_solver.DynamicsConstraint * d_impl

cdef DynamicsConstraint DynamicsConstraintFromPtr(c_mc_solver.DynamicsConstraint *)

cdef class CollisionsConstraint(ConstraintSet):
  cdef c_mc_solver.CollisionsConstraint * impl
  cdef cppbool __own_impl

cdef CollisionsConstraint CollisionsConstraintFromPtr(c_mc_solver.CollisionsConstraint*)

cdef class RobotEnvCollisionsConstraint(ConstraintSet):
  cdef c_mc_solver.RobotEnvCollisionsConstraint * impl
  cdef cppbool __own_impl

cdef RobotEnvCollisionsConstraint RobotEnvCollisionsConstraintFromPtr(c_mc_solver.RobotEnvCollisionsConstraint*)

cdef class QPSolver(object):
  cdef c_mc_solver.QPSolver * impl
  cdef cppbool __own_impl

cdef QPSolver QPSolverFromPtr(c_mc_solver.QPSolver*)

cdef QPSolver QPSolverFromRef(c_mc_solver.QPSolver&)
