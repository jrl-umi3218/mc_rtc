#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from eigen.c_eigen import *
from sva.c_sva cimport *
from rbdyn.c_rbdyn cimport *
cimport sch.c_sch as sch
cimport tasks.qp.c_qp as c_qp
from mc_rbdyn.c_mc_rbdyn cimport Robots, Collision
cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_tasks.c_mc_tasks as c_mc_tasks

from libcpp.map cimport map as cppmap
from libcpp.pair cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef extern from "<memory>" namespace "std" nogil:
  cdef cppclass shared_ptr[T]:
    T* get()

cdef extern from "<array>" namespace "std" nogil:
  cdef cppclass array[T, N]:
    array() except +
    T& operator[](size_t)

cdef extern from *:
  ctypedef int three "3"
  ctypedef array[double, three] array3d

cdef extern from "<mc_solver/ConstraintSet.h>" namespace "mc_solver":
  cdef cppclass ConstraintSet:
    ConstraintSet()

cdef extern from "<mc_solver/ContactConstraint.h>" namespace "mc_solver":
  ctypedef enum ContactConstraintContactType "mc_solver::ContactConstraint::ContactType":
    ContactTypeAcceleration "mc_solver::ContactConstraint::Acceleration"
    ContactTypeVelocity "mc_solver::ContactConstraint::Velocity"
    ContactTypePosition "mc_solver::ContactConstraint::Position"

  cdef cppclass ContactConstraint(ConstraintSet):
    ContactConstraint(double, ContactConstraintContactType)

cdef extern from "<mc_solver/KinematicsConstraint.h>" namespace "mc_solver":
  cdef cppclass KinematicsConstraint(ConstraintSet):
    KinematicsConstraint()
    KinematicsConstraint(const Robots&, unsigned int, double)
    KinematicsConstraint(const Robots&, unsigned int, double, const array3d&, double)

cdef extern from "<mc_solver/DynamicsConstraint.h>" namespace "mc_solver":
  cdef cppclass DynamicsConstraint(KinematicsConstraint):
    DynamicsConstraint(const Robots&, unsigned int, double, cppbool)
    DynamicsConstraint(const Robots&, unsigned int, double, const array3d&, double, cppbool)

cdef extern from "<mc_solver/CollisionsConstraint.h>" namespace "mc_solver":
  cdef double CollisionsConstraintDefaultDampingOffset "mc_solver::CollisionsConstraint::defaultDampingOffset"

  cdef cppclass CollisionsConstraint(ConstraintSet):
    CollisionsConstraint(const Robots&, unsigned int, unsigned int, double)

    cppbool removeCollision(const QPSolver&, string, string)
    cppbool removeCollisionByBody(const QPSolver&, string, string)
    void addCollision(const QPSolver&, const Collision&) except+
    void addCollisions(const QPSolver& robots, const vector[Collision]&)
    void reset()

    unsigned int r1Index
    unsigned int r2Index
    vector[Collision] cols

cdef extern from "<mc_solver/TasksQPSolver.h>" namespace "mc_solver":
  cdef cppclass QPSolver:
    void addConstraintSet(const ConstraintSet&)
    void removeConstraintSet(const ConstraintSet&)
    const vector[c_mc_rbdyn.Contact] & contacts()
    void setContacts(const vector[c_mc_rbdyn.Contact]&)
    void addTask(c_qp.Task *)
    void addTask(c_mc_tasks.MetaTask *)
    void removeTask(c_mc_tasks.MetaTask *)
    cppbool run()

    Robots& robots()

    double dt()

  cdef cppclass TasksQPSolver(QPSolver):
    TasksQPSolver(shared_ptr[Robots], double)

cdef extern from "mc_solver_wrapper.hpp" namespace "mc_solver":
  cdef cppclass ContactConstrCastResult:
    c_qp.ContactAccConstr * acc
    c_qp.ContactSpeedConstr * speed
    c_qp.ContactPosConstr * pos

  cdef ContactConstrCastResult get_contact_constr(ContactConstraint&)
