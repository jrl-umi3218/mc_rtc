#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from eigen.c_eigen import *
from sva.c_sva cimport *
from rbdyn.c_rbdyn cimport *
cimport sch.c_sch as sch
cimport tasks.qp.c_qp as c_qp
from mc_rbdyn.c_mc_rbdyn cimport *
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
    ContactConstraint(double, ContactConstraintContactType, cppbool)
    #shared_ptr[c_qp.ContactConstr] contactConstr
    shared_ptr[c_qp.PositiveLambda] posLambdaConstr

cdef extern from "<mc_solver/KinematicsConstraint.h>" namespace "mc_solver":
  cdef cppclass KinematicsConstraint(ConstraintSet):
    KinematicsConstraint()
    KinematicsConstraint(const Robots&, unsigned int, double)
    KinematicsConstraint(const Robots&, unsigned int, double, const array3d&, double)
    shared_ptr[c_qp.JointLimitsConstr] jointLimitsConstr
    shared_ptr[c_qp.DamperJointLimitsConstr] damperJointLimitsConstr

cdef extern from "<mc_solver/DynamicsConstraint.h>" namespace "mc_solver":
  cdef cppclass DynamicsConstraint(KinematicsConstraint):
    DynamicsConstraint(const Robots&, unsigned int, double, cppbool)
    DynamicsConstraint(const Robots&, unsigned int, double, const array3d&, double, cppbool)
    shared_ptr[c_qp.MotionConstr] motionConstr

cdef extern from "<mc_solver/CollisionsConstraint.h>" namespace "mc_solver":
  cdef double CollisionsConstraintDefaultDampingOffset "mc_solver::CollisionsConstraint::defaultDampingOffset"

  cdef cppclass CollisionsConstraint(ConstraintSet):
    CollisionsConstraint(const Robots&, unsigned int, unsigned int, double)

    cppbool removeCollision(const QPSolver&, string, string)
    cppbool removeCollisionByBody(const QPSolver&, string, string)
    void addCollision(const QPSolver&, const Collision&) except+
    void addCollisions(const QPSolver& robots, const vector[Collision]&)
    void reset()

    shared_ptr[c_qp.CollisionConstr] collConstr
    unsigned int r1Index
    unsigned int r2Index
    vector[Collision] cols

  cdef cppclass RobotEnvCollisionsConstraint(ConstraintSet):
    RobotEnvCollisionsConstraint(const Robots&, double)

    cppbool removeEnvCollision(QPSolver&, string, string)
    cppbool removeEnvCollisionByBody(QPSolver&, string, string)
    cppbool removeSelfCollision(QPSolver&, string, string)
    void addEnvCollision(QPSolver&, const Collision&)
    void addSelfCollision(QPSolver&, const Collision&)
    void setEnvCollisions(QPSolver&, const vector[Contact]&, const vector[Collision]&)
    void setSelfCollisions(QPSolver&, const vector[Contact]&, const vector[Collision]&)

    CollisionsConstraint selfCollConstrMng
    CollisionsConstraint envCollConstrMng

cdef extern from "<mc_solver/QPSolver.h>" namespace "mc_solver":
  cdef cppclass QPSolver:
    QPSolver(shared_ptr[Robots], double)

    void addConstraintSet(const ConstraintSet&)
    void removeConstraintSet(const ConstraintSet&)
    const vector[Contact] & contacts()
    void setContacts(const vector[Contact]&)
    void addTask(c_qp.Task *)
    void addTask(c_mc_tasks.MetaTask *)
    void removeTask(c_qp.Task *)
    void removeTask(c_mc_tasks.MetaTask *)
    void addConstraint[T](c_qp.ConstraintFunction[T] *)
    void removeConstraint[T](c_qp.ConstraintFunction[T] *)
    void updateConstrSize()
    void updateNrVars()
    cppbool run()

    pair[int, const c_qp.BilateralContact&] contactById(c_qp.ContactId)

    VectorXd lambdaVec(int)

    Robots& robots()

    double dt()

    c_qp.QPSolver solver

  cdef cppclass foo[T](QPSolver, PTransform[T]):
    pass

cdef extern from "mc_solver_wrapper.hpp" namespace "mc_solver":
  cdef cppclass ContactConstrCastResult:
    c_qp.ContactAccConstr * acc
    c_qp.ContactSpeedConstr * speed
    c_qp.ContactPosConstr * pos

  cdef ContactConstrCastResult get_contact_constr(ContactConstraint&)
