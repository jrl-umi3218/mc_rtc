#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

from eigen.c_eigen cimport *
from sva.c_sva cimport *
from rbdyn.c_rbdyn cimport *
cimport sch.c_sch as sch
cimport tasks.qp.c_qp as c_qp
from mc_rbdyn.c_mc_rbdyn cimport RobotModulePtr, Robot, Robots, Collision
from mc_solver.c_mc_solver cimport *
cimport mc_observers.c_mc_observers as c_mc_observers
cimport mc_rtc.c_mc_rtc as c_mc_rtc
cimport mc_rtc.gui.c_gui as c_mc_rtc_gui
cimport mc_tasks.c_mc_tasks as c_mc_tasks

from libcpp.map cimport map as cppmap
from libcpp.pair cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool


cdef extern from "<memory>" namespace "std" nogil:
  cdef cppclass shared_ptr[T]:
    shared_ptr(T*)
    T* get()

cdef extern from "<mc_control/Contact.h>" namespace "mc_control":
  cdef cppclass Contact:
    Contact()
    Contact(const string&, const string&, const string&, const string&, double)
    Contact(const string&, const string&, const string&, const string&, double, const Vector6d)
    string r1
    string r2
    string r1Surface
    string r2Surface
    double friction
    Vector6d dof

  # Actually std::set<Contact, std::less<Contact>, Eigen::aligned_allocator<Contact>> but Cython only know std::set<T>
  cdef cppclass ContactSet:
    cppclass iterator:
      Contact& operator*()
      iterator operator++()
      iterator operator--()
      bint operator==(iterator)
      bint operator!=(iterator)
    iterator begin()
    iterator end()


cdef extern from "<mc_control/mc_controller.h>" namespace "mc_control":
  cdef cppclass ControllerResetData:
    const vector[vector[double]] & q

  cdef cppclass MCController:
    cppbool run()
    void reset(const ControllerResetData&)
    Robot& robot()
    Robot& env()
    Robots& robots()
    c_mc_rtc.Configuration & config()
    void supported_robots(vector[string] &)
    c_mc_rtc.Logger & logger()
    shared_ptr[c_mc_rtc_gui.StateBuilder] gui()

    double timeStep
    ContactConstraint contactConstraint
    DynamicsConstraint dynamicsConstraint
    KinematicsConstraint kinematicsConstraint
    CollisionsConstraint selfCollisionConstraint
    shared_ptr[c_mc_tasks.PostureTask] postureTask
    QPSolver & solver()

    cppbool hasObserverPipeline(const string &)
    c_mc_observers.ObserverPipeline & observerPipeline()
    c_mc_observers.ObserverPipeline & observerPipeline(const string&)
    vector[c_mc_observers.ObserverPipeline] & observerPipelines()

    void addCollisions(const string&, const string&,
                       const vector[Collision] &)
    void removeCollisions(const string&, const string&)
    void removeCollisions(const string&, const string&,
                          const vector[Collision] &)
    cppbool hasRobot(const string&)
    Robot& robot(const string&)
    void addContact(const Contact&)
    void removeContact(const Contact&)
    const ContactSet & contacts()
    cppbool hasContact(const Contact &)

cdef extern from "<mc_control/mc_python_controller.h>" namespace "mc_control":
  cdef cppclass PythonRWCallback:
    cppbool success
    string out

  cdef cppclass MCPythonController(MCController):
    MCPythonController(const vector[RobotModulePtr]&, double)

cdef extern from "<array>" namespace "std" nogil:
  cdef cppclass array[T, N]:
    array() except +
    T& operator[](size_t)

cdef extern from *:
  ctypedef int seven "7"
  ctypedef array[double, seven] array7d


cdef extern from "<mc_control/mc_global_controller.h>" namespace "mc_control":
  cdef cppclass MCGlobalController:
    MCGlobalController()
    MCGlobalController(string)
    MCGlobalController(string, RobotModulePtr)

    void init(vector[double])
    void init(vector[double], array7d)

    void setSensorPosition(Vector3d)
    void setSensorOrientation(Quaterniond)
    void setSensorLinearVelocity(Vector3d)
    void setSensorAngularVelocity(Vector3d)
    void setSensorLinearAcceleration(Vector3d)
    void setEncoderValues(vector[double])
    void setEncoderVelocities(vector[double])
    void setFlexibilityValues(vector[double])
    void setJointTorques(vector[double])
    void setWrenches(cppmap[string, ForceVecd])

    cppbool run()

    double timestep()
    MCController& controller()
    vector[string] ref_joint_order()
    Robot& robot()

    cppbool running

cdef extern from "mc_control_wrapper.hpp" namespace "mc_control":
  ControllerResetData & const_cast_crd(const ControllerResetData&)

  ctypedef cppbool (*run_callback_t)(void*) except+
  ctypedef void (*reset_callback_t)(const ControllerResetData&, void*) except+

  void set_run_callback(MCPythonController&, run_callback_t fn, void*)
  void set_reset_callback(MCPythonController&, reset_callback_t fn, void *)

  void add_anchor_frame_callback[T, U](MCPythonController &, const string &, T, U)
  void remove_anchor_frame_callback(MCPythonController &, const string &)
