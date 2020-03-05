#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from eigen.c_eigen cimport *
from sva.c_sva cimport *
from rbdyn.c_rbdyn cimport *
cimport sch.c_sch as sch
cimport tasks.qp.c_qp as c_qp
from mc_rbdyn.c_mc_rbdyn cimport *
from mc_solver.c_mc_solver cimport *
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

cdef extern from "<mc_control/generic_gripper.h>" namespace "mc_control":
  cdef cppclass Gripper:
    vector[string] names
    vector[double] _q

ctypedef shared_ptr[Gripper] GripperPtr
ctypedef cppmap[string, GripperPtr] GripperMap

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
    vector[string] supported_robots()
    c_mc_rtc.Logger & logger()
    shared_ptr[c_mc_rtc_gui.StateBuilder] gui()

    double timeStep
    GripperMap grippers
    ContactConstraint contactConstraint
    DynamicsConstraint dynamicsConstraint
    KinematicsConstraint kinematicsConstraint
    CollisionsConstraint selfCollisionConstraint
    shared_ptr[c_mc_tasks.PostureTask] postureTask
    QPSolver & solver()

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
    void setSensorAcceleration(Vector3d)
    void setEncoderValues(vector[double])
    void setEncoderVelocities(vector[double])
    void setFlexibilityValues(vector[double])
    void setJointTorques(vector[double])
    void setWrenches(cppmap[string, ForceVecd])
    void setActualGripperQ(cppmap[string, vector[double]])

    cppbool run()

    double timestep()
    MCController& controller()
    vector[string] ref_joint_order()
    Robot& robot()

    cppbool running

cdef extern from "mc_control_wrapper.hpp" namespace "mc_control":
  ControllerResetData & const_cast_crd(const ControllerResetData&)

  ctypedef cppbool (*run_callback_t)(void*)
  ctypedef void (*reset_callback_t)(const ControllerResetData&, void*)

  void set_run_callback(MCPythonController&, run_callback_t fn, void*)
  void set_reset_callback(MCPythonController&, reset_callback_t fn, void *)
