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
    cppbool set_joint_pos(string, double)
    cppbool play_next_stance()
    cppbool read_msg(string)
    cppbool read_write_msg(string, string)
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


cdef extern from "mc_control_wrapper.hpp" namespace "mc_control":
  ControllerResetData & const_cast_crd(const ControllerResetData&)

  ctypedef cppbool (*run_callback_t)(void*)
  ctypedef void (*reset_callback_t)(const ControllerResetData&, void*)
  ctypedef cppbool (*read_msg_callback_t)(string, void*)
  ctypedef PythonRWCallback (*read_write_msg_callback_t)(string, void*)

  void set_run_callback(MCPythonController&, run_callback_t fn, void*)
  void set_reset_callback(MCPythonController&, reset_callback_t fn, void *)
  void set_read_msg_callback(MCPythonController&, read_msg_callback_t, void*)
  void set_read_write_msg_callback(MCPythonController&, read_write_msg_callback_t, void*)
