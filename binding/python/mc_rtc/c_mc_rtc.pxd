#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.map cimport map as cppmap  # Careful map is a python built-in
from libcpp cimport bool as cppbool

ctypedef cppmap[string, vector[string]] mapStrVecStr
ctypedef cppmap[string, vector[double]] mapStrVecDouble

cimport eigen.c_eigen as c_eigen
cimport sva.c_sva as c_sva

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn
cimport mc_control.c_mc_control as c_mc_control

cdef extern from "<mc_rtc/config.h>" namespace "mc_rtc":
  const char * MC_ENV_DESCRIPTION_PATH
  const char * INSTALL_PREFIX
  const char * MC_ROBOTS_INSTALL_PREFIX
  const char * MC_CONTROLLER_INSTALL_PREFIX
  const char * DATA_PATH
  const char * CONF_PATH

cdef extern from "<memory>" namespace "std" nogil:
  cdef cppclass shared_ptr[T]:
    shared_ptr(T*)
    T* get()

include "mc_rtc_config.pxi"
IF MC_RTC_HAS_ROS == 1:
  cdef extern from "<mc_rtc/ros.h>" namespace "mc_rtc":
    cdef cppclass RobotPublisher:
      RobotPublisher(const string& prefix, double rate, double dt)

      void update(double dt, const c_mc_rbdyn.Robot & robot,
                  const cppmap[string, shared_ptr[c_mc_control.Gripper]]  &)

cdef extern from "<mc_rtc/log/Logger.h>" namespace "mc_rtc":
  cdef cppclass Logger:
    # Simplified from C++
    void addLogEntry[T](const string&, T get_fn)
    void removeLogEntry(const string&)

cdef extern from "<mc_rtc/Configuration.h>" namespace "mc_rtc":
  cdef cppclass Configuration:
    Configuration()
    Configuration(const string&)
    Configuration(const Configuration&)

    void load(const Configuration&)
    void load(const string&)
    void loadData(const string&)
    void save(const string&, cppbool)
    string dump(cppbool)

    cppbool has(const string&)
    Configuration operator()(const string&) except +
    Configuration operator[](int) except +
    vector[string] keys()
    cppbool empty()
    int size()

    Configuration add(const string&) except +
    void add(const string &, const Configuration &) except +

    Configuration array(const string&, int) except +
    void push(const Configuration &) except +

    cppbool remove(const string&)

cdef extern from "mc_rtc_wrapper.hpp":
  cdef cppclass function[T]:
    pass
  function[c_eigen.Vector3d] make_v3d_log_callback[T,U](T,U)
  function[double] make_double_log_callback[T,U](T,U)
  function[vector[double]] make_doublev_log_callback[T,U](T,U)
  function[c_eigen.Quaterniond] make_quat_log_callback[T,U](T,U)
  function[c_sva.PTransformd] make_pt_log_callback[T,U](T,U)
  function[c_sva.ForceVecd] make_fv_log_callback[T,U](T,U)
  function[string] make_string_log_callback[T,U](T,U)

  T get_config_as[T](Configuration&) except +
  T get_config_as[T](Configuration&, const T&) except +
  Configuration get_as_config[T](const T&) except +
  Configuration ConfigurationFromData(const string&)
