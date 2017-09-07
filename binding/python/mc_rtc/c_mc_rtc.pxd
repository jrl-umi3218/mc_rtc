from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.map cimport map as cppmap  # Careful map is a python built-in

ctypedef cppmap[string, vector[string]] mapStrVecStr
ctypedef cppmap[string, vector[double]] mapStrVecDouble

cimport eigen.c_eigen as c_eigen
cimport sva.c_sva as c_sva

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn

cdef extern from "<mc_rtc/config.h>" namespace "mc_rtc":
  const char * MC_ENV_DESCRIPTION_PATH
  const char * HRP2_DRC_DESCRIPTION_PATH
  const char * HRP4_DESCRIPTION_PATH
  const char * INSTALL_PREFIX
  const char * MC_ROBOTS_INSTALL_PREFIX
  const char * MC_CONTROLLER_INSTALL_PREFIX
  const char * DATA_PATH
  const char * CONF_PATH

include "mc_rtc_config.pxi"
IF MC_RTC_HAS_ROS == 1:
  cdef extern from "<mc_rtc/ros.h>" namespace "mc_rtc":
    cdef cppclass RobotPublisher:
      RobotPublisher(const string& prefix, unsigned int rate)

      void update(double dt, const c_mc_rbdyn.Robot & robot,
                  const mapStrVecStr& gripperJ,
                  const mapStrVecDouble& gripperQ)

cdef extern from "<mc_rtc/log/Logger.h>" namespace "mc_rtc":
  cdef cppclass Logger:
    # Simplified from C++
    void addLogEntry[T](const string&, T get_fn)

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
