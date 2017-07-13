from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp.map cimport map as cppmap  # Careful map is a python built-in

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn

ctypedef cppmap[string, vector[string]] mapStrVecStr
ctypedef cppmap[string, vector[double]] mapStrVecDouble

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
