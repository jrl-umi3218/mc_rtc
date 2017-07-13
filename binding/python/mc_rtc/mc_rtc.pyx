# distutils: language = c++

from cython.operator cimport dereference as deref

cimport mc_rtc.c_mc_rtc as c_mc_rtc

cimport mc_rbdyn.mc_rbdyn as mc_rbdyn

MC_ENV_DESCRIPTION_PATH = c_mc_rtc.MC_ENV_DESCRIPTION_PATH
HRP2_DRC_DESCRIPTION_PATH = c_mc_rtc.HRP2_DRC_DESCRIPTION_PATH
HRP4_DESCRIPTION_PATH = c_mc_rtc.HRP4_DESCRIPTION_PATH
INSTALL_PREFIX = c_mc_rtc.INSTALL_PREFIX
MC_ROBOTS_INSTALL_PREFIX = c_mc_rtc.MC_ROBOTS_INSTALL_PREFIX
MC_CONTROLLER_INSTALL_PREFIX = c_mc_rtc.MC_CONTROLLER_INSTALL_PREFIX
DATA_PATH = c_mc_rtc.DATA_PATH
CONF_PATH = c_mc_rtc.CONF_PATH

include "mc_rtc_config.pxi"
IF MC_RTC_HAS_ROS == 1:
  cdef class RobotPublisher(object):
    cdef c_mc_rtc.RobotPublisher * impl

    def __cinit__(self, prefix, rate):
      self.impl = new c_mc_rtc.RobotPublisher(prefix, rate)

    def update(self, dt, mc_rbdyn.Robot robot, gripperJ = {}, gripperQ = {}):
      cdef c_mc_rtc.mapStrVecStr c_gripperJ
      cdef c_mc_rtc.mapStrVecDouble c_gripperQ

      # Cython knows how to convert vectors but not maps of vectors
      for k, v in gripperJ.items():
        c_gripperJ[k] = v

      for k, v in gripperQ.items():
        c_gripperQ[k] = v

      self.impl.update(dt, deref(robot.impl), c_gripperJ, c_gripperQ)
