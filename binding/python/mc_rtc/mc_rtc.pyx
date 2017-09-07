# distutils: language = c++

from cython.operator cimport dereference as deref

cimport mc_rtc.c_mc_rtc as c_mc_rtc

cimport mc_rbdyn.mc_rbdyn as mc_rbdyn

cimport eigen.c_eigen as c_eigen
cimport eigen.eigen as eigen

cimport sva.c_sva as c_sva
cimport sva.sva as sva

from libcpp.vector cimport vector
from libcpp.string cimport string
from cython.operator cimport dereference as deref

import collections
import numbers

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

cdef c_eigen.Vector3d python_log_v3d(get_fn) with gil:
  cdef eigen.Vector3d ret = get_fn()
  return ret.impl

cdef double python_log_double(get_fn) with gil:
  return <double>get_fn()

cdef vector[double] python_log_doublev(get_fn) with gil:
  return get_fn()

cdef c_eigen.Quaterniond python_log_quat(get_fn) with gil:
  cdef eigen.Quaterniond ret = get_fn()
  return ret.impl

cdef c_sva.PTransformd python_log_pt(get_fn) with gil:
  cdef sva.PTransformd ret = get_fn()
  return c_sva.PTransformd(deref(ret.impl))

cdef c_sva.ForceVecd python_log_fv(get_fn) with gil:
  cdef sva.ForceVecd ret = get_fn()
  return c_sva.ForceVecd(deref(ret.impl))

cdef string python_log_string(get_fn) with gil:
  return get_fn()

cdef class Logger(object):
  CALLBACKS = []
  def __cinit__(self):
    self.impl = NULL
  def addLogEntry(self, name, get_fn):
    Logger.CALLBACKS.append(get_fn)
    ret = get_fn()
    if isinstance(ret, eigen.Vector3d):
      self.impl.addLogEntry[c_mc_rtc.function[c_eigen.Vector3d]](name, <c_mc_rtc.function[c_eigen.Vector3d]>(c_mc_rtc.make_v3d_log_callback(python_log_v3d, get_fn)))
    elif isinstance(ret, numbers.Number):
      self.impl.addLogEntry[c_mc_rtc.function[double]](name, c_mc_rtc.make_double_log_callback(python_log_double, get_fn))
    elif isinstance(ret, collections.Iterable) and len(ret) and isinstance(ret[0], numbers.Number):
      self.impl.addLogEntry[c_mc_rtc.function[vector[double]]](name, c_mc_rtc.make_doublev_log_callback(python_log_doublev, get_fn))
    elif isinstance(ret, eigen.Quaterniond):
      self.impl.addLogEntry[c_mc_rtc.function[c_eigen.Quaterniond]](name, c_mc_rtc.make_quat_log_callback(python_log_quat, get_fn))
    elif isinstance(ret, sva.PTransformd):
      self.impl.addLogEntry[c_mc_rtc.function[c_sva.PTransformd]](name, c_mc_rtc.make_pt_log_callback(python_log_pt, get_fn))
    elif isinstance(ret, sva.ForceVecd):
      self.impl.addLogEntry[c_mc_rtc.function[c_sva.ForceVecd]](name, c_mc_rtc.make_fv_log_callback(python_log_fv, get_fn))
    elif isinstance(ret, str):
      self.impl.addLogEntry[c_mc_rtc.function[string]](name, c_mc_rtc.make_string_log_callback(python_log_string, get_fn))
    else:
      raise TypeError("Cannot convert a callback returning " + str(type(ret)))

cdef Logger LoggerFromRef(c_mc_rtc.Logger & logger):
  cdef Logger ret = Logger()
  ret.impl = &logger
  return ret
