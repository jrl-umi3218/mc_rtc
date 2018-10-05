# distutils: language = c++

from cython.operator cimport dereference as deref

cimport mc_rtc.c_mc_rtc as c_mc_rtc

cimport mc_rbdyn.mc_rbdyn as mc_rbdyn
cimport mc_control.c_mc_control as c_mc_control
cimport mc_control.mc_control as mc_control

cimport eigen.c_eigen as c_eigen
cimport eigen.eigen as eigen

cimport sva.c_sva as c_sva
cimport sva.sva as sva

from libcpp.vector cimport vector
from libcpp.string cimport string
from libcpp cimport bool as cppbool
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

    def __cinit__(self, prefix, rate, dt):
      self.impl = new c_mc_rtc.RobotPublisher(prefix, rate, dt)

    def update(self, dt, mc_rbdyn.Robot robot, grippersIn):
      cdef c_mc_control.GripperMap grippers
      for name, gripper in grippersIn.items():
        assert(isinstance(gripper, mc_control.Gripper))
        grippers[name] = (<mc_control.Gripper>gripper).impl
      self.impl.update(dt, deref(robot.impl), grippers)

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

cdef class Configuration(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, *args, skip_alloc = False):
    if not skip_alloc:
      self.__own_impl = True
      if(len(args) == 0):
        self.impl = new c_mc_rtc.Configuration()
      else:
        assert(len(args) == 1)
        self.impl = new c_mc_rtc.Configuration(<string>(args[0]))
    else:
      self.__own_impl = False
      self.impl = NULL
  @staticmethod
  def fromData(data):
    return ConfigurationFromValue(c_mc_rtc.ConfigurationFromData(data))
  # Convert Python object to Configuration
  @staticmethod
  def from_(value):
    t = type(value)
    cdef c_mc_rtc.Configuration ret
    if t is bool:
      ret = c_mc_rtc.get_as_config[cppbool](value)
    elif isinstance(t(), numbers.Integral):
      ret = c_mc_rtc.get_as_config[int](value)
    elif isinstance(t(), numbers.Number):
      ret = c_mc_rtc.get_as_config[double](value)
    elif t is str or t is unicode:
      ret = c_mc_rtc.get_as_config[string](value)
    elif t is eigen.Vector2d:
      ret = c_mc_rtc.get_as_config[c_eigen.Vector2d]((<eigen.Vector2d>(value)).impl)
    elif t is eigen.Vector3d:
      ret = c_mc_rtc.get_as_config[c_eigen.Vector3d]((<eigen.Vector3d>(value)).impl)
    elif t is eigen.Vector6d:
      ret = c_mc_rtc.get_as_config[c_eigen.Vector6d]((<eigen.Vector6d>(value)).impl)
    elif t is eigen.VectorXd:
      ret = c_mc_rtc.get_as_config[c_eigen.VectorXd]((<eigen.VectorXd>(value)).impl)
    elif t is eigen.Quaterniond:
      ret = c_mc_rtc.get_as_config[c_eigen.Quaterniond]((<eigen.Quaterniond>(value)).impl)
    elif t is eigen.Matrix3d:
      ret = c_mc_rtc.get_as_config[c_eigen.Matrix3d]((<eigen.Matrix3d>(value)).impl)
    elif t is eigen.Matrix6d:
      ret = c_mc_rtc.get_as_config[c_eigen.Matrix6d]((<eigen.Matrix6d>(value)).impl)
    elif t is eigen.MatrixXd:
      ret = c_mc_rtc.get_as_config[c_eigen.MatrixXd]((<eigen.MatrixXd>(value)).impl)
    elif t is sva.PTransformd:
      ret = c_mc_rtc.get_as_config[c_sva.PTransformd](deref((<sva.PTransformd>(value)).impl))
    elif t is list:
      ret = c_mc_rtc.Configuration()
      ret = ret.array("l", len(value))
      for v in value:
        ret.push(deref((<Configuration>Configuration.from_(v)).impl))
    else:
        raise TypeError("{} cannot be converted to Configuration".format(t))
    return ConfigurationFromValue(ret)
  # Retrieve this object as an actual type
  def to(self, t, default = None):
    if default is None and not isinstance(t, type):
      return self.to(type(t), t)
    assert(default is None or isinstance(default, t))
    if t is Configuration:
      return self
    if t is bool:
      if default is not None:
        return c_mc_rtc.get_config_as[cppbool](deref(self.impl), default)
      else:
        return c_mc_rtc.get_config_as[cppbool](deref(self.impl))
    if isinstance(t(), numbers.Integral):
      if default is not None:
        return c_mc_rtc.get_config_as[int](deref(self.impl), default)
      else:
        return c_mc_rtc.get_config_as[int](deref(self.impl))
    if isinstance(t(), numbers.Number):
      if default is not None:
        return c_mc_rtc.get_config_as[double](deref(self.impl), default)
      else:
        return c_mc_rtc.get_config_as[double](deref(self.impl))
    if t is str or t is unicode:
      if default is not None:
        return c_mc_rtc.get_config_as[string](deref(self.impl), default)
      else:
        return c_mc_rtc.get_config_as[string](deref(self.impl))
    if t is eigen.Vector2d:
      if default is not None:
        return eigen.Vector2dFromC(c_mc_rtc.get_config_as[c_eigen.Vector2d](deref(self.impl), (<eigen.Vector2d>(default)).impl))
      else:
        return eigen.Vector2dFromC(c_mc_rtc.get_config_as[c_eigen.Vector2d](deref(self.impl)))
    if t is eigen.Vector3d:
      if default is not None:
        return eigen.Vector3dFromC(c_mc_rtc.get_config_as[c_eigen.Vector3d](deref(self.impl), (<eigen.Vector3d>(default)).impl))
      else:
        return eigen.Vector3dFromC(c_mc_rtc.get_config_as[c_eigen.Vector3d](deref(self.impl)))
    if t is eigen.Vector6d:
      if default is not None:
        return eigen.Vector6dFromC(c_mc_rtc.get_config_as[c_eigen.Vector6d](deref(self.impl), (<eigen.Vector6d>(default)).impl))
      else:
        return eigen.Vector6dFromC(c_mc_rtc.get_config_as[c_eigen.Vector6d](deref(self.impl)))
    if t is eigen.VectorXd:
      if default is not None:
        return eigen.VectorXdFromC(c_mc_rtc.get_config_as[c_eigen.VectorXd](deref(self.impl), (<eigen.VectorXd>(default)).impl))
      else:
        return eigen.VectorXdFromC(c_mc_rtc.get_config_as[c_eigen.VectorXd](deref(self.impl)))
    if t is eigen.Quaterniond:
      if default is not None:
        return eigen.QuaterniondFromC(c_mc_rtc.get_config_as[c_eigen.Quaterniond](deref(self.impl), (<eigen.Quaterniond>(default)).impl))
      else:
        return eigen.QuaterniondFromC(c_mc_rtc.get_config_as[c_eigen.Quaterniond](deref(self.impl)))
    if t is eigen.Matrix3d:
      if default is not None:
        return eigen.Matrix3dFromC(c_mc_rtc.get_config_as[c_eigen.Matrix3d](deref(self.impl), (<eigen.Matrix3d>(default)).impl))
      else:
        return eigen.Matrix3dFromC(c_mc_rtc.get_config_as[c_eigen.Matrix3d](deref(self.impl)))
    if t is eigen.Matrix6d:
      if default is not None:
        return eigen.Matrix6dFromC(c_mc_rtc.get_config_as[c_eigen.Matrix6d](deref(self.impl), (<eigen.Matrix6d>(default)).impl))
      else:
        return eigen.Matrix6dFromC(c_mc_rtc.get_config_as[c_eigen.Matrix6d](deref(self.impl)))
    if t is eigen.MatrixXd:
      if default is not None:
        return eigen.MatrixXdFromC(c_mc_rtc.get_config_as[c_eigen.MatrixXd](deref(self.impl), (<eigen.MatrixXd>(default)).impl))
      else:
        return eigen.MatrixXdFromC(c_mc_rtc.get_config_as[c_eigen.MatrixXd](deref(self.impl)))
    if t is sva.PTransformd:
      if default is not None:
        return sva.PTransformdFromC(c_mc_rtc.get_config_as[c_sva.PTransformd](deref(self.impl), deref((<sva.PTransformd>(default)).impl)))
      else:
        return sva.PTransformdFromC(c_mc_rtc.get_config_as[c_sva.PTransformd](deref(self.impl)))
    if t is list:
      return [c.to(default[0]) for c in self]
    raise TypeError("Conversion to {} not supported".format(t))
  def load(self, other):
    if isinstance(other, Configuration):
      self.impl.load(deref((<Configuration>(other)).impl))
    else:
      self.impl.load(<string>(other))
  def loadData(self, data):
    self.impl.loadData(data)
  def save(self, path, pretty = True):
    self.impl.save(path, pretty)
  def dump(self, pretty = False):
    return self.impl.dump(pretty)
  def has(self, key):
    return self.impl.has(key)
  def empty(self):
    return self.impl.empty()
  def keys(self):
    return self.impl.keys()
  def size(self):
    return self.impl.size()
  def add(self, key, value = None):
    if value is None:
      return ConfigurationFromValue(self.impl.add(key))
    else:
      self.impl.add(key, deref((<Configuration>(Configuration.from_(value))).impl))
  def array(self, key, size = 0):
    return ConfigurationFromValue(self.impl.array(key, size))
  def push(self, value = None):
    self.impl.push(deref((<Configuration>(Configuration.from_(value))).impl))
  def remove(self, k):
    return self.impl.remove(k)
  # Special methods
  def __call__(self, key, t = None):
    if t is None:
      return ConfigurationFromValue(deref(self.impl)(key))
    default = None
    if not isinstance(t, type):
      default = t
      t = type(default)
    if default is None:
      return ConfigurationFromValue(deref(self.impl)(key)).to(t, default)
    try:
      return ConfigurationFromValue(deref(self.impl)(key)).to(t, default)
    except:
      return default
  def __getitem__(self, idx):
    if idx < self.size():
      return ConfigurationFromValue(deref(self.impl)[idx])
    else:
      raise IndexError("Out of bounds")

cdef Configuration ConfigurationFromValue(c_mc_rtc.Configuration conf):
  cdef Configuration ret = Configuration(skip_alloc = True)
  ret.__own_impl = True
  ret.impl = new c_mc_rtc.Configuration(conf)
  return ret

cdef Configuration ConfigurationFromRef(c_mc_rtc.Configuration & conf):
  cdef Configuration ret = Configuration(skip_alloc = True)
  ret.impl = &conf
  return ret
