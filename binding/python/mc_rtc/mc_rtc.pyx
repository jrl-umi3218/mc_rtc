# distutils: language = c++

#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

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
INSTALL_PREFIX = c_mc_rtc.INSTALL_PREFIX
MC_ROBOTS_INSTALL_PREFIX = c_mc_rtc.MC_ROBOTS_INSTALL_PREFIX
MC_CONTROLLER_INSTALL_PREFIX = c_mc_rtc.MC_CONTROLLER_INSTALL_PREFIX
DATA_PATH = c_mc_rtc.DATA_PATH
CONF_PATH = c_mc_rtc.CONF_PATH

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
  ret = get_fn()
  if isinstance(ret, unicode):
    ret = ret.encode(u'ascii')
  return ret

cdef class Logger(object):
  CALLBACKS = []
  def __cinit__(self):
    self.impl = NULL
  def addLogEntry(self, name, get_fn):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
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
  def removeLogEntry(self, name):
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    self.impl.removeLogEntry(name)

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
        path = args[0]
        if isinstance(path, unicode):
          path = path.encode(u'ascii')
        self.impl = new c_mc_rtc.Configuration(<string>(path))
    else:
      self.__own_impl = False
      self.impl = NULL
  @staticmethod
  def fromData(data):
    if isinstance(data, unicode):
      data = data.encode(u'ascii')
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
      value = value.encode(u'ascii')
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
        default = default.encode(u'ascii')
        return c_mc_rtc.get_config_as[string](deref(self.impl), default).decode(u'ascii')
      else:
        return c_mc_rtc.get_config_as[string](deref(self.impl)).decode(u'ascii')
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
      if isinstance(other, unicode):
        other = other.encode(u'ascii')
      self.impl.load(<string>(other))
  def loadData(self, data):
    if isinstance(data, unicode):
      data = data.encode(u'ascii')
    self.impl.loadData(data)
  def save(self, path, pretty = True):
    if isinstance(path, unicode):
      path = path.encode(u'ascii')
    self.impl.save(path, pretty)
  def dump(self, pretty = False):
    return self.impl.dump(pretty)
  def has(self, key):
    if isinstance(key, unicode):
      key = key.encode(u'ascii')
    return self.impl.has(key)
  def empty(self):
    return self.impl.empty()
  def keys(self):
    return self.impl.keys()
  def size(self):
    return self.impl.size()
  def add(self, key, value = None):
    if isinstance(key, unicode):
      key = key.encode(u'ascii')
    if value is None:
      return ConfigurationFromValue(self.impl.add(key))
    else:
      self.impl.add(key, deref((<Configuration>(Configuration.from_(value))).impl))
  def array(self, key, size = 0):
    if isinstance(key, unicode):
      key = key.encode(u'ascii')
    return ConfigurationFromValue(self.impl.array(key, size))
  def push(self, value = None):
    self.impl.push(deref((<Configuration>(Configuration.from_(value))).impl))
  def remove(self, k):
    if isinstance(k, unicode):
      k = k.encode(u'ascii')
    return self.impl.remove(k)
  # Special methods
  def __call__(self, key, t = None):
    if isinstance(key, unicode):
      key = key.encode(u'ascii')
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

class Loader(object):
  @staticmethod
  def debug_suffix(suffix):
    if isinstance(suffix, unicode):
      suffix = suffix.encode(u'ascii')
    c_mc_rtc.set_loader_debug_suffix(suffix)
