cimport c_mc_rtc

from libcpp cimport bool as cppbool

cdef class Logger(object):
  cdef c_mc_rtc.Logger * impl

cdef Logger LoggerFromRef(c_mc_rtc.Logger &)

cdef class Configuration(object):
  cdef c_mc_rtc.Configuration * impl
  cdef cppbool __own_impl

cdef Configuration ConfigurationFromValue(c_mc_rtc.Configuration)
cdef Configuration ConfigurationFromRef(c_mc_rtc.Configuration &)
