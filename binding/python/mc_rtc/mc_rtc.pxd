#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_rtc.c_mc_rtc as c_mc_rtc

from libcpp cimport bool as cppbool

cdef class Logger(object):
  cdef c_mc_rtc.Logger * impl

cdef Logger LoggerFromRef(c_mc_rtc.Logger &)

cdef class Configuration(object):
  cdef c_mc_rtc.Configuration * impl
  cdef cppbool own_impl__

cdef Configuration ConfigurationFromValue(c_mc_rtc.Configuration)
cdef Configuration ConfigurationFromRef(c_mc_rtc.Configuration &)
