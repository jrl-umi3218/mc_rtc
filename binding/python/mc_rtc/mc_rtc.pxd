cimport c_mc_rtc

cdef class Logger(object):
  cdef c_mc_rtc.Logger * impl

cdef Logger LoggerFromRef(c_mc_rtc.Logger &)
