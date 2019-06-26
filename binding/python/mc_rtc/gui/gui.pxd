#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport c_gui

cimport mc_rtc.c_mc_rtc as c_mc_rtc

from libcpp cimport bool as cppbool

cdef class StateBuilder(object):
  cdef c_gui.shared_ptr[c_gui.StateBuilder] impl

cdef StateBuilder StateBuilderFromShPtr(c_gui.shared_ptr[c_gui.StateBuilder])

