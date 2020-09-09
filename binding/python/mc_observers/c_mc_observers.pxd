#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

from libcpp cimport bool as cppbool
from libcpp.string cimport string

cdef extern from "<mc_observers/ObserverPipeline.h>" namespace "mc_observers":
    cdef cppclass ObserverPipeline:
        void reset()
        cppbool runObservers()
        void runObservers(cppbool)
        cppbool updateObservers()
        void updateObservers(cppbool)
        cppbool success()
        string desc()
        string name()
