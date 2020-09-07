#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport c_mc_observers

cdef class ObserverPipeline(object):
    cdef c_mc_observers.ObserverPipeline * impl

cdef ObserverPipeline ObserverPipelineFromRef(c_mc_observers.ObserverPipeline &)
