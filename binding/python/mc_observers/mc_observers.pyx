# distutils: language = c++

#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_observers.c_mc_observers as c_mc_observers

cdef class ObserverPipeline(object):
    def __cinit__(self):
        self.impl = NULL
    def reset(self):
        assert(self.impl)
        self.impl.reset()
    def runObservers(self, value = None):
        assert(self.impl)
        if value is None:
            return self.impl.runObservers()
        else:
            self.impl.runObservers(value)
    def updateObservers(self, value = None):
        assert(self.impl)
        if value is None:
            return self.impl.updateObservers()
        else:
            self.impl.updateObservers(value)
    def success(self):
        assert(self.impl)
        return self.impl.success()
    def desc(self):
        assert(self.impl)
        return self.impl.desc()
    def name(self):
        assert(self.impl)
        return self.impl.name()

cdef ObserverPipeline ObserverPipelineFromRef(c_mc_observers.ObserverPipeline & pipeline):
    cdef ObserverPipeline ret = ObserverPipeline()
    ret.impl = &pipeline
    return ret
