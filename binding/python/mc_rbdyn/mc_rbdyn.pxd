cimport c_mc_rbdyn
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef extern from "<memory>" namespace "std" nogil:
  cdef cppclass shared_ptr[T]:
    shared_ptr()
    shared_ptr(T*)
    T* get()
    T& operator*()

cdef class Collision(object):
  cdef c_mc_rbdyn.Collision impl

cdef Collision CollisionFromC(const c_mc_rbdyn.Collision&)

cdef class BodySensor(object):
  cdef cppbool __own_impl
  cdef c_mc_rbdyn.BodySensor * impl

cdef BodySensor BodySensorFromRef(c_mc_rbdyn.BodySensor&)

cdef BodySensor BodySensorFromCRef(const c_mc_rbdyn.BodySensor&)

cdef class Flexibility(object):
  cdef c_mc_rbdyn.Flexibility impl

cdef Flexibility FlexibilityFromC(const c_mc_rbdyn.Flexibility&)

cdef class ForceSensor(object):
  cdef cppbool __own_impl
  cdef c_mc_rbdyn.ForceSensor  * impl

cdef ForceSensor ForceSensorFromRef(c_mc_rbdyn.ForceSensor&)

cdef ForceSensor ForceSensorFromCRef(const c_mc_rbdyn.ForceSensor&)

cdef class Springs(object):
  cdef c_mc_rbdyn.Springs impl

cdef class Base(object):
  cdef c_mc_rbdyn.Base impl

cdef Springs SpringsFromC(const c_mc_rbdyn.Springs&)

cdef public api class RobotModule(object)[object RobotModuleObject, type RobotModuleType]:
  cdef c_mc_rbdyn.RobotModulePtr impl

cdef public api RobotModule RobotModuleFromC(c_mc_rbdyn.RobotModulePtr)

cdef public api class RobotModuleVector(object)[object RobotModuleVectorObject, type RobotModuleVectorType]:
  cdef vector[c_mc_rbdyn.RobotModulePtr] v

cdef class Robots(object):
  cdef shared_ptr[c_mc_rbdyn.Robots] impl

cdef Robots RobotsFromPtr(shared_ptr[c_mc_rbdyn.Robots])

cdef Robots RobotsFromRawPtr(c_mc_rbdyn.Robots *)

cdef Robots RobotsFromRef(c_mc_rbdyn.Robots &)

# Robot cannot be copied and will never own their pointer
cdef class Robot(object):
  cdef c_mc_rbdyn.Robot * impl

cdef Robot RobotFromC(const c_mc_rbdyn.Robot &)

# Surface are also not copiable
cdef class Surface(object):
  cdef shared_ptr[c_mc_rbdyn.Surface] ptr
  cdef c_mc_rbdyn.Surface * impl

cdef SurfaceFromC(const c_mc_rbdyn.Surface &)

cdef SurfaceFromPtr(shared_ptr[c_mc_rbdyn.Surface], cppbool keep_ptr=?)

cdef class PlanarSurface(Surface):
  cdef c_mc_rbdyn.PlanarSurface * surf

cdef PlanarSurface PlanarSurfaceFromPtr(c_mc_rbdyn.PlanarSurface*)

cdef class GripperSurface(Surface):
  cdef c_mc_rbdyn.GripperSurface * surf

cdef GripperSurface GripperSurfaceFromPtr(c_mc_rbdyn.GripperSurface*)

cdef class CylindricalSurface(Surface):
  cdef c_mc_rbdyn.CylindricalSurface * surf

cdef CylindricalSurface CylindricalSurfaceFromPtr(c_mc_rbdyn.CylindricalSurface*)

cdef class Contact(object):
  cdef c_mc_rbdyn.Contact * impl
  cdef cppbool __own_impl

cdef Contact ContactFromC(const c_mc_rbdyn.Contact &, cppbool copy=?)

cdef class ContactVector(object):
  cdef vector[c_mc_rbdyn.Contact] * v
  cdef cppbool __own_impl

cdef ContactVector ContactVectorFromC(const vector[c_mc_rbdyn.Contact] &, cppbool copy=?)

cdef class GeosGeomGeometry(object):
  cdef shared_ptr[c_mc_rbdyn.Geometry] impl

cdef GeosGeomGeometry GeosGeomGeometryFromSharedPtr(shared_ptr[c_mc_rbdyn.Geometry])

cdef class PolygonInterpolator(object):
  cdef c_mc_rbdyn.PolygonInterpolator * impl
  cdef cppbool __own_impl
