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

cdef class Stance(object):
  cdef c_mc_rbdyn.Stance * impl
  cdef cppbool __own_impl

# Stance reference are always borrowed
cdef Stance StanceFromC(const c_mc_rbdyn.Stance &)

cdef class StanceVector(object):
  cdef vector[c_mc_rbdyn.Stance] * v

cdef class StanceAction(object):
  cdef c_mc_rbdyn.StanceAction * base

cdef class IdentityContactAction(StanceAction):
  cdef c_mc_rbdyn.IdentityContactAction * impl
  cdef cppbool __own_impl

cdef class AddContactAction(StanceAction):
  cdef c_mc_rbdyn.AddContactAction * impl
  cdef cppbool __own_impl

cdef class RemoveContactAction(StanceAction):
  cdef c_mc_rbdyn.RemoveContactAction * impl
  cdef cppbool __own_impl

cdef StanceActionFromPtr(shared_ptr[c_mc_rbdyn.StanceAction])

cdef class StanceConfigCoMTask(object):
  cdef c_mc_rbdyn.StanceConfigCoMTask * impl
  cdef cppbool __own_impl

cdef StanceConfigCoMTask StanceConfigCoMTaskFromPtr(c_mc_rbdyn.StanceConfigCoMTask*)

cdef class StanceConfigCoMObj(object):
  cdef c_mc_rbdyn.StanceConfigCoMObj * impl
  cdef cppbool __own_impl

cdef StanceConfigCoMObj StanceConfigCoMObjFromPtr(c_mc_rbdyn.StanceConfigCoMObj*)

cdef class StanceConfigPostureTask(object):
  cdef c_mc_rbdyn.StanceConfigPostureTask * impl
  cdef cppbool __own_impl

cdef StanceConfigPostureTask StanceConfigPostureTaskFromPtr(c_mc_rbdyn.StanceConfigPostureTask*)

cdef class StanceConfigPosition(object):
  cdef c_mc_rbdyn.StanceConfigPosition * impl
  cdef cppbool __own_impl

cdef StanceConfigPosition StanceConfigPositionFromPtr(c_mc_rbdyn.StanceConfigPosition*)

cdef class StanceConfigOrientation(object):
  cdef c_mc_rbdyn.StanceConfigOrientation * impl
  cdef cppbool __own_impl

cdef StanceConfigOrientation StanceConfigOrientationFromPtr(c_mc_rbdyn.StanceConfigOrientation*)

cdef class StanceConfigLinVel(object):
  cdef c_mc_rbdyn.StanceConfigLinVel * impl
  cdef cppbool __own_impl

cdef StanceConfigLinVel StanceConfigLinVelFromPtr(c_mc_rbdyn.StanceConfigLinVel*)

cdef class StanceConfigWaypointConf(object):
  cdef c_mc_rbdyn.StanceConfigWaypointConf * impl
  cdef cppbool __own_impl

cdef StanceConfigWaypointConf StanceConfigWaypointConfFromPtr(c_mc_rbdyn.StanceConfigWaypointConf *)

cdef class StanceConfigCollisionConf(object):
  cdef c_mc_rbdyn.StanceConfigCollisionConf * impl
  cdef cppbool __own_impl

cdef StanceConfigCollisionConf StanceConfigCollisionConfFromPtr(c_mc_rbdyn.StanceConfigCollisionConf*)

cdef class StanceConfigContactTask(object):
  cdef c_mc_rbdyn.StanceConfigContactTask * impl
  cdef cppbool __own_impl

cdef StanceConfigContactTask StanceConfigContactTaskFromPtr(c_mc_rbdyn.StanceConfigContactTask*)

cdef class StanceConfigContactObj(object):
  cdef c_mc_rbdyn.StanceConfigContactObj * impl
  cdef cppbool __own_impl

cdef StanceConfigContactObj StanceConfigContactObjFromPtr(c_mc_rbdyn.StanceConfigContactObj*)

cdef class StanceConfigBodiesCollisionConf(object):
  cdef c_mc_rbdyn.StanceConfigBodiesCollisionConf * impl
  cdef cppbool __own_impl

cdef StanceConfigBodiesCollisionConf StanceConfigBodiesCollisionConfFromPtr(c_mc_rbdyn.StanceConfigBodiesCollisionConf*)

cdef class StanceConfigBodiesCollisionConfVector(object):
  cdef vector[c_mc_rbdyn.StanceConfigBodiesCollisionConf] * v
  cdef cppbool __own_impl

cdef StanceConfigBodiesCollisionConfVector StanceConfigBodiesCollisionConfVectorFromPtr(vector[c_mc_rbdyn.StanceConfigBodiesCollisionConf]*)

cdef class StanceConfigCollisions(object):
  cdef c_mc_rbdyn.StanceConfigCollisions * impl
  cdef cppbool __own_impl

cdef StanceConfigCollisions StanceConfigCollisionsFromPtr(c_mc_rbdyn.StanceConfigCollisions*)

cdef class StanceConfig(object):
  cdef c_mc_rbdyn.StanceConfig * impl
  cdef cppbool __own_impl

cdef class GeosGeomGeometry(object):
  cdef shared_ptr[c_mc_rbdyn.Geometry] impl

cdef GeosGeomGeometry GeosGeomGeometryFromSharedPtr(shared_ptr[c_mc_rbdyn.Geometry])

cdef class PolygonInterpolator(object):
  cdef c_mc_rbdyn.PolygonInterpolator * impl
  cdef cppbool __own_impl
