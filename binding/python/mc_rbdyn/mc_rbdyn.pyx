# distutils: language = c++

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn

cimport eigen.c_eigen as c_eigen
cimport eigen.eigen as eigen

cimport sva.c_sva as c_sva
cimport sva.sva as sva

cimport rbdyn.c_rbdyn as c_rbdyn
cimport rbdyn.rbdyn as rbdyn

cimport sch.sch as sch
cimport tasks.qp.qp

from cython.operator cimport preincrement as preinc
from cython.operator cimport dereference as deref
from libcpp.map cimport map as cppmap
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

import json
import warnings

def deprecated():
  warnings.simplefilter('always', category=DeprecationWarning)
  warnings.warn("This call is deprecated", DeprecationWarning)
  warnings.simplefilter('ignore', category=DeprecationWarning)

# Hold python object that have to be kept alive
global __MODULE_OBJECTS__
__MODULE_OBJECTS__ = []

cdef class Collision(object):
  def __copyctor__(self, Collision other):
    self.impl = other.impl
  def __cinit__(self, *args):
    if len(args) == 1 and isinstance(args[0], Collision):
      self.__copyctor__(args[0])
    elif len(args) == 0:
      self.impl = c_mc_rbdyn.Collision()
    elif len(args) == 5:
      self.impl = c_mc_rbdyn.Collision(args[0], args[1], args[2], args[3],
          args[4])
    else:
      raise TypeError("Invalid arguments passed to Collision ctor")

  def isNone(self):
    return self.impl.isNone()

  property body1:
    def __get__(self):
      return self.impl.body1
    def __set__(self, value):
      self.impl.body1 = value
  property body2:
    def __get__(self):
      return self.impl.body2
    def __set__(self, value):
      self.impl.body2 = value
  property iDist:
    def __get__(self):
      return self.impl.iDist
    def __set__(self, value):
      self.impl.iDist = value
  property sDist:
    def __get__(self):
      return self.impl.sDist
    def __set__(self, value):
      self.impl.sDist = value
  property damping:
    def __get__(self):
      return self.impl.damping
    def __set__(self, value):
      self.impl.damping = value

  def __richcmp__(Collision self, Collision other, int op):
    if op == 2:
      return self.impl == other.impl
    elif op == 3:
      return self.impl != other.impl
    else:
      raise NotImplementedError("This comparison is not supported")

  def __str__(self):
    return c_mc_rbdyn.CollisionToString(self.impl)
  def __repr__(self):
    return c_mc_rbdyn.CollisionToString(self.impl)

cdef Collision CollisionFromC(const c_mc_rbdyn.Collision & col):
  cdef Collision ret = Collision()
  ret.impl = c_mc_rbdyn.Collision(col)
  return ret

cdef class Flexibility(object):
  def __cinit__(self, *args):
    if len(args) == 4:
      self.jointName = args[0]
      self.K = args[1]
      self.C = args[2]
      self.O = args[3]
    elif len(args) != 0:
      raise TypeError("Invalid arguments passed to Flexibility ctor")
  property jointName:
    def __get__(self):
      return self.impl.jointName
    def __set__(self, value):
      self.impl.jointName = value
  property K:
    def __get__(self):
      return self.impl.K
    def __set__(self, value):
      self.impl.K = value
  property C:
    def __get__(self):
      return self.impl.C
    def __set__(self, value):
      self.impl.C = value
  property O:
    def __get__(self):
      return self.impl.O
    def __set__(self, value):
      self.impl.O = value

cdef Flexibility FlexibilityFromC(const c_mc_rbdyn.Flexibility & flex):
  cdef Flexibility ret = Flexibility()
  ret.impl = c_mc_rbdyn.Flexibility(flex)
  return ret

cdef class BodySensor(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, skip_alloc = False):
    self.__own_impl = not skip_alloc
    if skip_alloc:
      self.impl = NULL
      return
    self.impl = new c_mc_rbdyn.BodySensor()
  def name(self):
    return self.impl.name()
  def parentBody(self):
    return self.impl.parentBody()
  def X_b_s(self):
    return sva.PTransformdFromC(self.impl.X_b_s())
  def position(self):
    return eigen.Vector3dFromC(self.impl.position())
  def orientation(self):
    return eigen.QuaterniondFromC(self.impl.orientation())
  def linearVelocity(self):
    return eigen.Vector3dFromC(self.impl.linearVelocity())
  def angularVelocity(self):
    return eigen.Vector3dFromC(self.impl.angularVelocity())
  def acceleration(self):
    return eigen.Vector3dFromC(self.impl.acceleration())

cdef BodySensor BodySensorFromRef(c_mc_rbdyn.BodySensor & bs):
    cdef BodySensor ret = BodySensor(skip_alloc = True)
    ret.impl = &(bs)
    return ret

cdef class ForceSensor(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, string sn, string bn, sva.PTransformd X_p_f):
    self.impl = new c_mc_rbdyn.ForceSensor(sn, bn, deref(X_p_f.impl))
  def __cinit__(self, *args, skip_alloc = False):
    self.__own_impl = not skip_alloc
    if skip_alloc:
      assert(len(args) ==0)
      self.impl = NULL
      return
    if len(args) == 0:
      self.impl = new c_mc_rbdyn.ForceSensor()
    elif len(args) == 3:
      self.__ctor__(*args)
    else:
      raise TypeError("Invalid arguments passed to ForceSensor ctor")
  def name(self):
    return self.impl.name()
  def parentBody(self):
    return self.impl.parentBody()
  def X_p_f(self):
    return sva.PTransformdFromC(self.impl.X_p_f(), copy = False)
  def wrench(self):
    return sva.ForceVecdFromC(self.impl.wrench(), copy = False)
  def mass(self):
    return self.impl.mass()
  def removeGravity(self, Robot robot):
    return sva.ForceVecdFromC(self.impl.removeGravity(deref(robot.impl)))

cdef ForceSensor ForceSensorFromRef(c_mc_rbdyn.ForceSensor & fs):
    cdef ForceSensor ret = ForceSensor(skip_alloc = True)
    ret.impl = &(fs)
    return ret

cdef ForceSensor ForceSensorFromCRef(const c_mc_rbdyn.ForceSensor & fs):
    cdef ForceSensor ret = ForceSensor(skip_alloc = True)
    ret.impl = &(c_mc_rbdyn.const_cast_force_sensor(fs))
    return ret

cdef class Springs(object):
  def __cinit__(self, *args):
    if len(args) != 0:
      raise TypeError("Invalid arguments passed to Springs ctor")

  property springsBodies:
    def __get__(self):
      return self.impl.springsBodies
    def __set__(self, value):
      self.impl.springsBodies = value
  property afterSpringsBodies:
    def __get__(self):
      return self.impl.afterSpringsBodies
    def __set__(self, value):
      self.impl.afterSpringsBodies = value
  property springsJoints:
    def __get__(self):
      return self.impl.springsJoints
    def __set__(self, value):
      self.impl.springsJoints = value

cdef Springs SpringsFromC(const c_mc_rbdyn.Springs & sp):
  cdef Springs ret = Springs()
  ret.impl = c_mc_rbdyn.Springs(sp)
  return ret

cdef class Base(object):
  def __copyctor__(self, Base other):
    self.impl = other.impl
  def __cinit__(self, *args):
    if len(args) == 0:
      pass
    elif len(args) == 1:
      self.__copyctor__(args[0])
    elif len(args) == 4:
      self.baseName = args[0]
      self.X_0_s = args[1]
      self.X_b0_s = args[2]
      self.baseType = args[3]
    else:
      raise TypeError("Cannot build base from this arguments")
  property baseName:
    def __get__(self):
      return self.impl.baseName
    def __set__(self, value):
      self.impl.baseName = value
  property X_0_s:
    def __get__(self):
      return sva.PTransformdFromC(self.impl.X_0_s, copy = False)
    def __set__(self, sva.PTransformd value):
      self.impl.X_0_s = deref(value.impl)
  property X_b0_s:
    def __get__(self):
      return sva.PTransformdFromC(self.impl.X_b0_s, copy = False)
    def __set__(self, sva.PTransformd value):
      self.impl.X_b0_s = deref(value.impl)
  property baseType:
    def __get__(self):
      return self.impl.baseType
    def __set__(self, value):
      self.impl.baseType = value

cdef class RobotModule(object):
  def __cinit__(self, *args):
    if len(args) != 0:
      raise TypeError("Wrong argument passed to RobotModule ctor")

  def bounds(self):
    assert(self.impl.get())
    return deref(self.impl).bounds()
  def stance(self):
    assert(self.impl.get())
    return deref(self.impl).stance()
  def convexHull(self):
    assert(self.impl.get())
    return deref(self.impl).convexHull()
  def stpbvHull(self):
    assert(self.impl.get())
    return deref(self.impl).stpbvHull()
  def collisionTransforms(self):
    assert(self.impl.get())
    end = deref(self.impl)._collisionTransforms.end()
    it = deref(self.impl)._collisionTransforms.begin()
    ret = {}
    while it != end:
      ret[deref(it).first] = sva.PTransformdFromC(deref(it).second)
      preinc(it)
    return ret
  def flexibility(self):
    assert(self.impl.get())
    end = deref(self.impl)._flexibility.end()
    it = deref(self.impl)._flexibility.begin()
    ret = []
    while it != end:
      ret.append(FlexibilityFromC(deref(it)))
      preinc(it)
    return ret
  def forceSensors(self):
    assert(self.impl.get())
    end = deref(self.impl)._forceSensors.end()
    it = deref(self.impl)._forceSensors.begin()
    ret = []
    while it != end:
      ret.append(ForceSensorFromCRef(deref(it)))
      preinc(it)
    return ret
  def springs(self):
    assert(self.impl.get())
    return SpringsFromC(deref(self.impl).springs())
  def minimalSelfCollisions(self):
    assert(self.impl.get())
    end = deref(self.impl)._minimalSelfCollisions.end()
    it = deref(self.impl)._minimalSelfCollisions.begin()
    ret = []
    while it != end:
      ret.append(CollisionFromC(deref(it)))
      preinc(it)
    return ret
  def commonSelfCollisions(self):
    assert(self.impl.get())
    end = deref(self.impl)._commonSelfCollisions.end()
    it = deref(self.impl)._commonSelfCollisions.begin()
    ret = []
    while it != end:
      ret.append(CollisionFromC(deref(it)))
      preinc(it)
    return ret
  def ref_joint_order(self):
    assert(self.impl.get())
    return deref(self.impl).ref_joint_order()
  def default_attitude(self):
    assert(self.impl.get())
    return c_mc_rbdyn.robotModuleDefaultAttitude(self.impl)
  property path:
    def __get__(self):
      assert(self.impl.get())
      return deref(self.impl).path
  property name:
    def __get__(self):
      assert(self.impl.get())
      return deref(self.impl).name
  property urdf_path:
    def __get__(self):
      assert(self.impl.get())
      return deref(self.impl).urdf_path
  property rsdf_dir:
    def __get__(self):
      assert(self.impl.get())
      return deref(self.impl).rsdf_dir
  @property
  def calib_dir(self):
    assert(self.impl.get())
    return deref(self.impl).calib_dir
  property mb:
    def __get__(self):
      assert(self.impl.get())
      return rbdyn.MultiBodyFromC(deref(self.impl).mb, copy = False)
  property mbc:
    def __get__(self):
      assert(self.impl.get())
      return rbdyn.MultiBodyConfigFromC(deref(self.impl).mbc, copy = False)
  property mbg:
    def __get__(self):
      assert(self.impl.get())
      return rbdyn.MultiBodyGraphFromC(deref(self.impl).mbg, copy = False)

cdef RobotModule RobotModuleFromC(const c_mc_rbdyn.RobotModulePtr v):
  cdef RobotModule ret = RobotModule()
  ret.impl = v
  return ret

cdef class RobotModuleVector(object):
  def __addRM(self, RobotModule rm):
    self.v.push_back(rm.impl)
  def __cinit__(self, *args):
    if len(args) == 1:
      if isinstance(args[0], RobotModule):
        self.__addRM(args[0])
      elif isinstance(args[0], list):
        for rm in args[0]:
          self.__addRM(rm)
    else:
      for rm in args:
        self.__addRM(rm)


def get_robot_module(string name, *args):
  if len(args) == 0:
    return RobotModuleFromC(c_mc_rbdyn.get_robot_module(name))
  elif len(args) == 1:
    return RobotModuleFromC(c_mc_rbdyn.get_robot_module(name, args[0]))
  elif len(args) == 2:
    return RobotModuleFromC(c_mc_rbdyn.get_robot_module(name, args[0], args[1]))
  else:
    raise TypeError("Wrong arguments passed to get_robot_module")

cdef class Robots(object):
  def __copyctor__(self, Robots other):
    self.impl = shared_ptr[c_mc_rbdyn.Robots](new c_mc_rbdyn.Robots(deref(other.impl.get())))
  def __cinit__(self, *args, skip_alloc = False):
    if len(args) == 0:
      if not skip_alloc:
        self.impl = shared_ptr[c_mc_rbdyn.Robots](new c_mc_rbdyn.Robots())
    elif len(args) == 1 and isinstance(args[0], Robots):
      self.__copyctor__(args[0])
    else:
      raise TypeError("Wrong arguments passed to Robots ctor")

  def robots(self):
    cdef const vector[c_mc_rbdyn.Robot] & rV = deref(self.impl).robots()
    end = deref(self.impl).robots().end()
    it = deref(self.impl).robots().begin()
    ret = []
    while it != end:
      ret.append(RobotFromC(deref(it)))
    return ret

  def load(self, RobotModule module, *args):
    cdef c_sva.PTransformd * b = NULL
    bName = ""
    if len(args):
      if isinstance(args[0], sva.PTransformd):
        b = (<sva.PTransformd>(args[0])).impl
        if len(args) > 1:
          bName = args[1]
      else:
        deprecated()
        return self.load(module, *args[1:])
    return RobotFromC(deref(self.impl).load(deref(module.impl), b, bName))

  def mbs(self):
    return rbdyn.MultiBodyVectorFromPtr(&(deref(self.impl).mbs()))

  def mbcs(self):
    return rbdyn.MultiBodyConfigVectorFromPtr(&(deref(self.impl).mbcs()))

  def robotIndex(self):
    return deref(self.impl).robotIndex()
  def envIndex(self):
    return deref(self.impl).envIndex()

  def robot(self, idx = None):
    if idx is None:
      return RobotFromC(deref(self.impl).robot())
    else:
      return RobotFromC(deref(self.impl).robot(idx))
  def env(self):
    return RobotFromC(deref(self.impl).env())

cdef Robots RobotsFromPtr(shared_ptr[c_mc_rbdyn.Robots] p):
    cdef Robots ret = Robots(skip_alloc = True)
    ret.impl = p
    return ret

cdef Robots RobotsFromRawPtr(c_mc_rbdyn.Robots * p):
    cdef Robots ret = Robots(skip_alloc = True)
    ret.impl = c_mc_rbdyn.robots_fake_shared(p)
    return ret

cdef Robots RobotsFromRef(c_mc_rbdyn.Robots & p):
    return RobotsFromRawPtr(&p)

cdef class Robot(object):
  def __is_valid(self):
    assert self.impl, "This Robot instance has not been initialized correctly"
  def __cinit__(self, *args):
    if len(args):
      raise TypeError("You cannot create a stand-alone Robot, please go through a Robots")
    self.impl = NULL
  def name(self):
    self.__is_valid()
    return self.impl.name()
  def hasJoint(self, name):
    self.__is_valid()
    return self.impl.hasJoint(name)
  def hasBody(self, name):
    self.__is_valid()
    return self.impl.hasBody(name)
  def jointIndexByName(self, name):
    self.__is_valid()
    return self.impl.jointIndexByName(name)
  def bodyIndexByName(self, name):
    self.__is_valid()
    return self.impl.bodyIndexByName(name)

  def forceSensor(self, name):
    self.__is_valid()
    return ForceSensorFromRef(self.impl.forceSensor(name))
  def hasForceSensor(self, name):
    self.__is_valid()
    return self.impl.hasForceSensor(name)
  def bodyForceSensor(self, name):
    self.__is_valid()
    return ForceSensorFromRef(self.impl.bodyForceSensor(name))
  def bodyHasForceSensor(self, name):
    self.__is_valid()
    return self.impl.bodyHasForceSensor(name)

  def bodySensor(self, name = None):
    self.__is_valid()
    if name is None:
      return BodySensorFromRef(self.impl.bodySensor())
    else:
      return BodySensorFromRef(self.impl.bodySensor(name))
  def hasBodySensor(self, name):
    self.__is_valid()
    return self.impl.hasBodySensor(name)
  def bodyHasBodySensor(self, name):
    self.__is_valid()
    return self.impl.bodyHasBodySensor(name)
  def bodyBodySensor(self, name):
    self.__is_valid()
    return BodySensorFromRef(self.impl.bodyBodySensor(name))

  property mb:
    def __get__(self):
      self.__is_valid()
      return rbdyn.MultiBodyFromC(self.impl.mb(), False)
  property mbc:
    def __get__(self):
      self.__is_valid()
      return rbdyn.MultiBodyConfigFromC(self.impl.mbc(), False)
  property mbg:
    def __get__(self):
      self.__is_valid()
      return rbdyn.MultiBodyGraphFromC(self.impl.mbg(), False)

  property ql:
    def __get__(self):
      self.__is_valid()
      return rbdyn.DoubleVectorVectorWrapperFromC(self.impl.ql(), self)
  property qu:
    def __get__(self):
      self.__is_valid()
      return rbdyn.DoubleVectorVectorWrapperFromC(self.impl.qu(), self)
  property vl:
    def __get__(self):
      self.__is_valid()
      return rbdyn.DoubleVectorVectorWrapperFromC(self.impl.vl(), self)
  property vu:
    def __get__(self):
      self.__is_valid()
      return rbdyn.DoubleVectorVectorWrapperFromC(self.impl.vu(), self)
  property tl:
    def __get__(self):
      self.__is_valid()
      return rbdyn.DoubleVectorVectorWrapperFromC(self.impl.tl(), self)
  property tu:
    def __get__(self):
      self.__is_valid()
      return rbdyn.DoubleVectorVectorWrapperFromC(self.impl.tu(), self)

  def flexibility(self):
    self.__is_valid()
    end = deref(self.impl).flexibility().end()
    it = deref(self.impl).flexibility().begin()
    ret = []
    while it != end:
      ret.append(FlexibilityFromC(deref(it)))
      preinc(it)
    return ret

  def hasSurface(self, name):
    self.__is_valid()
    return self.impl.hasSurface(name)
  def surface(self, name):
    self.__is_valid()
    return SurfaceFromC(self.impl.surface(name))
  def surfaces(self):
    self.__is_valid()
    availableSurfaces = self.impl.availableSurfaces()
    ret = {}
    for s in availableSurfaces:
      ret[s] = SurfaceFromC(self.impl.surface(s))
    return ret
  def copySurface(self, sName, name):
    self.__is_valid()
    return SurfaceFromC(self.impl.copySurface(sName, name))

  def convex(self, name):
    self.__is_valid()
    return (self.impl.convex(name).first,sch.S_PolyhedronFromPtr(self.impl.convex(name).second))

  def bodyTransform(self, bName):
    self.__is_valid()
    return sva.PTransformdFromC(self.impl.bodyTransform(bName), False)

  def collisionTransform(self, bName):
    self.__is_valid()
    return sva.PTransformdFromC(self.impl.collisionTransform(bName), False)

  def fixSurfaces(self):
    self.__is_valid()
    self.impl.fixSurfaces()

  def loadRSDFFromDir(self, surfaceDir):
    self.__is_valid()
    self.impl.loadRSDFFromDir(surfaceDir)

  def stance(self):
    self.__is_valid()
    return self.impl.stance()

cdef Robot RobotFromC(const c_mc_rbdyn.Robot & robot):
  cdef Robot ret = Robot()
  ret.impl = &(c_mc_rbdyn.const_cast_robot(robot))
  return ret

cdef class Surface(object):
  def __cinit__(self):
    pass

  property name:
    def __get__(self):
      assert(self.impl)
      return self.impl.name()
  property bodyName:
    def __get__(self):
      assert(self.impl)
      return self.impl.bodyName()
  property materialName:
    def __get__(self):
      assert(self.impl)
      return self.impl.materialName()

  def points(self):
    assert(self.impl)
    # Force the C++ compiler to use the const variant
    return sva.PTransformdVectorFromC((<const c_mc_rbdyn.Surface*>(self.impl)).points())

  def bodyIndex(self, Robot robot):
    assert(self.impl)
    return self.impl.bodyIndex(deref(robot.impl))

  def X_0_s(self, Robot robot, rbdyn.MultiBodyConfig mbc = None):
    assert(self.impl)
    if mbc is None:
      return sva.PTransformdFromC(self.impl.X_0_s(deref(robot.impl)))
    else:
      return sva.PTransformdFromC(self.impl.X_0_s(deref(robot.impl),
        deref(mbc.impl)))

  property X_b_s:
    def __get__(self):
      assert(self.impl)
      return sva.PTransformdFromC(self.impl.X_b_s(), copy = False)
    def __set__(self, sva.PTransformd _x):
      assert(self.impl)
      self.impl.X_b_s(deref(_x.impl))

  def computePoints(self):
    assert(self.impl)
    self.impl.computePoints()

  def toStr(self):
    assert(self.impl)
    return self.impl.toStr()
  def __repr__(self):
    assert(self.impl)
    return self.toStr()
  def __hash__(self):
    return hash(str(self))

  def copy(self):
    assert(self.impl)
    return SurfaceFromPtr(self.impl.copy(), keep_ptr = True)

  def type(self):
    assert(self.impl)
    return self.impl.type()

  def __richcmp__(Surface self, Surface other, int op):
    assert(self.impl)
    assert(other.impl)
    if op == 2:
      return deref(self.impl) == deref(other.impl)
    elif op == 3:
      return deref(self.impl) != deref(other.impl)
    else:
      raise NotImplementedError("This comparison is not supported")

cdef SurfaceFromC(const c_mc_rbdyn.Surface & surface):
  cdef Surface ret = Surface()
  cdef c_mc_rbdyn.Surface * ptr = &(c_mc_rbdyn.const_cast_surface(surface))
  s_type = str(ptr.type())
  if s_type == "planar":
    ret = PlanarSurfaceFromPtr(c_mc_rbdyn.dynamic_cast_planar_surface(ptr))
  elif s_type == "gripper":
    ret = GripperSurfaceFromPtr(c_mc_rbdyn.dynamic_cast_gripper_surface(ptr))
  elif s_type == "cylindrical":
    ret = CylindricalSurfaceFromPtr(c_mc_rbdyn.dynamic_cast_cylindrical_surface(ptr))
  else:
    print "Unknown Surface type:",ptr.type()
    ret.impl = ptr
  return ret

cdef SurfaceFromPtr(shared_ptr[c_mc_rbdyn.Surface] ptr, cppbool keep_ptr = True):
  cdef Surface ret = Surface()
  s_type = str(ptr.get().type())
  if s_type == "planar":
    ret = PlanarSurfaceFromPtr(c_mc_rbdyn.dynamic_cast_planar_surface(ptr.get()))
  elif s_type == "gripper":
    ret = GripperSurfaceFromPtr(c_mc_rbdyn.dynamic_cast_gripper_surface(ptr.get()))
  elif s_type == "cylindrical":
    ret = CylindricalSurfaceFromPtr(c_mc_rbdyn.dynamic_cast_cylindrical_surface(ptr.get()))
  else:
    print "Unknown Surface type:",s_type
    ret.impl = ptr.get()
  if keep_ptr:
    ret.ptr = ptr
  return ret

cdef class PlanarSurface(Surface):
  def __cinit__(self):
    pass
  def planarTransform(self, T, B, N):
    assert(self.surf)
    self.surf.planarTransform(T, B, N)
  property planarPoints:
    def __get__(self):
      assert(self.surf)
      return self.surf.planarPoints()
    def __set__(self, points):
      assert(self.surf)
      self.surf.planarPoints(points)

cdef PlanarSurface PlanarSurfaceFromPtr(c_mc_rbdyn.PlanarSurface * surf):
    cdef PlanarSurface ret = PlanarSurface()
    ret.surf = ret.impl = surf
    return ret

cdef class GripperSurface(Surface):
  def __cinit__(self):
    pass
  def originTransform(self, sva.PTransformd X_s_sp):
    assert(self.surf)
    self.surf.originTransform(deref(X_s_sp.impl))
  property pointsFromOrigin:
    def __get__(self):
      assert(self.surf)
      return sva.PTransformdVectorFromC(self.surf.pointsFromOrigin())
  property X_b_motor:
    def __get__(self):
      assert(self.surf)
      return sva.PTransformdFromC(self.surf.X_b_motor())
  property motorMaxTorque:
    def __get__(self):
      assert(self.surf)
      return self.surf.motorMaxTorque()

cdef GripperSurface GripperSurfaceFromPtr(c_mc_rbdyn.GripperSurface * surf):
    cdef GripperSurface ret = GripperSurface()
    ret.surf = ret.impl = surf
    return ret

cdef class CylindricalSurface(Surface):
  def __cinit__(self):
    pass
  property radius:
    def __get__(self):
      assert(self.surf)
      return self.surf.radius()
  property width:
    def __get__(self):
      assert(self.surf)
      return self.surf.width()

cdef CylindricalSurface CylindricalSurfaceFromPtr(c_mc_rbdyn.CylindricalSurface * surf):
    cdef CylindricalSurface ret = CylindricalSurface()
    ret.surf = ret.impl = surf
    return ret

def readRSDFFromDir(dirname):
  cdef vector[shared_ptr[c_mc_rbdyn.Surface]] surfs = c_mc_rbdyn.readRSDFFromDir(dirname)
  ret = []
  for surf in surfs:
    ret.append(SurfaceFromPtr(surf, keep_ptr = True))
  return ret

cdef class Contact(object):
  nrConeGen = c_mc_rbdyn.ContactnrConeGen
  defaultFriction = c_mc_rbdyn.ContactdefaultFriction
  nrBilatPoints = c_mc_rbdyn.ContactnrBilatPoints
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __copyctor__(self, Contact other):
    self.impl = new c_mc_rbdyn.Contact(deref(other.impl))
  def __robotspt_ctor__(self, Robots robots, robotSurface, envSurface,
      sva.PTransformd X_es_rs = None):
    if X_es_rs is None:
      self.impl = new c_mc_rbdyn.Contact(deref(robots.impl), robotSurface,
          envSurface)
    else:
      self.impl = new c_mc_rbdyn.Contact(deref(robots.impl), robotSurface,
          envSurface, deref(X_es_rs.impl))
  def __full_ctor__(self, Robots robots, r1Index, r2Index, r1Surface, r2Surface,
      sva.PTransformd X_r2s_r1s = None, sva.PTransformd Xbs =
      sva.PTransformd.Identity(), ambId = -1):
    cdef c_sva.PTransformd * _x_r2s_r1s = NULL
    if not X_r2s_r1s is None:
      _x_r2s_r1s = X_r2s_r1s.impl
    self.impl = new c_mc_rbdyn.Contact(deref(robots.impl), r1Index, r2Index,
        r1Surface, r2Surface, _x_r2s_r1s, deref(Xbs.impl), ambId)
  def __cinit__(self, *args, **kwds):
    if "skip_alloc" in kwds:
      skip_alloc = bool(kwds["skip_alloc"])
    else:
      skip_alloc = False
    self.__own_impl = True
    if len(args) == 0 and skip_alloc:
      self.impl = NULL
      pass
    elif len(args) == 1 and isinstance(args[0], Contact):
      self.__copyctor__(args[0])
    elif len(args) > 2 and isinstance(args[0], Robots):
      if len(args) == 3 or (len(args) == 4 and args[3] is None):
        self.__robotspt_ctor__(args[0], args[1], args[2])
      elif len(args) == 4 and isinstance(args[3], sva.PTransformd):
        self.__robotspt_ctor__(args[0], args[1], args[2], args[3])
      elif len(args) >= 5:
        X_r2s_r1s = None
        if len(args) >= 6 or "X_r2s_r1s" in kwds:
          if "X_r2s_r1s" in kwds:
            X_r2s_r1s = kwds["X_r2s_r1s"]
          else:
            X_r2s_r1s = args[5]
        X_b_s = sva.PTransformd.Identity()
        if len(args) >= 7 or "X_b_s" in kwds:
          if "X_b_s" in kwds:
            X_b_s = kwds["X_b_s"]
          else:
            X_b_s = args[6]
        ambiguityId = -1
        if len(args) >= 8 or "ambiguityId" in kwds:
          if "ambiguityId" in kwds:
            ambiguityId = kwds["ambiguityId"]
          else:
            ambiguityId = args[7]
        self.__full_ctor__(args[0], args[1], args[2], args[3], args[4],
            X_r2s_r1s, X_b_s, ambiguityId)
      else:
        raise TypeError("Wrong arguments provided to Contact ctor")
    else:
      raise TypeError("Wrong arguments provided to Contact ctor")
  def __copy__(self):
    return Contact(self)
  def __deepcopy__(self, memo):
    return Contact(self)
  def r1Index(self):
    return self.impl.r1Index()
  def r2Index(self):
    return self.impl.r2Index()
  def r1Surface(self):
    return SurfaceFromPtr(self.impl.r1Surface())
  def r2Surface(self):
    return SurfaceFromPtr(self.impl.r2Surface())
  def X_r2s_r1s(self, sva.PTransformd _x = None):
    if _x is None:
      return sva.PTransformdFromC(self.impl.X_r2s_r1s())
    else:
      self.impl.X_r2s_r1s(deref(_x.impl))
  def X_b_s(self):
    return sva.PTransformdFromC(self.impl.X_b_s())
  def ambiguityId(self):
    return self.impl.ambiguityId()
  def isFixed(self):
    return self.impl.isFixed()
  def surfaces(self):
    return self.impl.surfaces()
  def X_0_r1s(self, robots):
    if isinstance(robots, Robot):
      return sva.PTransformdFromC(self.impl.X_0_r1s(deref((<Robot>robots).impl)))
    elif isinstance(robots, Robots):
      return sva.PTransformdFromC(self.impl.X_0_r1s(deref((<Robots>robots).impl)))
    else:
      raise TypeError("Wrong argument passed to X_0_r1s, expecting Robots or Robot")
  def X_0_r2s(self, robots):
    if isinstance(robots, Robot):
      return sva.PTransformdFromC(self.impl.X_0_r2s(deref((<Robot>robots).impl)))
    elif isinstance(robots, Robots):
      return sva.PTransformdFromC(self.impl.X_0_r2s(deref((<Robots>robots).impl)))
    else:
      raise TypeError("Wrong argument passed to X_0_r2s, expecting Robots or Robot")
  def r1Points(self):
    return sva.PTransformdVectorFromC(self.impl.r1Points())
  def r2Points(self):
    return sva.PTransformdVectorFromC(self.impl.r2Points())
  def compute_X_r2s_r1s(self, Robots robots):
    return sva.PTransformdFromC(self.impl.compute_X_r2s_r1s(deref(robots.impl)))
  def contactId(self, Robots robots):
    return tasks.qp.qp.ContactIdFromC(self.impl.contactId(deref(robots.impl)))
  def toStr(self):
    return self.impl.toStr()
  def __repr__(self):
    return self.toStr()
  def __richcmp__(Contact self, Contact other, int op):
    if op == 2:
      return deref(self.impl) == deref(other.impl)
    elif op == 3:
      return deref(self.impl) != deref(other.impl)
    else:
      raise NotImplementedError("This comparison is not supported")
  def __hash__(self):
    return hash(str(self))

cdef Contact ContactFromC(const c_mc_rbdyn.Contact& c, cppbool copy=True):
  cdef Contact ret = Contact(skip_alloc = True)
  if copy:
    ret.impl = new c_mc_rbdyn.Contact(c)
  else:
    ret.__own_impl = False
    ret.impl = &(c_mc_rbdyn.const_cast_contact(c))
  return ret

cdef class ContactVector(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.v
  def __push_contact(self, Contact c):
    self.v.push_back(deref(c.impl))
  def __cinit__(self, *args, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.v = new vector[c_mc_rbdyn.Contact]()
    if len(args) == 1:
      if isinstance(args[0], list):
        for c in args[0]:
          self.__push_contact(c)
      elif isinstance(args[0], Contact):
        self.__push_contact(args[0])
      elif isinstance(args[0], ContactVector):
        self.v = new vector[c_mc_rbdyn.Contact](deref((<ContactVector>args[0]).v))
      else:
        raise TypeError("Wrong arguments passed to ContactVector ctor")
    else:
      for c in args:
        self.__push_contact(c)
  def __getitem__(self, idx):
    if isinstance(idx, slice):
      [start, stop, step] = idx.start, idx.stop, idx.step
      if start is None:
        start = 0
      if stop is None:
        stop = self.v.size()
      if step is None:
        step = 1
      ret = []
      for i in xrange(start, stop, step):
        ret.append(ContactFromC(self.v.at(i), copy = False))
      return ret
    else:
      if idx == -1:
        idx = self.v.size() - 1
      return ContactFromC(self.v.at(idx), copy = False)
  def __setitem__(self, idx, Contact v):
    c_mc_rbdyn.contact_vector_set_item(deref(self.v), idx, deref(v.impl))
  def __add__(ContactVector self, other):
    cdef ContactVector ret = ContactVector(self)
    for c in other:
      ret.__push_contact(c)
    return ret
  def __iadd__(self, other):
    for c in other:
      self.__push_contact(c)
    return self
  def __len__(self):
    return self.v.size()
  def __copy__(self):
    return ContactVector(self)
  def __deepcopy__(self, memo):
    return ContactVector(self)

cdef ContactVector ContactVectorFromC(const vector[c_mc_rbdyn.Contact]& v,
    cppbool copy=True):
  cdef ContactVector ret = ContactVector(skip_alloc = True)
  if copy:
    ret.v = new vector[c_mc_rbdyn.Contact](v)
  else:
    ret.__own_impl = False
    ret.v = &(c_mc_rbdyn.const_cast_contact_vector(v))
  return ret

cdef class Stance(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __copyctor__(self, Stance st):
    self.impl = new c_mc_rbdyn.Stance(st.impl.q(), st.impl.geomContacts(), st.impl.stabContacts())
  def __ctor__(self, q, ContactVector geomContacts, ContactVector stabContacts):
    self.impl = new c_mc_rbdyn.Stance(q, deref(geomContacts.v),
        deref(stabContacts.v))
  def __cinit__(self, *args, skip_alloc = False):
    self.__own_impl = True
    if len(args) == 0 and skip_alloc:
      pass
    elif len(args) == 1 and isinstance(args[0], Stance):
      self.__copyctor__(args[0])
    elif len(args) == 3:
      if args[0] is None:
        args = ([], args[1], args[2])
      if not isinstance(args[1], ContactVector):
        args = (args[0], ContactVector(args[1]), args[2])
      if not isinstance(args[2], ContactVector):
        args = (args[0], args[1], ContactVector(args[2]))
      self.__ctor__(args[0], args[1], args[2])
    else:
      raise TypeError("Wrong arguments passed to Stance ctor")
  def __copy__(self):
    return Stance(self)
  def __deepcopy__(self, memo):
    return Stance(self)
  def contacts(self):
    return ContactVectorFromC(self.impl.contacts(), copy=False)
  def q(self):
    return self.impl.q()
  property geomContacts:
    def __get__(self):
      return ContactVectorFromC(self.impl.geomContacts(), copy=False)
    def __set__(self, value):
      if not isinstance(value, ContactVector):
        value = ContactVector(value)
      self.impl.geomContacts(deref((<ContactVector>value).v))
  property stabContacts:
    def __get__(self):
      return ContactVectorFromC(self.impl.stabContacts(), copy=False)
    def __set__(self, value):
      if not isinstance(value, ContactVector):
        value = ContactVector(value)
      self.impl.stabContacts(deref((<ContactVector>value).v))
  def updateContact(self, Contact oldC, Contact newC):
    self.impl.updateContact(deref(oldC.impl), deref(newC.impl))
  def com(self, Robot robot):
    return eigen.Vector3dFromC(self.impl.com(deref(robot.impl)))
  def robotSurfacesInContact(self):
    return self.impl.robotSurfacesInContact()

cdef Stance StanceFromC(const c_mc_rbdyn.Stance& s):
  cdef Stance ret = Stance(skip_alloc = True)
  ret.__own_impl = False
  ret.impl = &(c_mc_rbdyn.const_cast_stance(s))
  return ret

cdef class StanceVector(object):
  def __dealloc__(self):
    del self.v
  def __cinit__(self):
    pass
  def __getitem__(self, idx):
    return StanceFromC(self.v.at(idx))
  def __len__(self):
    return self.v.size()

cdef class StanceAction(object):
  def __cinit__(self):
    if type(self) is StanceAction:
      raise TypeError("StanceAction cannot be instantiated")
  def apply(self, Stance s):
    cdef c_mc_rbdyn.apply_return_t res = self.base.apply(deref(s.impl))
    return ((ContactVectorFromC(res.first.first),
             ContactVectorFromC(res.first.second)),
            (ContactVectorFromC(res.second.first),
             ContactVectorFromC(res.second.second)))
  def toStr(self):
    return self.base.toStr()
  def __repr__(self):
    return self.toStr()
  def type(self):
    return self.base.type()
  property contact:
    def __get__(self):
      return ContactFromC(self.base.contact())

cdef class IdentityContactAction(StanceAction):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = self.base = new c_mc_rbdyn.IdentityContactAction()

cdef class AddContactAction(StanceAction):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, Contact c = None, skip_alloc = False):
    self.__own_impl = True
    if skip_alloc and c is None:
      pass
    elif not c is None:
      self.impl = self.base = new c_mc_rbdyn.AddContactAction(deref(c.impl))
    else:
      raise TypeError("Wrong arguments passed to AddContactAction ctor")

cdef class RemoveContactAction(StanceAction):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, Contact c = None, skip_alloc = False):
    self.__own_impl = True
    if skip_alloc and c is None:
      pass
    elif not c is None:
      self.impl = self.base = new c_mc_rbdyn.RemoveContactAction(deref(c.impl))
    else:
      raise TypeError("Wrong arguments passed to RemoveContactAction ctor")

cdef StanceActionFromPtr(shared_ptr[c_mc_rbdyn.StanceAction] p):
  cdef IdentityContactAction ia = IdentityContactAction(skip_alloc = True)
  cdef c_mc_rbdyn.IdentityContactAction * ia_ptr = NULL
  cdef AddContactAction ac = AddContactAction(skip_alloc = True)
  cdef c_mc_rbdyn.AddContactAction * ac_ptr = NULL
  cdef RemoveContactAction rc = RemoveContactAction(skip_alloc = True)
  cdef c_mc_rbdyn.RemoveContactAction * rc_ptr = NULL
  if p.get() != NULL:
    sa_type = deref(p.get()).type()
  else:
    sa_type = "NULL"
  if sa_type == "identity":
    ia_ptr = c_mc_rbdyn.dynamic_cast_ica(p.get())
    assert(ia_ptr != NULL)
    ia.base = ia.impl = new c_mc_rbdyn.IdentityContactAction(deref(ia_ptr))
    return ia
  elif sa_type == "add":
    ac_ptr = c_mc_rbdyn.dynamic_cast_aca(p.get())
    assert(ac_ptr != NULL)
    ac.base = ac.impl = new c_mc_rbdyn.AddContactAction(deref(ac_ptr))
    return ac
  elif sa_type == "remove":
    rc_ptr = c_mc_rbdyn.dynamic_cast_rca(p.get())
    assert(rc_ptr != NULL)
    rc.base = rc.impl = new c_mc_rbdyn.RemoveContactAction(deref(rc_ptr))
    return rc
  else:
    raise TypeError("Unknown StanceAction type: " + sa_type)

cdef class StanceConfigCoMTask(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, stiffness = 2.0, extraStiffness = 0.0, weight = 500.0,
      targetSpeed = 0.003, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = new c_mc_rbdyn.StanceConfigCoMTask()
      self.impl.stiffness = stiffness
      self.impl.extraStiffness = extraStiffness
      self.impl.weight = weight
      self.impl.targetSpeed = targetSpeed
  property stiffness:
    def __get__(self):
      return self.impl.stiffness
    def __set__(self, value):
      self.impl.stiffness = value
  property extraStiffness:
    def __get__(self):
      return self.impl.extraStiffness
    def __set__(self, value):
      self.impl.extraStiffness = value
  property weight:
    def __get__(self):
      return self.impl.weight
    def __set__(self, value):
      self.impl.weight = value
  property targetSpeed:
    def __get__(self):
      return self.impl.targetSpeed
    def __set__(self, value):
      self.impl.targetSpeed = value

cdef StanceConfigCoMTask StanceConfigCoMTaskFromPtr(c_mc_rbdyn.StanceConfigCoMTask * p):
    cdef StanceConfigCoMTask ret = StanceConfigCoMTask(skip_alloc = True)
    ret.__own_impl = False
    ret.impl = p
    return ret

cdef class StanceConfigCoMObj(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, posThresh = 0.1, velThresh = 0.000099, eigen.Vector3d
      comOffset = eigen.Vector3d.Zero(), timeout = 5.0, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = new c_mc_rbdyn.StanceConfigCoMObj()
      self.impl.posThresh = posThresh
      self.impl.velThresh = velThresh
      self.impl.comOffset = comOffset.impl
      self.impl.timeout = timeout
  property posThresh:
    def __get__(self):
      return self.impl.posThresh
    def __set__(self, value):
      self.impl.posThresh = value
  property velThresh:
    def __get__(self):
      return self.impl.velThresh
    def __set__(self, value):
      self.impl.velThresh = value
  property comOffset:
    def __get__(self):
      return eigen.Vector3dFromC(self.impl.comOffset)
    def __set__(self, eigen.Vector3d value):
      self.impl.comOffset = value.impl
  property timeout:
    def __get__(self):
      return self.impl.timeout
    def __set__(self, value):
      self.impl.timeout = value

cdef StanceConfigCoMObj StanceConfigCoMObjFromPtr(c_mc_rbdyn.StanceConfigCoMObj* p):
    cdef StanceConfigCoMObj ret = StanceConfigCoMObj(skip_alloc = True)
    ret.__own_impl = False
    ret.impl = p
    return ret

cdef class StanceConfigPostureTask(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, stiffness = 2.0, weight = 10.0, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = new c_mc_rbdyn.StanceConfigPostureTask()
      self.impl.stiffness = stiffness
      self.impl.weight = weight
  property stiffness:
    def __get__(self):
      return self.impl.stiffness
    def __set__(self, value):
      self.impl.stiffness = value
  property weight:
    def __get__(self):
      return self.impl.weight
    def __set__(self, value):
      self.impl.weight = value

cdef StanceConfigPostureTask StanceConfigPostureTaskFromPtr(c_mc_rbdyn.StanceConfigPostureTask* p):
    cdef StanceConfigPostureTask ret = StanceConfigPostureTask(skip_alloc = True)
    ret.__own_impl = False
    ret.impl = p
    return ret

cdef class StanceConfigPosition(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, stiffness = 2.0, extraStiffness = 5.0, weight = 300.0,
      targetSpeed = 0.001, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = new c_mc_rbdyn.StanceConfigPosition()
      self.impl.stiffness = stiffness
      self.impl.extraStiffness = extraStiffness
      self.impl.weight = weight
      self.impl.targetSpeed = targetSpeed
  property stiffness:
    def __get__(self):
      return self.impl.stiffness
    def __set__(self, value):
      self.impl.stiffness = value
  property extraStiffness:
    def __get__(self):
      return self.impl.extraStiffness
    def __set__(self, value):
      self.impl.extraStiffness = value
  property weight:
    def __get__(self):
      return self.impl.weight
    def __set__(self, value):
      self.impl.weight = value
  property targetSpeed:
    def __get__(self):
      return self.impl.targetSpeed
    def __set__(self, value):
      self.impl.targetSpeed = value

cdef StanceConfigPosition StanceConfigPositionFromPtr(c_mc_rbdyn.StanceConfigPosition * p):
    cdef StanceConfigPosition ret = StanceConfigPosition(skip_alloc = True)
    ret.__own_impl = False
    ret.impl = p
    return ret

cdef class StanceConfigOrientation(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, stiffness = 2.0, weight = 1.0, finalWeight = 1000.0,
      skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = new c_mc_rbdyn.StanceConfigOrientation()
      self.impl.stiffness = stiffness
      self.impl.weight = weight
      self.impl.finalWeight = finalWeight
  property stiffness:
    def __get__(self):
      return self.impl.stiffness
    def __set__(self, value):
      self.impl.stiffness = value
  property weight:
    def __get__(self):
      return self.impl.weight
    def __set__(self, value):
      self.impl.weight = value
  property finalWeight:
    def __get__(self):
      return self.impl.finalWeight
    def __set__(self, value):
      self.impl.finalWeight = value

cdef StanceConfigOrientation StanceConfigOrientationFromPtr(c_mc_rbdyn.StanceConfigOrientation * p):
    cdef StanceConfigOrientation ret = StanceConfigOrientation(skip_alloc = True)
    ret.__own_impl = False
    ret.impl = p
    return ret

cdef class StanceConfigLinVel(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, stiffness = 5.0, weight = 10000.0, speed = 0.05,
      skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = new c_mc_rbdyn.StanceConfigLinVel()
      self.impl.stiffness = stiffness
      self.impl.weight = weight
      self.impl.speed = speed
  property stiffness:
    def __get__(self):
      return self.impl.stiffness
    def __set__(self, value):
      self.impl.stiffness = value
  property weight:
    def __get__(self):
      return self.impl.weight
    def __set__(self, value):
      self.impl.weight = value
  property speed:
    def __get__(self):
      return self.impl.speed
    def __set__(self, value):
      self.impl.speed = value

cdef StanceConfigLinVel StanceConfigLinVelFromPtr(c_mc_rbdyn.StanceConfigLinVel * p):
    cdef StanceConfigLinVel ret = StanceConfigLinVel(skip_alloc = True)
    ret.__own_impl = False
    ret.impl = p
    return ret

cdef c_eigen.Vector3d python_to_pos_callback(const c_sva.PTransformd & start, const c_sva.PTransformd & end, const c_eigen.Vector3d & N, void * f):
  cdef eigen.Vector3d v = (<object>f)(sva.PTransformdFromC(start, copy=False), sva.PTransformdFromC(end,copy=False), eigen.Vector3dFromC(N))
  return v.impl

cdef class StanceConfigWaypointConf(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, skip = False, thresh = 0.15, pos = None, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = new c_mc_rbdyn.StanceConfigWaypointConf()
      self.impl.skip = skip
      self.impl.thresh = thresh
      if not pos is None:
        self.pos = pos
  property skip:
    def __get__(self):
      return self.impl.skip
    def __set__(self, value):
      self.impl.skip = value
  property thresh:
    def __get__(self):
      return self.impl.thresh
    def __set__(self, value):
      self.impl.thresh = value

cdef StanceConfigWaypointConf StanceConfigWaypointConfFromPtr(c_mc_rbdyn.StanceConfigWaypointConf * p):
    cdef StanceConfigWaypointConf ret = StanceConfigWaypointConf(skip_alloc = True)
    ret.__own_impl = False
    ret.impl = p
    return ret

cdef class StanceConfigCollisionConf(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, iDist = 0.01, sDist = 0.005, damping = 0.05, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = new c_mc_rbdyn.StanceConfigCollisionConf()
      self.impl.iDist = iDist
      self.impl.sDist = sDist
      self.impl.damping = damping
  property iDist:
    def __get__(self):
      return self.impl.iDist
    def __set__(self,value):
      self.impl.iDist = value
  property sDist:
    def __get__(self):
      return self.impl.sDist
    def __set__(self,value):
      self.impl.sDist = value
  property damping:
    def __get__(self):
      return self.impl.damping
    def __set__(self,value):
      self.impl.damping = value

cdef StanceConfigCollisionConf StanceConfigCollisionConfFromPtr(c_mc_rbdyn.StanceConfigCollisionConf * p):
    cdef StanceConfigCollisionConf ret = StanceConfigCollisionConf(skip_alloc = True)
    ret.__own_impl = False
    ret.impl = p
    return ret

cdef class StanceConfigContactTask(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = new c_mc_rbdyn.StanceConfigContactTask()
  property position:
    def __get__(self):
      return StanceConfigPositionFromPtr(&self.impl.position)
    def __set__(self, StanceConfigPosition v):
      self.impl.position = deref(v.impl)
  property orientation:
    def __get__(self):
      return StanceConfigOrientationFromPtr(&self.impl.orientation)
    def __set__(self, StanceConfigOrientation v):
      self.impl.orientation = deref(v.impl)
  property linVel:
    def __get__(self):
      return StanceConfigLinVelFromPtr(&self.impl.linVel)
    def __set__(self, StanceConfigLinVel v):
      self.impl.linVel = deref(v.impl)
  property waypointConf:
    def __get__(self):
      return StanceConfigWaypointConfFromPtr(&self.impl.waypointConf)
    def __set__(self, StanceConfigWaypointConf v):
      self.impl.waypointConf = deref(v.impl)
  property collisionConf:
    def __get__(self):
      return StanceConfigCollisionConfFromPtr(&self.impl.collisionConf)
    def __set__(self, StanceConfigCollisionConf v):
      self.impl.collisionConf = deref(v.impl)

cdef StanceConfigContactTask StanceConfigContactTaskFromPtr(c_mc_rbdyn.StanceConfigContactTask * p):
    cdef StanceConfigContactTask ret = StanceConfigContactTask(skip_alloc = True)
    ret.__own_impl = False
    ret.impl = p
    return ret

cdef class StanceConfigContactObj(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, posThresh = 0.03, velThresh = 0.005, adjustPosThresh =
      0.05, adjustVelThresh = 0.005, adjustOriThresh = 0.05, eigen.Vector3d
      adjustOffset = eigen.Vector3d.Zero(), eigen.Vector3d adjustOriTBNWeight =
      eigen.Vector3d(1,1,1), preContactDist = 0.05, gripperMoveAwayDist = 0.0,
      skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = new c_mc_rbdyn.StanceConfigContactObj()
      self.impl.posThresh = posThresh
      self.impl.velThresh = velThresh
      self.impl.adjustPosThresh = adjustPosThresh
      self.impl.adjustVelThresh = adjustVelThresh
      self.impl.adjustOriThresh = adjustOriThresh
      self.impl.adjustOffset = adjustOffset.impl
      self.impl.adjustOriTBNWeight = adjustOriTBNWeight.impl
      self.impl.preContactDist = preContactDist
      self.impl.gripperMoveAwayDist = gripperMoveAwayDist
  property posThresh:
    def __get__(self):
      return self.impl.posThresh
    def __set__(self, value):
      self.impl.posThresh = value
  property velThresh:
    def __get__(self):
      return self.impl.velThresh
    def __set__(self, value):
      self.impl.velThresh = value
  property adjustPosThresh:
    def __get__(self):
      return self.impl.adjustPosThresh
    def __set__(self, value):
      self.impl.adjustPosThresh = value
  property adjustVelThresh:
    def __get__(self):
      return self.impl.adjustVelThresh
    def __set__(self, value):
      self.impl.adjustVelThresh = value
  property adjustOriThresh:
    def __get__(self):
      return self.impl.adjustOriThresh
    def __set__(self, value):
      self.impl.adjustOriThresh = value
  property adjustOffset:
    def __get__(self):
      return eigen.Vector3dFromC(self.impl.adjustOffset)
    def __set__(self, eigen.Vector3d value):
      self.impl.adjustOffset = value.impl
  property adjustOriTBNWeight:
    def __get__(self):
      return eigen.Vector3dFromC(self.impl.adjustOriTBNWeight)
    def __set__(self, eigen.Vector3d value):
      self.impl.adjustOriTBNWeight = value.impl
  property preContactDist:
    def __get__(self):
      return self.impl.preContactDist
    def __set__(self, value):
      self.impl.preContactDist = value
  property gripperMoveAwayDist:
    def __get__(self):
      return self.impl.gripperMoveAwayDist
    def __set__(self, value):
      self.impl.gripperMoveAwayDist = value

cdef StanceConfigContactObj StanceConfigContactObjFromPtr(c_mc_rbdyn.StanceConfigContactObj * p):
    cdef StanceConfigContactObj ret = StanceConfigContactObj(skip_alloc = True)
    ret.__own_impl = False
    ret.impl = p
    return ret

cdef class StanceConfigBodiesCollisionConf(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, body1 = None, body2 = None, StanceConfigCollisionConf collisionConf = StanceConfigCollisionConf(), skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      assert(not body1 is None)
      assert(not body2 is None)
      self.impl = new c_mc_rbdyn.StanceConfigBodiesCollisionConf()
      self.impl.body1 = body1
      self.impl.body2 = body2
      self.impl.collisionConf = deref(collisionConf.impl)
  property body1:
    def __get__(self):
      return self.impl.body1
    def __set__(self, value):
      self.impl.body1 = value
  property body2:
    def __get__(self):
      return self.impl.body2
    def __set__(self, value):
      self.impl.body2 = value
  property collisionConf:
    def __get__(self):
      return StanceConfigCollisionConfFromPtr(&self.impl.collisionConf)
    def __set__(self, StanceConfigCollisionConf value):
      self.impl.collisionConf = deref(value.impl)

cdef StanceConfigBodiesCollisionConf StanceConfigBodiesCollisionConfFromPtr(c_mc_rbdyn.StanceConfigBodiesCollisionConf * p):
    cdef StanceConfigBodiesCollisionConf ret = StanceConfigBodiesCollisionConf(skip_alloc = True)
    ret.__own_impl = False
    ret.impl = p
    return ret

cdef class StanceConfigBodiesCollisionConfVector(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.v
  def __add_c(self, StanceConfigBodiesCollisionConf c):
    self.v.push_back(deref(c.impl))
  def __cinit__(self, *args, skip_alloc = False):
    if not skip_alloc:
      self.v = new vector[c_mc_rbdyn.StanceConfigBodiesCollisionConf]()
      if len(args) == 1 and isinstance(args[0], list):
        for c in args[0]:
          self.__add_c(c)
      else:
        for c in args:
          self.__add_c(c)
  def __getitem__(self, idx):
    return StanceConfigBodiesCollisionConfFromPtr(&(<c_mc_rbdyn.StanceConfigBodiesCollisionConf&>(self.v[idx])))
  def __setitem__(self, idx, StanceConfigBodiesCollisionConf value):
    c_mc_rbdyn.scbc_vector_set_item(deref(self.v), idx, deref(value.impl))

cdef StanceConfigBodiesCollisionConfVector StanceConfigBodiesCollisionConfVectorFromPtr(vector[c_mc_rbdyn.StanceConfigBodiesCollisionConf] * p):
  cdef StanceConfigBodiesCollisionConfVector ret = StanceConfigBodiesCollisionConfVector(skip_alloc = True)
  ret.__own_impl = False
  ret.v = p
  return ret

cdef class StanceConfigCollisions(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, autoc = [], robotEnv = [], robotEnvContactFilter = {}, skip_alloc = False):
    self.__own_impl = True
    if not skip_alloc:
      self.impl = new c_mc_rbdyn.StanceConfigCollisions()
      self.impl.autoc = deref(StanceConfigBodiesCollisionConfVector(autoc).v)
      self.impl.robotEnv = deref(StanceConfigBodiesCollisionConfVector(robotEnv).v)
      self.impl.robotEnvContactFilter = robotEnvContactFilter
  property autoc:
    def __get__(self):
      return StanceConfigBodiesCollisionConfVectorFromPtr(&self.impl.autoc)
    def __set__(self, value):
      self.impl.autoc = deref(StanceConfigBodiesCollisionConfVector(value).v)
  property robotEnv:
    def __get__(self):
      return StanceConfigBodiesCollisionConfVectorFromPtr(&self.impl.robotEnv)
    def __set__(self, value):
      self.impl.robotEnv = deref(StanceConfigBodiesCollisionConfVector(value).v)
  property robotEnvContactFilter:
    def __get__(self):
      return self.impl.robotEnvContactFilter
    def __set__(self, value):
      self.impl.robotEnvContactFilter = value

cdef StanceConfigCollisions StanceConfigCollisionsFromPtr(c_mc_rbdyn.StanceConfigCollisions * p):
    cdef StanceConfigCollisions ret = StanceConfigCollisions(skip_alloc = True)
    ret.__own_impl = False
    ret.impl = p
    return ret

cdef class StanceConfig(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self):
    self.__own_impl = True
    self.impl = new c_mc_rbdyn.StanceConfig()
  property comTask:
    def __get__(self):
      return StanceConfigCoMTaskFromPtr(&self.impl.comTask)
    def __set__(self, StanceConfigCoMTask v):
      self.impl.comTask = deref(v.impl)
  property comObj:
    def __get__(self):
      return StanceConfigCoMObjFromPtr(&self.impl.comObj)
    def __set__(self, StanceConfigCoMObj v):
      self.impl.comObj = deref(v.impl)
  property postureTask:
    def __get__(self):
      return StanceConfigPostureTaskFromPtr(&self.impl.postureTask)
    def __set__(self, StanceConfigPostureTask v):
      self.impl.postureTask = deref(v.impl)
  property contactTask:
    def __get__(self):
      return StanceConfigContactTaskFromPtr(&self.impl.contactTask)
    def __set__(self, StanceConfigContactTask v):
      self.impl.contactTask = deref(v.impl)
  property contactObj:
    def __get__(self):
      return StanceConfigContactObjFromPtr(&self.impl.contactObj)
    def __set__(self, StanceConfigContactObj v):
      self.impl.contactObj = deref(v.impl)
  property collisions:
    def __get__(self):
      return StanceConfigCollisionsFromPtr(&self.impl.collisions)
    def __set__(self, StanceConfigCollisions v):
      self.impl.collisions = deref(v.impl)

cdef class GeosGeomGeometry(object):
  def __cinit__(self):
    pass

cdef GeosGeomGeometry GeosGeomGeometryFromSharedPtr(shared_ptr[c_mc_rbdyn.Geometry] p):
  cdef GeosGeomGeometry ret = GeosGeomGeometry()
  ret.impl = p
  return ret

cdef class PolygonInterpolator(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __cinit__(self, *args, skip_alloc = False):
    if skip_alloc:
      assert(len(args) == 0)
      self.impl = NULL
      self.__own_impl = False
    else:
      self.__own_impl = True
      assert(len(args) == 1)
      data = json.load(args[0])
      tuple_pairs = []
      for p in data["tuple_pairs"]:
        p1 = map(float, p["p1"])
        p2 = map(float, p["p2"])
        tuple_pairs.append(((p1[0], p1[1]), (p2[0], p2[1])))
      self.impl = c_mc_rbdyn.polygonInterpolatorFromTuplePairs(tuple_pairs)
  def fast_interpolate(self, percent):
    return GeosGeomGeometryFromSharedPtr(self.impl.fast_interpolate(percent))

def points_from_polygon(GeosGeomGeometry geom):
  cdef vector[c_eigen.Vector3d] vp = c_mc_rbdyn.points_from_polygon(geom.impl)
  return [eigen.Vector3dFromC(v) for v in vp]

def loadStances(Robots robots, filename):
  cdef vector[c_mc_rbdyn.Stance] * stances = new vector[c_mc_rbdyn.Stance]()
  cdef vector[shared_ptr[c_mc_rbdyn.StanceAction]] actions = vector[shared_ptr[c_mc_rbdyn.StanceAction]]()
  cdef vector[c_mc_rbdyn.PolygonInterpolator] * interpolators = new vector[c_mc_rbdyn.PolygonInterpolator]()
  cdef StanceVector stances_ret = StanceVector()
  c_mc_rbdyn.loadStances(deref(robots.impl), filename, deref(stances), actions, deref(interpolators))
  actions_ret = []
  for i in range(actions.size()):
    actions_ret.append(StanceActionFromPtr(actions.at(i)))
  del interpolators
  stances_ret.v = stances
  return stances_ret,actions_ret

cdef shared_ptr[c_mc_rbdyn.StanceAction] fake_shared_from_sa(StanceAction sa):
  return c_mc_rbdyn.sa_fake_shared(sa.base)

cdef c_mc_rbdyn.StanceRawPtr get_stance_impl(Stance s):
  return s.impl

def saveStances(Robots robots, filename, stances_in, actions_in):
  cdef vector[c_mc_rbdyn.StanceRawPtr] stances = vector[c_mc_rbdyn.StanceRawPtr]()
  cdef vector[shared_ptr[c_mc_rbdyn.StanceAction]] actions = vector[shared_ptr[c_mc_rbdyn.StanceAction]]()
  for s in stances_in:
    stances.push_back(get_stance_impl(s))
  for sa in actions_in:
    actions.push_back(fake_shared_from_sa(sa))
  c_mc_rbdyn.pSaveStances(deref(robots.impl), filename, stances, actions)

def loadRobot(RobotModule module, *args):#sva.PTransformd base = None, bName = ""):
  cdef c_sva.PTransformd * b = NULL
  bName = ""
  if len(args):
    if isinstance(args[0], sva.PTransformd):
      b = (<sva.PTransformd>(args[0])).impl;
      if len(args) > 1:
        bName = args[1]
    else:
      deprecated()
      return loadRobot(module, *args[1:])
  return RobotsFromPtr(c_mc_rbdyn.loadRobot(deref(module.impl.get()), b, bName))

def loadRobots(robot_modules, robot_surface_dirs = None):
  if robot_surface_dirs is not None:
    deprecated()
  return RobotsFromPtr(c_mc_rbdyn.loadRobots(RobotModuleVector(robot_modules).v))

def loadRobotAndEnv(RobotModule module, RobotModule envModule,
                    sva.PTransformd base = None, bId = -1):
  if base is None:
    return RobotsFromPtr(c_mc_rbdyn.loadRobotAndEnv(deref(module.impl.get()), deref(envModule.impl.get())))
  else:
    return RobotsFromPtr(c_mc_rbdyn.loadRobotAndEnv(deref(module.impl.get()), deref(envModule.impl.get()), base.impl, bId))

def loadRobotFromUrdf(name, urdf, withVirtualLinks = True, filteredLinks = [],
    fixed = False, sva.PTransformd base = None, bName = ""):
  if base is None:
    base = sva.PTransformd(skip_alloc = True)
  robots = RobotsFromPtr(c_mc_rbdyn.loadRobotFromUrdf(name, urdf, withVirtualLinks,
    filteredLinks, fixed, base.impl, bName))
  return robots

def createRobotWithBase(Robot robot, Base base, eigen.Vector3d baseAxis = eigen.Vector3d.UnitZ()):
  cdef Robots ret = Robots()
  ret.impl.get().createRobotWithBase(deref(robot.impl), base.impl, baseAxis.impl)
  return ret

def planar(T, B, N):
  return sva.PTransformdFromC(c_mc_rbdyn.planar(T, B, N))

def cylindrical(T, T_rot):
  return sva.PTransformdFromC(c_mc_rbdyn.cylindrical(T, T_rot))

def planarParam(sva.PTransformd X_es_rs):
  T, B, N = 0., 0., 0.
  c_mc_rbdyn.planarParam(deref(X_es_rs.impl), T, B, N)
  return T, B, N

def cylindricalParam(sva.PTransformd X_es_rs):
  T, T_rot = 0., 0.
  c_mc_rbdyn.cylindricalParam(deref(X_es_rs.impl), T, T_rot)
  return T, T_rot

def jointParam(Surface r1s, Surface r2s, sva.PTransformd X_es_rs):
  return c_mc_rbdyn.jointParam(deref(r1s.impl), deref(r2s.impl), deref(X_es_rs.impl))

def robotCopy(robots, robot_idx = None):
  cdef Robots ret = Robots()
  if isinstance(robots, Robots):
    assert(robot_idx is not None)
    ret.impl.get().robotCopy(deref((<Robots>robots).impl).robot(robot_idx))
  elif isinstance(robots, Robot):
    assert(robot_idx is None)
    ret.impl.get().robotCopy(deref((<Robot>robots).impl))
  else:
    raise TypeError("Wrong arguments passed to robotCopy")
  return ret
