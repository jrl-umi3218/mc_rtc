# distutils: language = c++

#
# Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport mc_rbdyn.c_mc_rbdyn as c_mc_rbdyn

cimport eigen.c_eigen as c_eigen
cimport eigen.eigen as eigen

cimport sva.c_sva as c_sva
cimport sva.sva as sva

cimport rbdyn.c_rbdyn as c_rbdyn
cimport rbdyn.rbdyn as rbdyn
cimport rbdyn.parsers.c_parsers as c_rbdyn_parsers
cimport rbdyn.parsers.parsers as rbdyn_parsers

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
      body1 = args[0]
      if isinstance(body1, unicode):
        body1 = body1.encode(u'ascii')
      body2 = args[1]
      if isinstance(body2, unicode):
        body2 = body2.encode(u'ascii')
      self.impl = c_mc_rbdyn.Collision(body1, body2, args[2], args[3],
          args[4])
    else:
      raise TypeError("Invalid arguments passed to Collision ctor")

  def isNone(self):
    return self.impl.isNone()

  property body1:
    def __get__(self):
      return self.impl.body1
    def __set__(self, value):
      if isinstance(value, unicode):
        value = value.encode(u'ascii')
      self.impl.body1 = value
  property body2:
    def __get__(self):
      return self.impl.body2
    def __set__(self, value):
      if isinstance(value, unicode):
        value = value.encode(u'ascii')
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
      if isinstance(value, unicode):
        value = value.encode(u'ascii')
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
    deprecated()
    return eigen.Vector3dFromC(self.impl.linearAcceleration())
  def linearAcceleration(self):
    return eigen.Vector3dFromC(self.impl.linearAcceleration())

cdef BodySensor BodySensorFromRef(c_mc_rbdyn.BodySensor & bs):
    cdef BodySensor ret = BodySensor(skip_alloc = True)
    ret.impl = &(bs)
    return ret

cdef BodySensor BodySensorFromCRef(const c_mc_rbdyn.BodySensor & bs):
    cdef BodySensor ret = BodySensor(skip_alloc = True)
    ret.impl = &(c_mc_rbdyn.const_cast_body_sensor(bs))
    return ret

cdef class ForceSensor(object):
  def __dealloc__(self):
    if self.__own_impl:
      del self.impl
  def __ctor__(self, sn, bn, sva.PTransformd X_p_f):
    if isinstance(sn, unicode):
      sn = sn.encode(u'ascii')
    if isinstance(bn, unicode):
      bn = bn.encode(u'ascii')
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
  def wrenchWithoutGravity(self, Robot robot):
    return sva.ForceVecdFromC(self.impl.wrenchWithoutGravity(deref(robot.impl)))
  def worldWrench(self, Robot robot):
    return sva.ForceVecdFromC(self.impl.worldWrench(deref(robot.impl)))
  def worldWrenchWithoutGravity(self, Robot robot):
    return sva.ForceVecdFromC(self.impl.worldWrenchWithoutGravity(deref(robot.impl)))

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
      if isinstance(value, unicode):
        value = value.encode(u'ascii')
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
    bounds = list(deref(self.impl).bounds())
    return [{k.decode(): v for k,v in i.items()} for i in bounds]
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
  def bodySensors(self):
    assert(self.impl.get())
    size = c_mc_rbdyn.getBodySensorsSize(deref(self.impl))
    ret = []
    for i in range(size):
        ret.append(BodySensorFromCRef(c_mc_rbdyn.getBodySensor(deref(self.impl), i)))
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
    cdef vector[string] joints = deref(self.impl).ref_joint_order()
    return [ j.decode('ascii') for j in joints ]
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
    def __set__(self, rbdyn.MultiBody mb):
      assert(self.impl.get())
      deref(self.impl).mb = deref(mb.impl)
  property mbc:
    def __get__(self):
      assert(self.impl.get())
      return rbdyn.MultiBodyConfigFromC(deref(self.impl).mbc, copy = False)
    def __set__(self, rbdyn.MultiBodyConfig mbc):
      assert(self.impl.get())
      deref(self.impl).mbc = deref(mbc.impl)
  property mbg:
    def __get__(self):
      assert(self.impl.get())
      return rbdyn.MultiBodyGraphFromC(deref(self.impl).mbg, copy = False)
  property _visual:
    def __get__(self):
      res = {}
      for it in deref(self.impl)._visual:
        res[it.first] = []
        for v in it.second:
          res[it.first].append(rbdyn_parsers.VisualFromC(v))
      return res
  property _collision:
    def __get__(self):
      res = {}
      for it in deref(self.impl)._collision:
        res[it.first] = []
        for c in it.second:
          res[it.first].append(rbdyn_parsers.VisualFromC(c))
      return res

cdef RobotModule RobotModuleFromC(const c_mc_rbdyn.RobotModulePtr v):
  cdef RobotModule ret = RobotModule()
  ret.impl = v
  return ret

cdef RobotModule RobotModuleFromCRef(const c_mc_rbdyn.RobotModule & rm):
  cdef RobotModule ret = RobotModule()
  ret.impl = c_mc_rbdyn.copyRobotModule(rm)
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

class RobotLoader(object):
  @staticmethod
  def get_robot_module(name, *args):
    cdef shared_ptr[c_mc_rbdyn.RobotModule] rm
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    if len(args) == 0:
      rm = c_mc_rbdyn.get_robot_module(name)
    elif len(args) == 1:
      arg0 = args[0]
      if isinstance(arg0, unicode):
        arg0 = arg0.encode(u'ascii')
      rm = c_mc_rbdyn.get_robot_module(name, arg0)
    elif len(args) == 2:
      arg0 = args[0]
      if isinstance(arg0, unicode):
        arg0 = arg0.encode(u'ascii')
      arg1 = args[1]
      if isinstance(arg1, unicode):
        arg1 = arg1.encode(u'ascii')
      rm = c_mc_rbdyn.get_robot_module(name, arg0, arg1)
    else:
      raise TypeError("Wrong arguments passed to get_robot_module")
    return RobotModuleFromC(rm)
  @staticmethod
  def available_robots():
    cdef vector[string] bots
    bots = c_mc_rbdyn.available_robots()
    return [ b.decode('ascii') for b in bots ]
  @staticmethod
  def clear():
    c_mc_rbdyn.clear_robot_module_path()
  @staticmethod
  def update_robot_module_path(paths):
    paths = [ p.encode(u'ascii') for p in paths ]
    c_mc_rbdyn.update_robot_module_path(paths)

def get_robot_module(name, *args):
  if isinstance(name, unicode):
    name = name.encode(u'ascii')
  return RobotLoader.get_robot_module(name, *args)

cdef class Robots(object):
  def __copyctor__(self, Robots other):
    self.impl = c_mc_rbdyn.robots_copy(other.impl)
  def __cinit__(self, *args, skip_alloc = False):
    if len(args) == 0:
      if not skip_alloc:
        self.impl = c_mc_rbdyn.robots_make()
    elif len(args) == 1 and isinstance(args[0], Robots):
      self.__copyctor__(args[0])
    else:
      raise TypeError("Wrong arguments passed to Robots ctor")

  def robots(self):
    ret = []
    for i in range(deref(self.impl).size()):
      ret.append(self.Robot(i))
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
    if isinstance(bName, unicode):
      bName = bName.encode(u'ascii')
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

cdef Robots RobotsFromRef(c_mc_rbdyn.Robots & p):
    return RobotsFromPtr(c_mc_rbdyn.robots_shared_from_ref(p))

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
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.impl.hasJoint(name)
  def hasBody(self, name):
    self.__is_valid()
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.impl.hasBody(name)
  def jointIndexByName(self, name):
    self.__is_valid()
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.impl.jointIndexByName(name)
  def bodyIndexByName(self, name):
    self.__is_valid()
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.impl.bodyIndexByName(name)

  def forceSensor(self, name):
    self.__is_valid()
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return ForceSensorFromRef(self.impl.forceSensor(name))
  def hasForceSensor(self, name):
    self.__is_valid()
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.impl.hasForceSensor(name)
  def bodyForceSensor(self, name):
    self.__is_valid()
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return ForceSensorFromRef(self.impl.bodyForceSensor(name))
  def bodyHasForceSensor(self, name):
    self.__is_valid()
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.impl.bodyHasForceSensor(name)

  def bodySensor(self, name = None):
    self.__is_valid()
    if name is None:
      if isinstance(name, unicode):
        name = name.encode(u'ascii')
      return BodySensorFromRef(self.impl.bodySensor())
    else:
      return BodySensorFromRef(self.impl.bodySensor(name))
  def bodySensors(self):
    self.__is_valid()
    size = c_mc_rbdyn.getBodySensorsSize(deref(self.impl))
    ret = []
    for i in range(size):
        ret.append(BodySensorFromCRef(c_mc_rbdyn.getBodySensor(deref(self.impl), i)))
    return ret
  def hasBodySensor(self, name):
    self.__is_valid()
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.impl.hasBodySensor(name)
  def bodyHasBodySensor(self, name):
    self.__is_valid()
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.impl.bodyHasBodySensor(name)
  def bodyBodySensor(self, name):
    self.__is_valid()
    return BodySensorFromRef(self.impl.bodyBodySensor(name))

  def surfaceWrench(self, surfaceName):
      self.__is_valid()
      if isinstance(surfaceName, unicode):
        surfaceName = surfaceName.encode(u'ascii')
      return sva.ForceVecdFromC(self.impl.surfaceWrench(surfaceName))

  def cop(self, surfaceName, min_pressure):
      self.__is_valid()
      return eigen.Vector2dFromC(self.impl.cop(surfaceName, min_pressure))

  def copW(self, Robot robot, surfaceName, min_pressure):
      self.__is_valid()
      return eigen.Vector3dFromC(self.impl.copW(surfaceName,min_pressure))

  def zmp(self, vector[string] sensorsName, eigen.Vector3d plane_p, eigen.Vector3d plane_n, forceThreshold):
      self.__is_valid()
      return eigen.Vector3dFromC(self.impl.zmp(sensorsName, plane_p.impl, plane_n.impl, forceThreshold))


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

  property q:
    def __get__(self):
      self.__is_valid()
      return rbdyn.DoubleVectorVectorWrapperFromC(self.impl.q(), self)
  property alpha:
    def __get__(self):
      self.__is_valid()
      return rbdyn.DoubleVectorVectorWrapperFromC(self.impl.alpha(), self)
  property alphaD:
    def __get__(self):
      self.__is_valid()
      return rbdyn.DoubleVectorVectorWrapperFromC(self.impl.alphaD(), self)
  property jointTorque:
    def __get__(self):
      self.__is_valid()
      return rbdyn.DoubleVectorVectorWrapperFromC(self.impl.jointTorque(), self)
  property bodyPosW:
    def __get__(self):
      self.__is_valid()
      ret = []
      end = self.impl.bodyPosW().end()
      it = self.impl.bodyPosW().begin()
      while it != end:
        ret.append(sva.PTransformdFromC(deref(it)))
        preinc(it)
      return ret
  property bodyVelW:
    def __get__(self):
      self.__is_valid()
      ret = []
      end = self.impl.bodyVelW().end()
      it = self.impl.bodyVelW().begin()
      while it != end:
        ret.append(sva.MotionVecdFromC(deref(it)))
        preinc(it)
      return ret
  property bodyVelB:
    def __get__(self):
      self.__is_valid()
      ret = []
      end = self.impl.bodyVelB().end()
      it = self.impl.bodyVelB().begin()
      while it != end:
        ret.append(sva.MotionVecdFromC(deref(it)))
        preinc(it)
      return ret
  property bodyAccB:
    def __get__(self):
      self.__is_valid()
      ret = []
      end = self.impl.bodyAccB().end()
      it = self.impl.bodyAccB().begin()
      while it != end:
        ret.append(sva.MotionVecdFromC(deref(it)))
        preinc(it)
      return ret
  property com:
    def __get__(self):
      self.__is_valid()
      return eigen.Vector3dFromC(self.impl.com())
  property comVelocity:
    def __get__(self):
      self.__is_valid()
      return eigen.Vector3dFromC(self.impl.comVelocity())
  property comAcceleration:
    def __get__(self):
      self.__is_valid()
      return eigen.Vector3dFromC(self.impl.comAcceleration())

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
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return self.impl.hasSurface(name)
  def surface(self, name):
    self.__is_valid()
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
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
    if isinstance(sName, unicode):
      sName = sName.encode(u'ascii')
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return SurfaceFromC(self.impl.copySurface(sName, name))
  def surfacePose(self, surface):
    if isinstance(surface, unicode):
      surface = surface.encode(u'ascii')
    return sva.PTransformdFromC(deref(self.impl).surfacePose(surface))

  def convex(self, name):
    self.__is_valid()
    if isinstance(name, unicode):
      name = name.encode(u'ascii')
    return (self.impl.convex(name).first,sch.S_ObjectFromPtr(self.impl.convex(name).second.get()))

  def convexes(self):
    self.__is_valid()
    end = deref(self.impl).convexes().const_end()
    it = deref(self.impl).convexes().const_begin()
    ret = {}
    while it  != end:
      ret[deref(it).first] = self.convex(deref(it).first)
      preinc(it)
    return ret

  def bodyTransform(self, bName):
    self.__is_valid()
    if isinstance(bName, unicode):
      bName = bName.encode(u'ascii')
    return sva.PTransformdFromC(self.impl.bodyTransform(bName), False)

  def collisionTransform(self, bName):
    self.__is_valid()
    if isinstance(bName, unicode):
      bName = bName.encode(u'ascii')
    return sva.PTransformdFromC(self.impl.collisionTransform(bName), False)

  def loadRSDFFromDir(self, surfaceDir):
    self.__is_valid()
    if isinstance(surfaceDir, unicode):
      surfaceDir = surfaceDir.encode(u'ascii')
    self.impl.loadRSDFFromDir(surfaceDir)

  def stance(self):
    self.__is_valid()
    return self.impl.stance()

  def forwardKinematics(self, rbdyn.MultiBodyConfig mbc=None):
    self.__is_valid()
    if mbc is None:
      self.impl.forwardKinematics()
    else:
      self.impl.forwardKinematics(deref(mbc.impl))

  def forwardVelocity(self, rbdyn.MultiBodyConfig mbc=None):
    self.__is_valid()
    if mbc is None:
      self.impl.forwardVelocity()
    else:
      self.impl.forwardVelocity(deref(mbc.impl))

  def forwardAcceleration(self, rbdyn.MultiBodyConfig mbc=None, sva.MotionVecd A_0 = None):
    self.__is_valid()
    if mbc is None:
      if A_0 is None:
        self.impl.forwardAcceleration()
      else:
        self.impl.forwardAcceleration(deref(A_0.impl))
    else:
      if A_0 is None:
        self.impl.forwardAcceleration(deref(mbc.impl))
      else:
        self.impl.forwardAcceleration(deref(mbc.impl), deref(A_0.impl))

  def posW(self, sva.PTransformd pt = None):
    self.__is_valid()
    if pt is None:
      return sva.PTransformdFromC(self.impl.posW())
    else:
      self.impl.posW(deref(pt.impl))

  def module(self):
      self.__is_valid()
      return RobotModuleFromCRef(self.impl.module())


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
  if isinstance(dirname, unicode):
    dirname = dirname.encode(u'ascii')
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
      sva.PTransformd X_es_rs = None, friction = Contact.defaultFriction):
    if isinstance(robotSurface, unicode):
      robotSurface = robotSurface.encode(u'ascii')
    if isinstance(envSurface, unicode):
      envSurface = envSurface.encode(u'ascii')
    if X_es_rs is None:
      self.impl = new c_mc_rbdyn.Contact(deref(robots.impl), robotSurface,
          envSurface, friction)
    else:
      self.impl = new c_mc_rbdyn.Contact(deref(robots.impl), robotSurface,
          envSurface, deref(X_es_rs.impl), friction)
  def __full_ctor__(self, Robots robots, r1Index, r2Index, r1Surface, r2Surface,
      sva.PTransformd X_r2s_r1s = sva.PTransformd.Identity(), sva.PTransformd Xbs =
      None, friction = Contact.defaultFriction, ambId = -1):
    if isinstance(r1Surface, unicode):
      r1Surface = r1Surface.encode(u'ascii')
    if isinstance(r2Surface, unicode):
      r2Surface = r2Surface.encode(u'ascii')
    if Xbs is None:
      Xbs = robots.robot(r1Index).surface(r1Surface).X_b_s()
    self.impl = new c_mc_rbdyn.Contact(deref(robots.impl), r1Index, r2Index,
        r1Surface, r2Surface, deref(X_r2s_r1s.impl), deref(Xbs.impl), friction, ambId)
  def __cinit__(self, *args, **kwds):
    if "skip_alloc" in kwds:
      skip_alloc = bool(kwds["skip_alloc"])
    else:
      skip_alloc = False
    self.__own_impl = True
    if len(args) == 0 and skip_alloc:
      self.impl = NULL
    elif len(args) == 1 and isinstance(args[0], Contact):
      self.__copyctor__(args[0])
    elif len(args) > 2 and isinstance(args[0], Robots):
      if len(args) == 3:
        self.__robotspt_ctor__(args[0], args[1], args[2])
      elif len(args) == 4:
        if isinstance(args[3], sva.PTransformd):
          self.__robotspt_ctor__(args[0], args[1], args[2], args[3])
        else:
          self.__robotspt_ctor__(args[0], args[1], args[2], None, args[3])
      elif len(args) == 5 and isinstance(args[3], sva.PTransformd):
        self.__robtspt_ctor(args[0], args[1], args[2], args[3], args[4])
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
        friction = Contact.defaultFriction
        if len(args) >= 8 or "friction" in kwds:
          if "friction" in kwds:
            friction = kwds["friction"]
          else:
            friction = args[7]
        ambiguityId = -1
        if len(args) >= 9 or "ambiguityId" in kwds:
          if "ambiguityId" in kwds:
            ambiguityId = kwds["ambiguityId"]
          else:
            ambiguityId = args[8]
        self.__full_ctor__(args[0], args[1], args[2], args[3], args[4],
            X_r2s_r1s, X_b_s, friction, ambiguityId)
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
      if idx >= self.v.size():
        raise IndexError
      return ContactFromC(self.v.at(idx), copy = False)
  def __setitem__(self, idx, Contact v):
    if idx >= self.v.size():
      raise IndexError
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
      data_path = args[0]
      if isinstance(data_path, unicode):
        data_path = data_path.encode(u'ascii')
      data = json.load(data_path)
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
  if isinstance(bName, unicode):
    bName = bName.encode(u'ascii')
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
  if isinstance(name, unicode):
    name = name.encode(u'ascii')
  if isinstance(urdf, unicode):
    urdf = urdf.encode(u'ascii')
  if base is None:
    base = sva.PTransformd(skip_alloc = True)
  robots = RobotsFromPtr(c_mc_rbdyn.loadRobotFromUrdf(name, urdf, withVirtualLinks,
    filteredLinks, fixed, base.impl, bName))
  return robots

def createRobotWithBase(name, Robot robot, Base base, eigen.Vector3d baseAxis = eigen.Vector3d.UnitZ()):
  cdef Robots ret = Robots()
  ret.impl.get().createRobotWithBase(name, deref(robot.impl), base.impl, baseAxis.impl)
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
    robot = robots.robot(robot_idx)
    ret.impl.get().robotCopy(deref((<Robot>robot).impl), robot.name())
  elif isinstance(robots, Robot):
    assert(robot_idx is None)
    ret.impl.get().robotCopy(deref((<Robot>robots).impl), robots.name())
  else:
    raise TypeError("Wrong arguments passed to robotCopy")
  return ret

def rpyToMat(*args):
  if len(args) == 1:
    assert len(args[0]) == 3, "Sequence argument must be of length 3"
    return eigen.Matrix3dFromC(c_mc_rbdyn.rpyToMat(args[0][0], args[0][1], args[0][2]))
  elif len(args) == 3:
    return eigen.Matrix3dFromC(c_mc_rbdyn.rpyToMat(args[0], args[1], args[2]))
  else:
    raise TypeError("rpyToMat expect one argument of length 3 or 3 arguments")

def rpyToPT(*args):
  if len(args) == 1:
    assert len(args[0]) == 3, "Sequence argument must be of length 3"
    return sva.PTransformdFromC(c_mc_rbdyn.rpyToPT(args[0][0], args[0][1], args[0][2]))
  elif len(args) == 3:
    return sva.PTransformdFromC(c_mc_rbdyn.rpyToPT(args[0], args[1], args[2]))
  else:
    raise TypeError("rpyToPT expect one argument of length 3 or 3 arguments")

def rpyFromMat(eigen.Matrix3d mat):
  return eigen.Vector3dFromC(c_mc_rbdyn.rpyFromMat(mat.impl))

def rpyFromQuat(eigen.Quaterniond quat):
  return eigen.Vector3dFromC(c_mc_rbdyn.rpyFromQuat(quat.impl))
