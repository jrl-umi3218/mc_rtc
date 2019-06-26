#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from eigen.c_eigen cimport *
from sva.c_sva cimport *
from rbdyn.c_rbdyn cimport *
cimport sch.c_sch as sch
cimport tasks.qp.c_qp

from libcpp.map cimport map as cppmap
from libcpp.pair cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef extern from "<memory>" namespace "std" nogil:
  cdef cppclass shared_ptr[T]:
    shared_ptr()
    shared_ptr(T*)
    T* get()
    T& operator*()

cdef extern from "<mc_rbdyn/Collision.h>" namespace "mc_rbdyn":
  cdef cppclass Collision:
    Collision()
    Collision(const string&, const string&, double, double, double)
    Collision(const Collision&)

    cppbool isNone()

    string body1
    string body2
    double iDist
    double sDist
    double damping

    cppbool operator==(const Collision&)
    cppbool operator!=(const Collision&)

cdef extern from "<mc_rbdyn/BodySensor.h>" namespace "mc_rbdyn":
  cdef cppclass BodySensor:
    BodySensor()

    string name()
    string parentBody()
    const PTransformd & X_b_s()
    const Vector3d position()
    const Quaterniond orientation()
    const Vector3d linearVelocity()
    const Vector3d angularVelocity()
    const Vector3d acceleration()

cdef extern from "<mc_rbdyn/Flexibility.h>" namespace "mc_rbdyn":
  cdef cppclass Flexibility:
    Flexibility()
    Flexibility(const Flexibility&)
    string jointName
    double K
    double C
    double O

cdef extern from "<mc_rbdyn/ForceSensor.h>" namespace "mc_rbdyn":
  cdef cppclass ForceSensor:
    ForceSensor()
    ForceSensor(const string&, const string&, const PTransformd&)
    ForceSensor(const ForceSensor&)

    string name()
    string parentBody()
    const PTransformd & X_p_f()
    const ForceVecd & wrench()
    double mass()
    ForceVecd wrenchWithoutGravity(const Robot &)
    ForceVecd worldWrench(const Robot &)
    ForceVecd worldWrenchWithoutGravity(const Robot &)

cdef extern from "<mc_rbdyn/Springs.h>" namespace "mc_rbdyn":
  cdef cppclass Springs:
    Springs()
    Springs(const Springs&)
    vector[string] springsBodies
    vector[string] afterSpringsBodies
    vector[vector[string]] springsJoints

cdef extern from "<mc_rbdyn/Base.h>" namespace "mc_rbdyn":
  cdef cppclass Base:
    Base()
    Base(const Base&)
    string baseName
    PTransformd X_0_s
    PTransformd X_b0_s
    JointType baseType

cdef extern from "<mc_rbdyn/RobotModule.h>" namespace "mc_rbdyn":
  cdef cppclass RobotModule:
    RobotModule(const string &, const string &)
    RobotModule(const string &, const string &, const string&)

    const vector[cppmap[string, vector[double]]] & bounds()
    const map[string, vector[double]] & stance()
    const map[string, pair[string, string]] & convexHull()
    const map[string, pair[string, string]] & stpbvHull()
    map[string, PTransformd] _collisionTransforms
    vector[Flexibility] _flexibility
    vector[ForceSensor] _forceSensors
    const Springs & springs()
    vector[Collision] _minimalSelfCollisions
    vector[Collision] _commonSelfCollisions
    const vector[string]& ref_joint_order()

    string path
    string name
    string urdf_path
    string rsdf_dir
    string calib_dir
    MultiBody mb
    MultiBodyConfig mbc
    MultiBodyGraph mbg

cdef extern from "mc_rbdyn_wrapper.hpp" namespace "mc_rbdyn":
  ctypedef shared_ptr[RobotModule] RobotModulePtr

cdef extern from "<mc_rbdyn/Robots.h>" namespace "mc_rbdyn":
  cdef cppclass Robots:
    Robots()
    Robots(const Robots &)

    const vector[Robot] & robots()

    const vector[MultiBody] & mbs()

    const vector[MultiBodyConfig] & mbcs()

    unsigned int robotIndex()
    unsigned int envIndex()

    const Robot & robot()
    const Robot & env()

    Robot & load(const RobotModule&, PTransformd*, const string&)

    const Robot & robot(unsigned int)

    void createRobotWithBase(Robots&, unsigned int, const Base&, const Vector3d&)

    void createRobotWithBase(Robot&, const Base&, const Vector3d&)

    void robotCopy(const Robots&, unsigned int)

    void robotCopy(const Robot&)

cdef extern from "<mc_rbdyn/Robot.h>" namespace "mc_rbdyn":
  cdef cppclass Robot:
    string name()
    cppbool hasJoint(string)
    cppbool hasBody(string)
    unsigned int jointIndexByName(string)
    unsigned int bodyIndexByName(string)

    bool hasForceSensor(string)
    ForceSensor& forceSensor(string)
    bool bodyHasForceSensor(string)
    ForceSensor & bodyForceSensor(string)

    BodySensor& bodySensor()
    bool hasBodySensor(string)
    BodySensor& bodySensor(string)
    bool bodyHasBodySensor(string)
    BodySensor& bodyBodySensor(string)

    const MultiBody& mb()
    const MultiBodyConfig& mbc()
    const MultiBodyGraph& mbg()

    vector[vector[double]]& q()
    vector[vector[double]]& alpha()
    vector[vector[double]]& alphaD()
    vector[vector[double]]& jointTorque()

    vector[PTransformd]& bodyPosW()
    vector[MotionVecd]& bodyVelW()
    vector[MotionVecd]& bodyVelB()
    vector[MotionVecd]& bodyAccB()

    vector[vector[double]]& ql()
    vector[vector[double]]& qu()
    vector[vector[double]]& vl()
    vector[vector[double]]& vu()
    vector[vector[double]]& tl()
    vector[vector[double]]& tu()

    Vector3d com()
    Vector3d comVelocity()
    Vector3d comAcceleration()

    ForceVecd surfaceWrench(string)
    Vector2d cop(string,double)
    Vector3d copW(string,double)
    Vector3d zmp(const vector[string]&,const Vector3d&,const Vector3d&,double)

    const vector[Flexibility]& flexibility()

    bool hasSurface(string)
    const Surface& surface(string)
    Surface& copySurface(string, string)
    const map[string, shared_ptr[Surface]]& surfaces()
    vector[string] availableSurfaces()

    const pair[string, shared_ptr[sch.S_Object]]& convex(string)

    const PTransformd& bodyTransform(const string&)

    const PTransformd& collisionTransform(const string&)

    void loadRSDFFromDir(string)

    map[string, vector[double]] stance()

    void forwardKinematics()
    void forwardKinematics(MultiBodyConfig& mbc)
    void forwardVelocity()
    void forwardVelocity(MultiBodyConfig& mbc)
    void forwardAcceleration()
    void forwardAcceleration(MotionVecd & A_0)
    void forwardAcceleration(MultiBodyConfig & mbc)
    void forwardAcceleration(MultiBodyConfig & mbc, MotionVecd & A_0)

    void eulerIntegration(double step)
    void eulerIntegration(MultiBodyConfig & mbc, double step)

    PTransformd posW()
    void posW(PTransformd pt)

  shared_ptr[Robots] loadRobot(const RobotModule&, PTransformd *, const string&)
  shared_ptr[Robots] loadRobots(const vector[RobotModulePtr]&)
  shared_ptr[Robots] loadRobotAndEnv(const RobotModule&, const RobotModule&)
  shared_ptr[Robots] loadRobotAndEnv(const RobotModule&, const RobotModule&, PTransformd*, const string&)
  shared_ptr[Robots] loadRobotFromUrdf(const string&, const string&, cppbool, const
      vector[string]&, cppbool, PTransformd*, const string&)

cdef extern from "<mc_rbdyn/Surface.h>" namespace "mc_rbdyn":
  cdef cppclass Surface:
    string name()
    string bodyName()
    string materialName()

    const vector[PTransformd]& points()

    unsigned int bodyIndex(const Robot&)

    PTransformd X_0_s(const Robot&)
    PTransformd X_0_s(const Robot&, const MultiBodyConfig& mbc)

    const PTransformd& X_b_s()
    void X_b_s(const PTransformd&)

    void computePoints()

    string toStr()

    shared_ptr[Surface] copy()

    string type()

    bool operator==(const Surface&)
    bool operator!=(const Surface&)

  ctypedef shared_ptr[Surface] SurfacePtr

cdef extern from "<mc_rbdyn/PlanarSurface.h>" namespace "mc_rbdyn":
  cdef cppclass PlanarSurface(Surface):
    void planarTransform(const double&, const double&, const double&)

    const vector[pair[double,double]]& planarPoints()

    void planarPoints(const vector[pair[double,double]]&)

  PlanarSurface * dynamic_cast_planar_surface"dynamic_cast<mc_rbdyn::PlanarSurface*>"(Surface*)

cdef extern from "<mc_rbdyn/GripperSurface.h>" namespace "mc_rbdyn":
  cdef cppclass GripperSurface(Surface):
    void originTransform(const PTransformd&)

    const vector[PTransformd]& pointsFromOrigin()

    const PTransformd& X_b_motor()

    const double& motorMaxTorque()

  GripperSurface * dynamic_cast_gripper_surface"dynamic_cast<mc_rbdyn::GripperSurface*>"(Surface*)

cdef extern from "<mc_rbdyn/CylindricalSurface.h>" namespace "mc_rbdyn":
  cdef cppclass CylindricalSurface(Surface):
    const double & radius()
    const double & width()
    void width(const double&)

  CylindricalSurface * dynamic_cast_cylindrical_surface"dynamic_cast<mc_rbdyn::CylindricalSurface*>"(Surface*)

cdef extern from "<mc_rbdyn/surface_utils.h>" namespace "mc_rbdyn":
  vector[shared_ptr[Surface]] readRSDFFromDir(const string&)

cdef extern from "<mc_rbdyn/Contact.h>" namespace "mc_rbdyn":
  cdef cppclass Contact:
    Contact(const Robots&, string, string)
    Contact(const Robots&, string, string, const PTransformd&)
    Contact(const Robots&, unsigned int, unsigned int, string, string, const
        PTransformd&, const PTransformd&, int)
    Contact(const Contact&)

    unsigned int r1Index()
    unsigned int r2Index()

    SurfacePtr r1Surface()
    SurfacePtr r2Surface()

    const PTransformd& X_r2s_r1s()
    void X_r2s_r1s(const PTransformd&)

    const PTransformd& X_b_s()

    int ambiguityId()

    bool isFixed()

    pair[string, string] surfaces()

    PTransformd X_0_r1s(const Robot&)
    PTransformd X_0_r1s(const Robots&)
    PTransformd X_0_r2s(const Robot&)
    PTransformd X_0_r2s(const Robots&)

    vector[PTransformd] r1Points()
    vector[PTransformd] r2Points()

    PTransformd compute_X_r2s_r1s(const Robots&)

    tasks.qp.c_qp.ContactId contactId(const Robots&)

    #c_mc_solver.QPContactPtr taskContact(const Robots&)
    #c_mc_solver.QPContactPtrWPoints taskContactWPoints(const Robots&, const PTransformd *)

    string toStr()

    cppbool operator==(const Contact&)
    cppbool operator!=(const Contact&)

  cdef double ContactnrConeGen "mc_rbdyn::Contact::nrConeGen"
  cdef double ContactdefaultFriction "mc_rbdyn::Contact::defaultFriction"
  cdef double ContactnrBilatPoints "mc_rbdyn::Contact::nrBilatPoints"

cdef extern from "<geos/geom/Geometry.h>" namespace "geos::geom":
  cdef cppclass Geometry:
      pass

cdef extern from "<mc_rbdyn/PolygonInterpolator.h>" namespace "mc_rbdyn":
  cdef cppclass PolygonInterpolator:
   # Actual constructor
   # PolygonInterpolator(const Json::Value&)
   shared_ptr[Geometry] fast_interpolate(double)

cdef extern from "<mc_rbdyn/polygon_utils.h>" namespace "mc_rbdyn":
  cdef vector[Vector3d] points_from_polygon(shared_ptr[Geometry])

cdef extern from "<mc_rbdyn/contact_transform.h>" namespace "mc_rbdyn":
  PTransformd planar(const double&, const double&, const double&)
  PTransformd cylindrical(const double&, const double&)
  void planarParam(const PTransformd&, double&, double&, double&)
  void cylindricalParam(const PTransformd&, double&, double&)
  vector[double] jointParam(const Surface&, const Surface&, const PTransformd&)

cdef extern from "mc_rbdyn_wrapper.hpp" namespace "mc_rbdyn":
  string CollisionToString(const Collision &)
  #FIXME Work-around the lack of variadic template support
  RobotModulePtr get_robot_module(const string&) except +
  RobotModulePtr get_robot_module(const string&, const cppbool&) except +
  RobotModulePtr get_robot_module_str "mc_rbdyn::get_robot_module"(const string&, const string&) except +
  RobotModulePtr get_robot_module(const string&, const string&, const string&) except +
  #XXX
  void update_robot_module_path(const vector[string] &)
  void clear_robot_module_path()
  vector[string] available_robots()
  Robots& const_cast_robots(const Robots&)
  Robot& const_cast_robot(const Robot&)
  ForceSensor& const_cast_force_sensor(const ForceSensor &)
  BodySensor& const_cast_body_sensor(const BodySensor &)
  Surface& const_cast_surface(const Surface&)
  Contact& const_cast_contact(const Contact&)
  vector[Contact]& const_cast_contact_vector(const vector[Contact]&)
  void contact_vector_set_item(vector[Contact]&, unsigned int, const Contact&)
  shared_ptr[Robots] robots_fake_shared(Robots*)
  PolygonInterpolator * polygonInterpolatorFromTuplePairs(const vector[pair[pair[double, double], pair[double, double]]]&)
  #FIXME Work-around lack of array support
  vector[double] robotModuleDefaultAttitude(RobotModulePtr rm)
  #XXX
  #FIXME Lack of support for vector[T,A] in Cython 0.20
  unsigned int getBodySensorsSize[T](T &)
  (BodySensor&) getBodySensor[T](T &, unsigned int)
  #XXX
