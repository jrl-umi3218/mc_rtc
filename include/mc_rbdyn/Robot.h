/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Base.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Surface.h>

#include <mc_control/generic_gripper.h>

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>

#include <sch/S_Object/S_Object.h>

#include <memory>
#include <unordered_map>

namespace mc_rbdyn
{

struct Robots;

struct MC_RBDYN_DLLAPI Robot
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend struct Robots;

public:
  using S_ObjectPtr = std::shared_ptr<sch::S_Object>;
  using convex_pair_t = std::pair<std::string, S_ObjectPtr>;

public:
  Robot(Robot &&) = default;
  Robot & operator=(Robot &&) = default;

  /** Returns the name of the robot
   *
   * \note To rename a robot, use
   * Robots::rename(const std::string &, const std::string &)
   **/
  const std::string & name() const;

  /** Retrieve the associated RobotModule */
  const RobotModule & module() const;

  /** @name Body sensors
   *
   * These functions are related to force sensors
   *
   * @{
   */

  /** Return the first BodySensor in the robot
   *
   * If the robot does not have body sensors, it returns a default
   * (invalid) one
   *
   */
  BodySensor & bodySensor();

  /** Return the first BodySensor in the robot (const) */
  const BodySensor & bodySensor() const;

  /** Return true if the robot has a body sensor named name
   *
   * @param name Name of the body sensor
   *
   */
  bool hasBodySensor(const std::string & name) const;

  /** Return true if the specified body has a body sensor attached to it
   *
   * @param body Body to query
   *
   */
  bool bodyHasBodySensor(const std::string & body) const;

  /** Return a specific BobySensor by name
   *
   * @param name Name of the sensor
   *
   * @throws If the sensor does not exist
   *
   */
  BodySensor & bodySensor(const std::string & name);

  /** Return a specific BodySensor by name (const) */
  const BodySensor & bodySensor(const std::string & name) const;

  /** Return a specific BodySensor by body name
   *
   * @param name Name of the body
   *
   * @throws If there is no sensor attached to the body
   *
   */
  BodySensor & bodyBodySensor(const std::string & name);

  /** Return a specific BodySensor by body name (const) */
  const BodySensor & bodyBodySensor(const std::string & name) const;

  /** Return all body sensors */
  BodySensorVector & bodySensors();

  /** Return all body sensors (const) */
  const BodySensorVector & bodySensors() const;

  /** @} */
  /* End of Body sensors group */

  /** Returns true if the robot has a joint named \p name */
  bool hasJoint(const std::string & name) const;

  /** Returns true if the robot has a body named \p name */
  bool hasBody(const std::string & name) const;

  /** Returns the joint index of joint named \name
   *
   * \throws If the joint does not exist within the robot.
   */
  unsigned int jointIndexByName(const std::string & name) const;

  /** Returns the joint index in the mbc of the joint with index jointIndex in
   * refJointOrder
   *
   * @note Joint indices can be -1 for joints present in refJointOrder but not
   * in the robot's mbc (such as filtered joints in some robot modules)
   *
   * @param jointIndex Joint index in refJointOrder
   *
   * @returns joint index in the mbc
   *
   * @throws If jointIndex >= refJointOrder.size()
   */
  int jointIndexInMBC(size_t jointIndex) const;

  /** Returns the body index of joint named \name
   *
   * \throws If the body does not exist within the robot.
   */
  unsigned int bodyIndexByName(const std::string & name) const;

  /** Access MultiBody representation of the robot */
  rbd::MultiBody & mb();
  /** Access MultiBody representation of the robot (const) */
  const rbd::MultiBody & mb() const;

  /** Access MultiBodyConfig of the robot's mb() */
  rbd::MultiBodyConfig & mbc();
  /** Access MultiBodyConfig of the robot's mb() (const) */
  const rbd::MultiBodyConfig & mbc() const;

  /** Access MultiBodyGraph that generated the robot's mb() */
  rbd::MultiBodyGraph & mbg();
  /** Access MultiBodyGraph that generated the robot's mb() (const) */
  const rbd::MultiBodyGraph & mbg() const;

  /** Equivalent to robot.mbc().q (const) */
  const std::vector<std::vector<double>> & q() const;
  /** Equivalent to robot.mbc().alpha (const) */
  const std::vector<std::vector<double>> & alpha() const;
  /** Equivalent to robot.mbc().alphaD (const) */
  const std::vector<std::vector<double>> & alphaD() const;
  /** Equivalent to robot.mbc().jointTorque (const) */
  const std::vector<std::vector<double>> & jointTorque() const;
  /** Equivalent to robot.mbc().bodyPosW (const) */
  const std::vector<sva::PTransformd> & bodyPosW() const;
  /** Equivalent to robot.mbc().bodyVelW (const) */
  const std::vector<sva::MotionVecd> & bodyVelW() const;
  /** Equivalent to robot.mbc().bodyVelB (const) */
  const std::vector<sva::MotionVecd> & bodyVelB() const;
  /** Equivalent to robot.mbc().bodyAccB (const) */
  const std::vector<sva::MotionVecd> & bodyAccB() const;
  /** Equivalent to robot.mbc().q */
  std::vector<std::vector<double>> & q();
  /** Equivalent to robot.mbc().alpha */
  std::vector<std::vector<double>> & alpha();
  /** Equivalent to robot.mbc().alphaD */
  std::vector<std::vector<double>> & alphaD();
  /** Equivalent to robot.mbc().jointTorque */
  std::vector<std::vector<double>> & jointTorque();
  /** Equivalent to robot.mbc().bodyPosW */
  std::vector<sva::PTransformd> & bodyPosW();
  /** Equivalent to robot.mbc().bodyVelW */
  std::vector<sva::MotionVecd> & bodyVelW();
  /** Equivalent to robot.mbc().bodyVelB */
  std::vector<sva::MotionVecd> & bodyVelB();
  /** Equivalent to robot.mbc().bodyAccB */
  std::vector<sva::MotionVecd> & bodyAccB();

  /** Access the position of body \p name in world coordinates
   *
   * \throws If the body does not exist within the robot
   */
  const sva::PTransformd & bodyPosW(const std::string & name) const;

  /** Relative transformation X_b1_b2 from body b1 to body b2
   *
   * \param b1 name of first body
   * \param b2 name of second body
   * \throws If b1 or b2 does not exist within the robot
   */
  sva::PTransformd X_b1_b2(const std::string & b1, const std::string & b2) const;

  /** Access the velocity of body \p name in world coordinates
   *
   * \throws If the body doest not exist within the robot
   */
  const sva::MotionVecd & bodyVelW(const std::string & name) const;

  /** Access the velocity of body \p name in body coordinates
   *
   * \throws If the body doest not exist within the robot
   */
  const sva::MotionVecd & bodyVelB(const std::string & name) const;

  /** Access the acceleration of body \p name in body coordinates
   *
   * \throws If the body doest not exist within the robot
   */
  const sva::MotionVecd & bodyAccB(const std::string & name) const;

  /** Compute and returns the current robot's CoM */
  Eigen::Vector3d com() const;
  /** Compute and returns the current robot's CoM velocity */
  Eigen::Vector3d comVelocity() const;
  /** Compute and returns the current robot's CoM acceleration */
  Eigen::Vector3d comAcceleration() const;

  /** Compute the gravity-free wrench in surface frame
   *
   * @note If the surface is indirectly attached to the sensor (i.e there are
   * joints in-between), then the kinematic transformation will be taken into
   * account but the effect of bodies in-between is not accounted for in the
   * returned wrench.
   *
   * @param surfaceName A surface attached to a force sensor
   *
   * @return Measured wrench in surface frame
   *
   * @throws If no sensor is attached to this surface
   */
  sva::ForceVecd surfaceWrench(const std::string & surfaceName) const;

  /** Compute the gravity-free wrench in body frame
   *
   * @note If the body is indirectly attached to the sensor (i.e there are
   * joints in-between), then the kinematic transformation will be taken into
   * account but the effect of bodies in-between is not accounted for in the
   * returned wrench.
   *
   * @param bodyName A body attached to a force sensor
   *
   * @return Measured wrench in body frame
   *
   * @throws If no sensor is attached to this surface
   */
  sva::ForceVecd bodyWrench(const std::string & bodyName) const;

  /** Compute the cop in surface frame computed from gravity-free force
   * measurements
   *
   * @param surfaceName A surface attached to a force sensor
   * @param min_pressure Minimum pressure in N (default 0.5N).
   *
   * @return Measured cop in surface frame
   *  - CoP if pressure >= min_pressure
   *  - Zero otherwise
   *
   * @throws If no sensor is attached to this surface
   */
  Eigen::Vector2d cop(const std::string & surfaceName, double min_pressure = 0.5) const;
  /** Compute the cop in inertial frame compute from gravity-free force
   * measurements
   *
   * @param surfaceName A surface attached to a force sensor
   * @param min_pressure Minimum pressure in N (default 0.5N).
   *
   * @return Measured cop in inertial frame
   *  - CoP if pressure >= min_pressure
   *  - Zero otherwise
   *
   * @throws If no sensor is attached to this surface
   */
  Eigen::Vector3d copW(const std::string & surfaceName, double min_pressure = 0.5) const;

  /**
   * @brief Computes net total wrench from a list of sensors
   *
   * @param sensorNames Names of all sensors used to compute the net wrench
   *
   * @return Net total wrench (without gravity) in the inertial frame
   */
  sva::ForceVecd netWrench(const std::vector<std::string> & sensorNames) const;

  /**
   * @brief Actual ZMP computation from net total wrench and the ZMP plane
   *
   * @param netTotalWrench Total wrench for all links in contact
   * @param plane_p Arbitrary point on the ZMP plane
   * @param plane_n Normal to the ZMP plane (normalized)
   * @param minimalNetNormalForce[N] Mininal force above which the ZMP computation
   * is considered valid. Must be >0 (prevents a divide by zero).
   *
   * @return zmp expressed in the requested plane
   *
   * @throws To prevent dividing by zero, throws if the projected force is below minimalNetNormalForce newton.
   * This is highly unlikely to happen and would likely indicate indicate that you are computing a ZMP from
   * invalid forces (such as with the robot in the air).
   *
   * \anchor zmpDoc
   *
   * \see Eigen::Vector3d mc_rbdyn::zmp(const sva::ForceVecd & netTotalWrench, const Eigen::Vector3d & plane_p, const
   * Eigen::Vector3d & plane_n, double minimalNetNormalForce)
   */
  Eigen::Vector3d zmp(const sva::ForceVecd & netTotalWrench,
                      const Eigen::Vector3d & plane_p,
                      const Eigen::Vector3d & plane_n,
                      double minimalNetNormalForce = 1.) const;

  /**
   * @brief ZMP computation from net total wrench and a frame
   *
   * See \ref zmpDoc
   *
   * \see Eigen::Vector3d mc_rbdyn::zmp(const sva::ForceVecd & netTotalWrench, const sva::PTransformd & zmpFrame, double
   * minimalNetNormalForce) const;
   *
   * @param netTotalWrench
   * @param zmpFrame Frame used for ZMP computation. The convention here is
   * that the contact frame should have its z-axis pointing in the normal
   * direction of the contact towards the robot.
   *
   * @return ZMP expressed in the plane defined by the zmpFrame frame.
   */
  Eigen::Vector3d zmp(const sva::ForceVecd & netTotalWrench,
                      const sva::PTransformd & zmpFrame,
                      double minimalNetNormalForce = 1.) const;

  /** Computes the ZMP from sensor names and a plane
   *
   * See \ref zmpDoc
   *
   * @param sensorNames Names of all sensors attached to a link in contact with the environment
   */
  Eigen::Vector3d zmp(const std::vector<std::string> & sensorNames,
                      const Eigen::Vector3d & plane_p,
                      const Eigen::Vector3d & plane_n,
                      double minimalNetNormalForce = 1.) const;

  /**
   * @brief Computes the ZMP from sensor names and a frame
   *
   * See \ref zmpDoc
   *
   * @param sensorNames Names of all sensors attached to a link in contact with the environment
   */
  Eigen::Vector3d zmp(const std::vector<std::string> & sensorNames,
                      const sva::PTransformd & zmpFrame,
                      double minimalNetNormalForce = 1.) const;

  /** Access the robot's angular lower limits (const) */
  const std::vector<std::vector<double>> & ql() const;
  /** Access the robot's angular upper limits (const) */
  const std::vector<std::vector<double>> & qu() const;
  /** Access the robot's angular lower velocity limits (const) */
  const std::vector<std::vector<double>> & vl() const;
  /** Access the robot's angular upper velocity limits (const) */
  const std::vector<std::vector<double>> & vu() const;
  /** Access the robot's angular lower acceleration limits (const) */
  const std::vector<std::vector<double>> & al() const;
  /** Access the robot's angular upper acceleration limits (const) */
  const std::vector<std::vector<double>> & au() const;
  /** Access the robot's angular lower torque limits (const) */
  const std::vector<std::vector<double>> & tl() const;
  /** Access the robot's angular upper torque limits (const) */
  const std::vector<std::vector<double>> & tu() const;
  /** Access the robot's angular lower torque-derivative limits (const) */
  const std::vector<std::vector<double>> & tdl() const;
  /** Access the robot's angular upper torque-derivative limits (const) */
  const std::vector<std::vector<double>> & tdu() const;
  /** Access the robot's angular lower limits */
  std::vector<std::vector<double>> & ql();
  /** Access the robot's angular upper limits */
  std::vector<std::vector<double>> & qu();
  /** Access the robot's angular lower velocity limits */
  std::vector<std::vector<double>> & vl();
  /** Access the robot's angular upper velocity limits */
  std::vector<std::vector<double>> & vu();
  /** Access the robot's angular lower acceleration limits */
  std::vector<std::vector<double>> & al();
  /** Access the robot's angular upper acceleration limits */
  std::vector<std::vector<double>> & au();
  /** Access the robot's angular lower torque limits */
  std::vector<std::vector<double>> & tl();
  /** Access the robot's angular upper torque limits */
  std::vector<std::vector<double>> & tu();
  /** Access the robot's angular lower torque-derivative limits */
  std::vector<std::vector<double>> & tdl();
  /** Access the robot's angular upper torque-derivative limits */
  std::vector<std::vector<double>> & tdu();

  /** Return the flexibilities of the robot (const) */
  const std::vector<Flexibility> & flexibility() const;
  /** Return the flexibilities of the robot */
  std::vector<Flexibility> & flexibility();

  /** Set the target zmp defined with respect to base-link. This target is intended to be used by an external stabilizer
   * such as Kawada's
   *
   * @param zmp Note that usually the ZMP is a 2-vector assuming a perfectly
   * flat ground. The convention here is that the ground is at (tz=0). Therefore
   * the target zmp should be defined as (ZMPx, ZMPy, -zbaselink)
   */
  void zmpTarget(const Eigen::Vector3d & zmp);

  /** Returns the target zmp
   *
   * @return Target ZMP. See zmpTarget(Eigen::Vector3d) for details.
   */
  const Eigen::Vector3d & zmpTarget() const;

  /** Compute and returns the mass of the robot */
  double mass() const;

  /** @name Joint sensors
   *
   * These functions give information about joints' status
   *
   * @{
   */

  /** Return the encoder values */
  const std::vector<double> & encoderValues() const;

  /** Set the encoder values */
  void encoderValues(const std::vector<double> & encoderValues);

  /** Return the encoder velocities */
  const std::vector<double> & encoderVelocities() const;

  /** Set the encoder velocities */
  void encoderVelocities(const std::vector<double> & encoderVelocities);

  /** Return the flexibilities values */
  const std::vector<double> & flexibilityValues() const;

  /** Set the flexibilities values */
  void flexibilityValues(const std::vector<double> & flexibilityValues);

  /** Return the joint torques from sensors */
  const std::vector<double> & jointTorques() const;

  /** Set joint torques from sensors */
  void jointTorques(const std::vector<double> & jointTorques);

  /** Return the reference joint order for this robot */
  const std::vector<std::string> & refJointOrder() const;

  /** @} */
  /* End Joints sensors section */

  /** @name Force sensors
   *
   * These functions are related to force sensors
   *
   * @{
   */

  /** Check if a force sensor exists
   *
   * @param name Name of the sensor
   *
   * @returns True if the sensor exists, false otherwise
   */
  bool hasForceSensor(const std::string & name) const;

  /** Check if the body has a force sensor directly attached to it
   *
   * @param body Name of the body to which the sensor is directly attached
   *
   * @see bodyHasIndirectForceSensor(const std::string &) if you wish to check whether a
   * sensor is indirectly attached to a body
   *
   * @returns True if the body has a force sensor attached to it, false
   * otherwise
   */
  bool bodyHasForceSensor(const std::string & body) const;

  /**
   * @brief Checks if the surface has a force sensor directly attached to it
   *
   * @param surface Name of the surface to which the sensor is directly attached
   *
   * @see surfaceHasIndirectForceSensor(const std::string &) if you wish to check whether a
   * sensor is indirectly attached to a body
   *
   * @return True if the surface has a force sensor attached to it, false
   * otherwise
   */
  bool surfaceHasForceSensor(const std::string & surface) const;

  /** Check if the body has a force sensor attached to it (directly or
   * indirectly)
   *
   * @param body Name of the body to which the sensor is (directly or
   * indirectly) attached
   *
   * @returns True if the body has a force sensor attached to it, false
   * otherwise
   */
  bool bodyHasIndirectForceSensor(const std::string & body) const;

  /** Check if the surface has a force sensor attached to it (directly or
   * indirectly)
   *
   * @param surface Name of the surface to which the sensor is directly attached
   *
   * @returns True if the surface has a force sensor attached to it, false
   * otherwise
   */
  bool surfaceHasIndirectForceSensor(const std::string & surface) const;

  /** Return a force sensor by name
   *
   * @param name Name of the sensor
   *
   * @return The sensor named name
   *
   * @throws If no sensor with this name exists
   *
   */
  ForceSensor & forceSensor(const std::string & name);

  /** Const variant */
  const ForceSensor & forceSensor(const std::string & name) const;

  /** Return a force sensor attached to the provided body
   *
   * @param body Name of the body to which the sensor is attached
   *
   * @return The attached sensor
   *
   * @throws If no sensor is directly attached to this body
   *
   * @see ForceSensor & indirectBodyForceSensor(const std::string & body);
   * To get a sensor directly or indirectly attached to the body.
   */
  ForceSensor & bodyForceSensor(const std::string & body);

  /** Const variant */
  const ForceSensor & bodyForceSensor(const std::string & body) const;

  /** Return a force sensor attached to the provided surface
   *
   * @param surface Name of the surface to which the sensor is attached
   *
   * @return The attached sensor
   *
   * @throws If no sensor is directly attached to this surface
   *
   * @see ForceSensor & indirectBodyForceSensor(const std::string & surface);
   * To get a sensor directly or indirectly attached to the surface.
   */
  ForceSensor & surfaceForceSensor(const std::string & surfaceName);
  /** Const variant */
  const ForceSensor & surfaceForceSensor(const std::string & surfaceName) const;

  /**
   * @brief Return a force sensor directly or indirectly attached to a body
   *
   * When the sensor is not directly attached to the body, look up the kinematic chain until the root until a sensor is
   * found.
   *
   * @return The sensor to which the body is indirectly attached
   *
   * @throws If no sensor is found between the body and the root
   */
  ForceSensor & indirectBodyForceSensor(const std::string & body);

  /** Const variant */
  const ForceSensor & indirectBodyForceSensor(const std::string & body) const;

  /**
   * @brief Return a force sensor directly or indirectly attached to a surface
   *
   * When the sensor is not directly attached to the surface, look up the kinematic chain until the root until a sensor
   * is found.
   *
   * @param surface Name of surface indirectly attached to the sensor
   *
   * @return The sensor to which the surface is indirectly attached
   *
   * @throws If no sensor is found between the surface and the root
   */
  ForceSensor & indirectSurfaceForceSensor(const std::string & surface);

  /** Const variant */
  const ForceSensor & indirectSurfaceForceSensor(const std::string & surface) const;

  /** Returns all force sensors */
  std::vector<ForceSensor> & forceSensors();

  /** Returns all force sensors (const) */
  const std::vector<ForceSensor> & forceSensors() const;

  /** @} */
  /* End of Force sensors group */

  /** @name Devices
   *
   * These functions are related to generic devices handling
   *
   * @{
   */

  /** Returns true if a generic device of type T and named name exists in the robot
   *
   * \param name Name of the device
   *
   * \tparam T Type of device requested
   *
   */
  template<typename T>
  bool hasDevice(const std::string & name) const;

  /** Alias for \see hasDevice */
  template<typename T>
  inline bool hasSensor(const std::string & name) const
  {
    return hasDevice<T>(name);
  }

  /** Get a generic device of type T named name
   *
   * The reference returned by this function is remains valid
   *
   * \param name Name of the device
   *
   * \tparam T type of the device requested
   *
   * \throws If the device does not exist or does not have the right type
   *
   */
  template<typename T>
  const T & device(const std::string & name) const;

  /** Non-const variant */
  template<typename T>
  T & device(const std::string & name)
  {
    return const_cast<T &>(const_cast<const Robot *>(this)->device<T>(name));
  }

  /** Alias for \see device */
  template<typename T>
  inline const T & sensor(const std::string & name) const
  {
    return device<T>(name);
  }

  /** Alias for \see device */
  template<typename T>
  inline T & sensor(const std::string & name)
  {
    return device<T>(name);
  }

  /** Add a generic device to the robot */
  void addDevice(DevicePtr device);

  /** Alias for \see addDevice */
  inline void addSensor(SensorPtr sensor)
  {
    addDevice(std::move(sensor));
  }

  /** @} */
  /* End of Devices group */

  /** Check if a surface \p surface exists
   *
   * \returns True if the surface exists, false otherwise
   */
  bool hasSurface(const std::string & surface) const;

  /** Access a surface by its name \p sName */
  mc_rbdyn::Surface & surface(const std::string & sName);
  /** Access a surface by its name \p sName (const) */
  const mc_rbdyn::Surface & surface(const std::string & sName) const;

  /** Get the pose of a surface frame with respect to the inertial frame.
   *
   * \param sName Name of surface frame.
   *
   */
  sva::PTransformd surfacePose(const std::string & sName) const;

  /** Copy an existing surface with a new name */
  mc_rbdyn::Surface & copySurface(const std::string & sName, const std::string & name);

  /** Adds a surface with a new name */
  void addSurface(mc_rbdyn::SurfacePtr surface, bool doNotReplace = true);

  /** Returns all available surfaces */
  const std::map<std::string, mc_rbdyn::SurfacePtr> & surfaces() const;

  /** Returns a list of available surfaces */
  std::vector<std::string> availableSurfaces() const;

  /** Check if a convex \p name exists
   *
   * \returns True if the convex exists, false otherwise
   */
  bool hasConvex(const std::string & name) const;

  /** Access a convex named \p cName
   *
   * \returns a pair giving the convex's parent body and the sch::Object
   * object
   */
  convex_pair_t & convex(const std::string & cName);
  /** Access a convex named \p cName (const) */
  const convex_pair_t & convex(const std::string & cName) const;

  /** Access all convexes available in this robot
   *
   * \returns a map where keys are the convex name and values are those returned by \ref convex
   */
  const std::map<std::string, convex_pair_t> & convexes() const;

  /** Add a convex online
   *
   * This has no effect if \p name is already a convex of the robot
   *
   * \param name Name of the convex
   *
   * \param body Name of the convex's parent body
   *
   * \param convex sch::Object object representing the convex
   *
   * \param X_b_c Transformation fro the convex's parent body to the convex
   *
   */
  void addConvex(const std::string & name,
                 const std::string & body,
                 S_ObjectPtr convex,
                 const sva::PTransformd & X_b_c = sva::PTransformd::Identity());

  /** Remove a given convex
   *
   * Using this function while the given convex is involved in a collision is
   * *not* safe and will very likely result in a crash.
   *
   * This has no effect if \p name is not a convex of the robot.
   *
   * \param name Name of the convex
   *
   */
  void removeConvex(const std::string & name);

  /** Access transformation from body \p bName to original base.
   *
   * This can be used to correct transformations that were stored with the
   * original base. Usually the robot's base is the original base so these
   * transforms are identity.
   */
  const sva::PTransformd & bodyTransform(const std::string & bName) const;

  /** Access body transform by index */
  const sva::PTransformd & bodyTransform(int bodyIndex) const;

  /** Access body transform vector */
  const std::vector<sva::PTransformd> & bodyTransforms() const;

  /** Access transformation between the collision mesh and the body */
  const sva::PTransformd & collisionTransform(const std::string & cName) const;

  /** Load surfaces from the directory \p surfaceDir */
  void loadRSDFFromDir(const std::string & surfaceDir);

  /** Return the robot's default stance (e.g. half-sitting for humanoid) */
  std::map<std::string, std::vector<double>> stance() const;

  /** Access the robot's index in robots() */
  unsigned int robotIndex() const;

  /** Apply forward kinematics to the robot */
  void forwardKinematics();
  /** Apply forward kinematics to \p mbc using the robot's mb() */
  void forwardKinematics(rbd::MultiBodyConfig & mbc) const;

  /** Apply forward velocity to the robot */
  void forwardVelocity();
  /** Apply forward velocity to \p mbc using the robot's mb() */
  void forwardVelocity(rbd::MultiBodyConfig & mbc) const;

  /** Apply forward acceleration to the robot */
  void forwardAcceleration(const sva::MotionVecd & A_0 = sva::MotionVecd(Eigen::Vector6d::Zero()));
  /** Apply forward acceleration to \p mbc using the robot's mb() */
  void forwardAcceleration(rbd::MultiBodyConfig & mbc,
                           const sva::MotionVecd & A_0 = sva::MotionVecd(Eigen::Vector6d::Zero())) const;

  /** Apply Euler integration to the robot using \p step timestep */
  void eulerIntegration(double step);
  /** Apply Euler integration to \p mbc using the robot's mb() and \p step timestep */
  void eulerIntegration(rbd::MultiBodyConfig & mbc, double step) const;

  /** Return the robot's global pose */
  const sva::PTransformd & posW() const;
  /** Set the robot's global pose.
   * This is mostly meant for initialization purposes.
   * In other scenarios there might be more things to do
   * to properly move a robot (e.g. update contacts, set speed to zero).
   *
   * @param pt The new global pose
   *
   * @throws If joint(0) is neither free flyer nor fixed
   *
   * @note This function takes care of calling rbd::forwardKinematics
   */
  void posW(const sva::PTransformd & pt);

  /** Update the robot's floating base velocity.
   *
   * \param vel New floating base velocity in the inertial frame.
   *
   * @note This function takes care of calling rbd::forwardVelocity
   */
  void velW(const sva::MotionVecd & vel);

  /** Return the robot's floating base velocity expressed in the inertial frame */
  const sva::MotionVecd & velW() const;

  /** Update the robot's floating base acceleration.
   *
   * \param vel New floating base acceleration in the inertial frame.
   *
   * @note This function takes care of calling rbd::forwardAcceleration
   */
  void accW(const sva::MotionVecd & acc);

  /** Return the robot's floating base acceleration expressed in the inertial
   * frame
   */
  const sva::MotionVecd accW() const;

  /** Access a gripper by name
   *
   * \param gripper Gripper name
   *
   * \throws If the gripper does not exist within this robot
   */
  mc_control::Gripper & gripper(const std::string & gripper);

  /** Checks whether a gripper is part of this robot */
  bool hasGripper(const std::string & gripper) const;

  inline const std::unordered_map<std::string, mc_control::GripperPtr> & grippersByName() const
  {
    return grippers_;
  }

  /** Access all grippers */
  inline const std::vector<mc_control::GripperRef> & grippers() const
  {
    return grippersRef_;
  }

private:
  Robots * robots_;
  unsigned int robots_idx_;
  std::string name_;
  Eigen::Vector3d zmp_;
  std::vector<sva::PTransformd> bodyTransforms_;
  std::vector<std::vector<double>> ql_;
  std::vector<std::vector<double>> qu_;
  std::vector<std::vector<double>> vl_;
  std::vector<std::vector<double>> vu_;
  std::vector<std::vector<double>> al_;
  std::vector<std::vector<double>> au_;
  std::vector<std::vector<double>> tl_;
  std::vector<std::vector<double>> tu_;
  std::vector<std::vector<double>> tdl_;
  std::vector<std::vector<double>> tdu_;
  std::map<std::string, convex_pair_t> convexes_;
  std::map<std::string, sva::PTransformd> collisionTransforms_;
  std::map<std::string, mc_rbdyn::SurfacePtr> surfaces_;
  std::vector<ForceSensor> forceSensors_;
  std::map<std::string, std::vector<double>> stance_;
  /** Reference joint order see mc_rbdyn::RobotModule */
  std::vector<std::string> refJointOrder_;
  /** Correspondance between refJointOrder (actuated joints) index and
   * mbc index. **/
  std::vector<int> refJointIndexToMBCIndex_;
  /** Encoder values provided by the low-level controller */
  std::vector<double> encoderValues_;
  /** Encoder velocities provided by the low-level controller or estimated from
   * encoder values **/
  std::vector<double> encoderVelocities_;
  /** Joint torques provided by the low-level controller */
  std::vector<double> jointTorques_;
  std::vector<double> flexibilityValues_;
  /** Hold all body sensors */
  BodySensorVector bodySensors_;
  /** Correspondance between body sensor's name and body sensor index*/
  std::unordered_map<std::string, size_t> bodySensorsIndex_;
  /** Correspondance between bodies' names and attached body sensors */
  std::unordered_map<std::string, size_t> bodyBodySensors_;
  Springs springs_;
  std::vector<std::vector<Eigen::VectorXd>> tlPoly_;
  std::vector<std::vector<Eigen::VectorXd>> tuPoly_;
  std::vector<Flexibility> flexibility_;
  /** Correspondance between force sensor's name and force sensor index */
  std::unordered_map<std::string, size_t> forceSensorsIndex_;
  /** Correspondance between bodies' names and attached force sensors */
  std::map<std::string, size_t> bodyForceSensors_;
  /** Grippers attached to this robot */
  std::unordered_map<std::string, mc_control::GripperPtr> grippers_;
  /** Grippers reference for this robot */
  std::vector<mc_control::GripperRef> grippersRef_;
  /** Hold all devices that are neither force sensors nor body sensors */
  DevicePtrVector devices_;
  /** Correspondance between a device's name and a device index */
  std::unordered_map<std::string, size_t> devicesIndex_;

protected:
  /** Invoked by Robots parent instance after mb/mbc/mbg/RobotModule are stored
   *
   * When loadFiles is set to false, the convex and surfaces files are not
   * loaded. This is used when copying one robot into another.
   *
   */
  Robot(const std::string & name,
        Robots & robots,
        unsigned int robots_idx,
        bool loadFiles,
        const sva::PTransformd * base = nullptr,
        const std::string & baseName = "");

  /** Copy existing Robot with a new base
   *
   * \throws std::runtime_error if a robot named <copyName> already exists
   **/
  void copy(Robots & robots, const std::string & copyName, unsigned int robots_idx, const Base & base) const;
  /** Copy existing Robot
   *
   * \throws std::runtime_error if a robot named <copyName> already exists
   **/
  void copy(Robots & robots, const std::string & copyName, unsigned int robots_idx) const;

  /** Used to set the surfaces' X_b_s correctly */
  void fixSurfaces();

  /** Used to set the collision transforms correctly */
  void fixCollisionTransforms();

  /**
   * @brief Finds the name of the body to which a force sensor is attached,
   * starting from the provided body and going up the kinematic tree.
   *
   * @param bodyName Name of the body to which the sensor is attached
   *
   * @return Body name to which the sensor is attached when found. Empty string otherwise
   */
  std::string findIndirectForceSensorBodyName(const std::string & bodyName) const;

private:
  Robot(const Robot &) = delete;
  Robot & operator=(const Robot &) = delete;

  /** Set the name of the robot
   *
   * \note It is not recommended to call this late in the life cycle of the
   * Robot object as the change is not communicated in any way.
   */
  void name(const std::string & n);
};

/** @defgroup robotFromConfig Helpers to obtain robot index/name from configuration
 *  The intent of these functions is:
 *  - to facilitate supporting managing robots by name in the FSM, while maintaining compatibility with the existing
 * FSMs using robotIndex.
 *  - to facilitate the upcoming transition from Tasks (requires robotIndex) to TVM (using only robotName).
 *    When the transition occurs, robotIndexFromConfig can be deprecated, and its uses be replaced with
 * robotNameFromConfig instead.
 *  @{
 */

/**
 * @brief Obtains a reference to a loaded robot from configuration (using robotName or robotIndex)
 *
 * - If robotName is present, it is used to find the robot
 * - Otherwise, it'll attempt to use robotIndex and inform the user that its use
 *   is deprecated in favor of robotName
 *
 * @param config Configuration from which to look for robotName/robotIndex
 * @param robots Loaded robots
 * @param prefix Prefix used for printint outputs to the user (deprecation
 * warning, non-existing robot, etc).
 * @param required
 * - When true, throws if the robotName/robotIndex is invalid or missing.
 * - When false, returns the main robot if the robotName/robotIndex is invalid or missing.
 *
 * @param robotIndexKey Configuration key for robotIndex
 * @param robotNameKey Configuration key for robotName
 * @param defaultRobotName When empty, return the main robot name, otherwise use
 * the specified name
 *
 * @return Robot as configured by the robotName or robotIndex configuration
 * entry.
 */
MC_RBDYN_DLLAPI const mc_rbdyn::Robot & robotFromConfig(const mc_rtc::Configuration & config,
                                                        const mc_rbdyn::Robots & robots,
                                                        const std::string & prefix,
                                                        bool required = false,
                                                        const std::string & robotIndexKey = "robotIndex",
                                                        const std::string & robotNameKey = "robot",
                                                        const std::string & defaultRobotName = "");

/**
 * @brief Helper to obtain the robot name from configuration
 *
 * @see const mc_rbdyn::Robot & robotFromConfig(const mc_rtc::Configuration & config, const mc_rbdyn::Robots & robots,
 * const std::string & prefix, bool required);
 */
std::string MC_RBDYN_DLLAPI robotNameFromConfig(const mc_rtc::Configuration & config,
                                                const mc_rbdyn::Robots & robots,
                                                const std::string & prefix = "",
                                                bool required = false,
                                                const std::string & robotIndexKey = "robotIndex",
                                                const std::string & robotNameKey = "robot",
                                                const std::string & defaultRobotName = "");

/**
 * @brief Helper to obtain the robot index from configuration
 *
 * @note This function will be removed when transitioning to TVM. To facilitate
 * the transition from index-based to named-based robots, this function can be
 * deprecated to help with the transition.
 *
 * @see const mc_rbdyn::Robot & robotFromConfig(const mc_rtc::Configuration & config, const mc_rbdyn::Robots & robots,
 * const std::string & prefix, bool required);
 */
unsigned int MC_RBDYN_DLLAPI robotIndexFromConfig(const mc_rtc::Configuration & config,
                                                  const mc_rbdyn::Robots & robots,
                                                  const std::string & prefix = "",
                                                  bool required = false,
                                                  const std::string & robotIndexKey = "robotIndex",
                                                  const std::string & robotNameKey = "robot",
                                                  const std::string & defaultRobotName = "");

/** @} */

/*FIXME Not implemetend for now, only used for ATLAS
void loadPolyTorqueBoundsData(const std::string & file, Robot & robot);
*/

} // namespace mc_rbdyn

#include <mc_rbdyn/Robot.hpp>
