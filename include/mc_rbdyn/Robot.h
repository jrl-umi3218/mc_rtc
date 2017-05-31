#pragma once

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>

#include <sch/S_Polyhedron/S_Polyhedron.h>
#include <sch/STP-BV/STP_BV.h>

#include <mc_rbdyn/Base.h>
#include <mc_rbdyn/BodySensor.h>
#include <mc_rbdyn/Flexibility.h>
#include <mc_rbdyn/ForceSensor.h>
#include <mc_rbdyn/Springs.h>
#include <mc_rbdyn/Surface.h>

#include <memory>

namespace mc_control
{
  struct MCController;
}

namespace mc_rbdyn
{

struct Robots;

struct MC_RBDYN_DLLAPI Robot
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  friend struct Robots;
  #if defined __GLIBC__
  friend struct __gnu_cxx::new_allocator<Robot>;
  #else
  friend class std::allocator<Robot>;
  #endif
public:
  typedef std::pair<std::string, std::shared_ptr<sch::S_Polyhedron>> convex_pair_t;
  typedef std::pair<std::string, std::shared_ptr<sch::STP_BV>> stpbv_pair_t;
public:
  Robot(Robot&&) = default;
  Robot& operator=(Robot&&) = default;

  std::string name() const;

  void name(const std::string & n);

  /** @name Body sensors
   *
   * These functions are related to force sensors
   *
   * @{
   */

  /** Return the first BodySensor in the robot
   *
   * If the robot does not have body sensors, it returns a defautl
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

  bool hasJoint(const std::string & name) const;

  bool hasBody(const std::string & name) const;

  unsigned int jointIndexByName(const std::string & name) const;

  unsigned int bodyIndexByName(const std::string & name) const;

  rbd::MultiBody & mb();
  const rbd::MultiBody & mb() const;

  rbd::MultiBodyConfig & mbc();
  const rbd::MultiBodyConfig & mbc() const;

  rbd::MultiBodyGraph & mbg();
  const rbd::MultiBodyGraph & mbg() const;

  const std::vector<std::vector<double>> & ql() const;
  const std::vector<std::vector<double>> & qu() const;
  const std::vector<std::vector<double>> & vl() const;
  const std::vector<std::vector<double>> & vu() const;
  const std::vector<std::vector<double>> & tl() const;
  const std::vector<std::vector<double>> & tu() const;
  std::vector<std::vector<double>> & ql();
  std::vector<std::vector<double>> & qu();
  std::vector<std::vector<double>> & vl();
  std::vector<std::vector<double>> & vu();
  std::vector<std::vector<double>> & tl();
  std::vector<std::vector<double>> & tu();

  const std::vector<Flexibility> & flexibility() const;

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

  /** Check if the body has a force sensor attached to it
   *
   * @param body Name of the body
   *
   * @returns True if the body has a force sensor attached to it, false
   * otherwise
   */
  bool bodyHasForceSensor(const std::string & body) const;

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
   * @throws If no sensor is attached to this body
   */
  ForceSensor & bodyForceSensor(const std::string & body);

  /** Const variant */
  const ForceSensor & bodyForceSensor(const std::string & body) const;

  /** Returns all force sensors */
  std::vector<ForceSensor> & forceSensors();

  /** Returns all force sensors (const) */
  const std::vector<ForceSensor> & forceSensors() const;

  /** @} */
  /* End of Force sensors group */

  bool hasSurface(const std::string & surface) const;

  mc_rbdyn::Surface & surface(const std::string & sName);
  const mc_rbdyn::Surface & surface(const std::string & sName) const;

  /** Copy an existing surface with a new name */
  mc_rbdyn::Surface & copySurface(const std::string & sName, const std::string & name);

  const std::map<std::string, mc_rbdyn::SurfacePtr> & surfaces() const;

  convex_pair_t & convex(const std::string & cName);
  const convex_pair_t & convex(const std::string & cName) const;

  const sva::PTransformd & bodyTransform(const std::string& bName) const;

  const sva::PTransformd & collisionTransform(const std::string& cName) const;

  void fixSurfaces();

  void loadRSDFFromDir(const std::string & surfaceDir);

  /** Return the robot's default stance (e.g. half-sitting for humanoid) */
  std::map<std::string, std::vector<double>> stance() const;
private:
  std::string name_;
  Robots * robots;
  unsigned int robots_idx;
  std::map<std::string, sva::PTransformd> bodyTransforms;
  std::vector< std::vector<double> > ql_;
  std::vector< std::vector<double> > qu_;
  std::vector< std::vector<double> > vl_;
  std::vector< std::vector<double> > vu_;
  std::vector< std::vector<double> > tl_;
  std::vector< std::vector<double> > tu_;
  std::map<std::string, convex_pair_t> convexes;
  std::map<std::string, stpbv_pair_t> stpbvs;
  std::map<std::string, sva::PTransformd> collisionTransforms;
  std::map<std::string, mc_rbdyn::SurfacePtr> surfaces_;
  std::vector<ForceSensor> forceSensors_;
  std::map<std::string, std::vector<double>> stance_;
  /** Encoder values provided by the low-level controller */
  std::vector<double> encoderValues_;
  /** Joint torques provided by the low-level controller */
  std::vector<double> jointTorques_;
  /** Reference joint order see mc_rbdyn::RobotModule */
  std::vector<std::string> refJointOrder_;
  /** Hold all body sensors */
  BodySensorVector bodySensors_;
  /** Correspondance between body sensor's name and body sensor index*/
  std::map<std::string, size_t> bodySensorsIndex_;
  /** Correspondance between bodies' names and attached body sensors */
  std::map<std::string, size_t> bodyBodySensors_;
  Springs springs;
  std::vector< std::vector<Eigen::VectorXd> > tlPoly;
  std::vector< std::vector<Eigen::VectorXd> > tuPoly;
  std::vector<Flexibility> flexibility_;
  std::map<std::string, unsigned int> jointIndexByNameD;
  std::map<std::string, unsigned int> bodyIndexByNameD;
  /** Correspondance between force sensor's name and force sensor index */
  std::map<std::string, size_t> forceSensorsIndex_;
  /** Correspondance between bodies' names and attached force sensors */
  std::map<std::string, size_t> bodyForceSensors_;
protected:
  Robot(const std::string & name, Robots & robots, unsigned int robots_idx,
        const std::map<std::string, sva::PTransformd> & bodyTransforms,
        const std::vector< std::vector<double> > & ql, const std::vector< std::vector<double> > & qu,
        const std::vector< std::vector<double> > & vl, const std::vector< std::vector<double> > & vu,
        const std::vector< std::vector<double> > & tl, const std::vector< std::vector<double> > & tu,
        const std::map<std::string, convex_pair_t> & convex,
        const std::map<std::string, stpbv_pair_t> & stpbv,
        const std::map<std::string, sva::PTransformd> & collisionTransforms,
        const std::map<std::string, mc_rbdyn::SurfacePtr> & surfaces,
        const std::vector<ForceSensor> & forceSensors,
        const std::vector<std::string> & refJointOrder,
        const std::map<std::string, std::vector<double>> stance = {},
        const BodySensorVector & bodySensors = BodySensorVector(),
        const Springs & springs = Springs(), const std::vector< std::vector<Eigen::VectorXd> > & tlPoly = {},
        const std::vector< std::vector<Eigen::VectorXd> > & tuPoly = {}, const std::vector<Flexibility> & flexibility = {});
  Robot(const Robot&) = delete;
  Robot& operator=(const Robot&) = delete;
  void createWithBase(Robots & robots, unsigned int robots_idx, const Base & base) const;
  void copy(Robots & robots, unsigned int robots_idx) const;
};

MC_RBDYN_DLLAPI std::vector< std::vector<double> > jointsParameters(const rbd::MultiBody & mb, const double & coeff);

MC_RBDYN_DLLAPI std::vector< std::vector<double> > jointsDof(const rbd::MultiBody & mb, const double & coeff);

MC_RBDYN_DLLAPI std::map<std::string, std::vector<double> > jointsVectorToName(const rbd::MultiBody & mb, const std::vector< std::vector<double> > & jointsVec,
                                                              const std::function<bool(const rbd::Joint &, const std::vector<double> &)> & filter
                                                                        = [](const rbd::Joint &, const std::vector<double> &){return true;});

MC_RBDYN_DLLAPI std::vector< std::vector<double> > jointsNameToVector(const rbd::MultiBody & mb, std::map<std::string, std::vector<double> > & jointsName, const std::vector<double> & def = {}, const std::function<bool (const rbd::Joint &)> & filter = [](const rbd::Joint &){return true;} );

// Return [ql, qu, vl, vu, tl, tu]
MC_RBDYN_DLLAPI std::vector< std::map< std::string, std::vector<double> > > defaultBounds(const rbd::MultiBody & mb);

/*FIXME Not implemetend for now, only used for ATLAS
void loadPolyTorqueBoundsData(const std::string & file, Robot & robot);
*/

}
