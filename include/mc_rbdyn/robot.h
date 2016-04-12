#ifndef _H_MCRBDYNROBOT_H_
#define _H_MCRBDYNROBOT_H_

#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>

#include <sch/S_Polyhedron/S_Polyhedron.h>
#include <sch/STP-BV/STP_BV.h>

#include <mc_rbdyn/Surface.h>

#include <memory>

namespace mc_control
{
  struct MCController;
}

namespace mc_rbdyn
{

struct RobotModule;

struct MC_RBDYN_DLLAPI Flexibility
{
public:
  std::string jointName;
  double K;
  double C;
  double O;
};

struct MC_RBDYN_DLLAPI ForceSensor
{
public:
  ForceSensor();
  ForceSensor(const std::string &, const std::string &, const sva::PTransformd &);

  std::string sensorName;
  std::string parentBodyName;
  sva::PTransformd X_p_f;
};

struct MC_RBDYN_DLLAPI Springs
{
public:
  Springs() : springsBodies(0), afterSpringsBodies(0), springsJoints(0) {}
public:
  std::vector<std::string> springsBodies;
  std::vector<std::string> afterSpringsBodies;
  std::vector< std::vector<std::string> > springsJoints;
};

struct MC_RBDYN_DLLAPI Base
{
  std::string baseName;
  sva::PTransformd X_0_s;
  sva::PTransformd X_b0_s;
  rbd::Joint::Type baseType;
};

struct MC_RBDYN_DLLAPI Robots
{
  friend struct Robot;
public:
  Robots();
  Robots(const Robots & rhs);
  Robots & operator=(const Robots & rhs);

  std::vector<Robot> & robots();
  const std::vector<Robot> & robots() const;

  std::vector<rbd::MultiBody> & mbs();
  const std::vector<rbd::MultiBody> & mbs() const;

  std::vector<rbd::MultiBodyConfig> & mbcs();
  const std::vector<rbd::MultiBodyConfig> & mbcs() const;

  unsigned int robotIndex() const;
  unsigned int envIndex() const;

  /* Robot(s) loader functions */
  Robot & load(const RobotModule & module, const std::string & surfaceDir, sva::PTransformd * base = nullptr, const std::string& bName = "");

  void load(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir);

  void load(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, sva::PTransformd * base, const std::string& baseName);

  void load(const std::vector<std::shared_ptr<RobotModule>> & modules, const std::vector<std::string> & surfaceDirs);

  Robot & loadFromUrdf(const std::string & name, const std::string & urdf, bool withVirtualLinks = true, const std::vector<std::string> & filteredLinks = {}, bool fixed = false, sva::PTransformd * base = nullptr, const std::string& baseName = "");

  void robotCopy(const Robots & robots, unsigned int robots_idx);

  void createRobotWithBase(Robots & robots, unsigned int robots_idx, const Base & base, const Eigen::Vector3d & baseAxis = Eigen::Vector3d::UnitZ());

  Robot & robot();
  const Robot & robot() const;

  Robot & env();
  const Robot & env() const;

  Robot & robot(unsigned int idx);
  const Robot & robot(unsigned int idx) const;
protected:
  std::vector<mc_rbdyn::Robot> robots_;
  std::vector<rbd::MultiBody> mbs_;
  std::vector<rbd::MultiBodyConfig> mbcs_;
  std::vector<rbd::MultiBodyGraph> mbgs_;
  unsigned int robotIndex_;
  unsigned int envIndex_;
  void updateIndexes();
};

/* Static pendant of the loader functions to create Robots directly */
MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobot(const RobotModule & module, const std::string & surfaceDir, sva::PTransformd * base = nullptr, const std::string& baseName = "");

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobots(const std::vector<std::shared_ptr<RobotModule>> & modules, const std::vector<std::string> & surfaceDirs);

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobotAndEnv(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir);

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobotAndEnv(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, sva::PTransformd * base, const std::string& baseName);

MC_RBDYN_DLLAPI std::shared_ptr<Robots> loadRobotFromUrdf(const std::string & name, const std::string & urdf, bool withVirtualLinks = true, const std::vector<std::string> & filteredLinks = {}, bool fixed = false, sva::PTransformd * base = nullptr, const std::string& baseName = "");

struct MC_RBDYN_DLLAPI Robot
{
  friend struct Robots;
  #if defined __GNUC__ && ! defined  __clang__
  friend class __gnu_cxx::new_allocator<Robot>;
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

  const std::string & accelerometerBody() const;

  bool hasJoint(const std::string & name) const;

  bool hasBody(const std::string & name) const;

  unsigned int jointIndexByName(const std::string & name) const;

  unsigned int bodyIndexByName(const std::string & name) const;

  std::string forceSensorParentBodyName(const std::string & fs) const;

  const ForceSensor & forceSensorData(const std::string & fs) const;

  bool hasForceSensor(const std::string & body) const;

  std::string forceSensorByBody(const std::string & body) const;

  std::vector<std::string> forceSensorsByName() const;

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

  const std::vector<Flexibility> & flexibility() const;

  bool hasSurface(const std::string & surface) const;

  mc_rbdyn::Surface & surface(const std::string & sName);
  const mc_rbdyn::Surface & surface(const std::string & sName) const;

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
  Robots & robots;
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
  std::vector<ForceSensor> forceSensors;
  std::map<std::string, std::vector<double>> stance_;
  std::string _accelerometerBody;
  Springs springs;
  std::vector< std::vector<Eigen::VectorXd> > tlPoly;
  std::vector< std::vector<Eigen::VectorXd> > tuPoly;
  std::vector<Flexibility> flexibility_;
  std::map<std::string, unsigned int> jointIndexByNameD;
  std::map<std::string, unsigned int> bodyIndexByNameD;
  std::map<std::string, ForceSensor> forceSensorsParentD;
  std::map<std::string, std::string> parentBodyForceSensorD;
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
        const std::map<std::string, std::vector<double>> stance = {},
        const std::string & accelerometerBody = "",
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

template<typename sch_T>
void applyTransformToSchById(const rbd::MultiBody & mb, const rbd::MultiBodyConfig & mbc, std::map<std::string, std::pair<int, sch_T> > & schById);

/*FIXME Not implemetend for now, only used for ATLAS
void loadPolyTorqueBoundsData(const std::string & file, Robot & robot);
*/

}

#include <mc_rbdyn/robot.hpp>

#endif
