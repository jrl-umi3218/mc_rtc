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

struct Flexibility
{
public:
  std::string jointName;
  double K;
  double C;
  double O;
};

struct ForceSensor
{
public:
  ForceSensor();
  ForceSensor(const std::string &, const std::string &, const sva::PTransformd &);

  std::string sensorName;
  std::string parentBodyName;
  sva::PTransformd X_p_f;
};

struct Springs
{
public:
  Springs() : springsBodies(0), afterSpringsBodies(0), springsJoints(0) {}
public:
  std::vector<std::string> springsBodies;
  std::vector<std::string> afterSpringsBodies;
  std::vector< std::vector<std::string> > springsJoints;
};

struct Base
{
  int baseId;
  sva::PTransformd X_0_s;
  sva::PTransformd X_b0_s;
  rbd::Joint::Type baseType;
};

struct Robots;

struct Robot
{
  friend struct Robots;
public:
  Robot(const std::shared_ptr<rbd::MultiBody> & mb, const std::shared_ptr<rbd::MultiBodyConfig> & mbc, const std::shared_ptr<rbd::MultiBodyGraph> & mbg,
        const std::map<int, sva::PTransformd> & bodyTransforms,
        const std::vector< std::vector<double> > & ql, const std::vector< std::vector<double> > & qu,
        const std::vector< std::vector<double> > & vl, const std::vector< std::vector<double> > & vu,
        const std::vector< std::vector<double> > & tl, const std::vector< std::vector<double> > & tu,
        const std::map<std::string, std::pair<unsigned int, std::shared_ptr<sch::S_Polyhedron> > > & convex,
        const std::map<std::string, std::pair<unsigned int, std::shared_ptr<sch::STP_BV> > > & stpbv,
        const std::map<int, sva::PTransformd> & collisionTransforms,
        const std::map<std::string, mc_rbdyn::SurfacePtr> & surfaces,
        const std::vector<ForceSensor> & forceSensors, const std::string & accelerometerBody = "",
        const Springs & springs = Springs(), const std::vector< std::vector<Eigen::VectorXd> > & tlPoly = {},
        const std::vector< std::vector<Eigen::VectorXd> > & tuPoly = {}, const std::vector<Flexibility> & flexibility = {});

  bool hasJoint(const std::string & name) const;

  bool hasBody(const std::string & name) const;

  unsigned int jointIndexByName(const std::string & name) const;

  unsigned int bodyIndexByName(const std::string & name) const;

  unsigned int jointIdByName(const std::string & name) const;

  unsigned int bodyIdByName(const std::string & name) const;

  std::string forceSensorParentBodyName(const std::string & fs) const;

  const ForceSensor & forceSensorData(const std::string & fs) const;

  bool hasForceSensor(const std::string & body) const;

  std::string forceSensorByBody(const std::string & body) const;

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

  mc_rbdyn::Surface & surface(const std::string & sName);
  const mc_rbdyn::Surface & surface(const std::string & sName) const;

  const std::map<std::string, mc_rbdyn::SurfacePtr> & surfaces() const;

  typedef std::pair<unsigned int, std::shared_ptr<sch::S_Polyhedron>> convex_pair_t;
  convex_pair_t & convex(const std::string & cName);
  const convex_pair_t & convex(const std::string & cName) const;

  const sva::PTransformd & bodyTransform(int id) const;

  const sva::PTransformd & collisionTransform(int id) const;

  void fixSurfaces();

  Robot createWithBase(const Base & base, const Eigen::Vector3d & baseAxis = Eigen::Vector3d::UnitZ()) const;
  Robot copy() const;
protected:
  std::shared_ptr<rbd::MultiBody> mb_;
  std::shared_ptr<rbd::MultiBodyConfig> mbc_;
private:
  std::shared_ptr<rbd::MultiBodyGraph> mbg_;
  std::map<int, sva::PTransformd> bodyTransforms;
  std::vector< std::vector<double> > ql_;
  std::vector< std::vector<double> > qu_;
  std::vector< std::vector<double> > vl_;
  std::vector< std::vector<double> > vu_;
  std::vector< std::vector<double> > tl_;
  std::vector< std::vector<double> > tu_;
  std::map< std::string, convex_pair_t > convexes;
  std::map< std::string, std::pair<unsigned int, std::shared_ptr<sch::STP_BV> > > stpbvs;
  std::map<int, sva::PTransformd> collisionTransforms;
  std::map<std::string, mc_rbdyn::SurfacePtr> surfaces_;
  std::vector<ForceSensor> forceSensors;
  std::string accelerometerBody;
  Springs springs;
  std::vector< std::vector<Eigen::VectorXd> > tlPoly;
  std::vector< std::vector<Eigen::VectorXd> > tuPoly;
  std::vector<Flexibility> flexibility_;
  std::map<std::string, unsigned int> jointIndexByNameD;
  std::map<std::string, unsigned int> bodyIndexByNameD;
  std::map<std::string, ForceSensor> forceSensorsParentD;
  std::map<std::string, std::string> parentBodyForceSensorD;
public:
  Robot() {}
};

struct Robots
{
public:
  Robots() {}
  Robots(const std::vector<mc_rbdyn::Robot> & robots, int robotIndex = -1, int envIndex = -1);
  Robots(const Robots & rhs);
  Robots & operator=(const Robots & rhs);

  Robot & robot();

  const Robot & robot() const;

  const Robot & env() const;

  Robot & env();
public:
  std::vector<mc_rbdyn::Robot> robots;
  std::vector<rbd::MultiBody> mbs;
  std::vector<rbd::MultiBodyConfig> mbcs;
  unsigned int robotIndex;
  unsigned int envIndex;
};

std::vector< std::vector<double> > jointsParameters(const rbd::MultiBody & mb, const double & coeff);

std::vector< std::vector<double> > jointsDof(const rbd::MultiBody & mb, const double & coeff);

std::map<int, std::vector<double> > jointsVectorToId(const rbd::MultiBody & mb, const std::vector< std::vector<double> > & jointsVec,
                                                              const std::function<bool(const rbd::Joint &, const std::vector<double> &)> & filter
                                                                        = [](const rbd::Joint &, const std::vector<double> &){return true;});

std::vector< std::vector<double> > jointsIdToVector(const rbd::MultiBody & mb, std::map<int, std::vector<double> > & jointsId, const std::vector<double> & def = {}, const std::function<bool (const rbd::Joint &)> & filter = [](const rbd::Joint &){return true;} );

// Return [ql, qu, vl, vu, tl, tu]
std::vector< std::map< int, std::vector<double> > > defaultBounds(const rbd::MultiBody & mb);

template<typename sch_T>
void applyTransformToSchById(const rbd::MultiBody & mb, const rbd::MultiBodyConfig & mbc, std::map<std::string, std::pair<unsigned int, sch_T> > & schById);

Robot loadRobot(const RobotModule & module, const std::string & surfaceDir, sva::PTransformd * base = 0, int bId = -1);

/*FIXME Not implemetend for now, only used for ATLAS
void loadPolyTorqueBoundsData(const std::string & file, Robot & robot);
*/

void loadRobotAndEnv(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, Robot & robot, Robot & env);

void loadRobotAndEnv(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, sva::PTransformd * base, int bId, Robot & robot, Robot & env);

Robots loadRobots(const std::vector<std::shared_ptr<RobotModule>> & modules, const std::vector<std::string> & surfaceDirs);

Robot loadRobotFromUrdf(const std::string & urdf, bool withVirtualLinks = true, const std::vector<std::string> & filteredLinks = {}, bool fixed = false, sva::PTransformd * base = 0, int bId = -1);

}

#include <mc_rbdyn/robot.hpp>

#endif
