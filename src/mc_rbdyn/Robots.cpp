#include <mc_rbdyn/Robots.h>

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/SCHAddon.h>

#include <mc_rtc/logging.h>

#include <RBDyn/FK.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace
{
  template<typename sch_T>
  void applyTransformToSchById(const rbd::MultiBody & mb, const rbd::MultiBodyConfig & mbc, std::map<std::string, std::pair<std::string, std::shared_ptr<sch_T> > > & schByName)
  {
    for(auto & p : schByName)
    {
      unsigned int index = static_cast<unsigned int>(mb.bodyIndexByName(p.second.first));
      sch::mc_rbdyn::transform(*(p.second.second.get()), mbc.bodyPosW[index]);
    }
  }

  template<typename X, typename Y>
  inline void update(std::map<X,Y> & oldData, const std::map<X,Y> & nData)
  {
    for(const auto & p : nData)
    {
      oldData[p.first] = p.second;
    }
  }

}

namespace mc_rbdyn
{

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#ifdef __clang__
#pragma clang diagnostic ignored "-Wshorten-64-to-32"
#endif

Robots::Robots()
: robots_(), mbs_(), mbcs_(), robotIndex_(0), envIndex_(0)
{
}

Robots::Robots(const Robots & rhs)
: robots_(), mbs_(rhs.mbs_), mbcs_(rhs.mbcs_), robotIndex_(rhs.robotIndex_), envIndex_(rhs.envIndex_)
{
  for(unsigned int i = 0; i < rhs.robots_.size(); ++i)
  {
    const Robot & robot = rhs.robots_[i];
    robot.copy(*this, i);
  }
}

Robots & Robots::operator=(const Robots & rhs)
{
  if(&rhs == this) { return *this; }
  robots_.clear();
  mbs_ = rhs.mbs_;
  mbcs_ = rhs.mbcs_;
  mbgs_ = rhs.mbgs_;
  robotIndex_ = rhs.robotIndex_;
  envIndex_ = rhs.envIndex_;
  for(unsigned int i = 0; i < rhs.robots_.size(); ++i)
  {
    const Robot & robot = rhs.robots_[i];
    robot.copy(*this, i);
  }
  return *this;
}

std::vector<Robot> & Robots::robots()
{
  return robots_;
}
const std::vector<Robot> & Robots::robots() const
{
  return robots_;
}

std::vector<rbd::MultiBody> & Robots::mbs()
{
  return mbs_;
}
const std::vector<rbd::MultiBody> & Robots::mbs() const
{
  return mbs_;
}

std::vector<rbd::MultiBodyConfig> & Robots::mbcs()
{
  return mbcs_;
}
const std::vector<rbd::MultiBodyConfig> & Robots::mbcs() const
{
  return mbcs_;
}

unsigned int Robots::robotIndex() const
{
  return robotIndex_;
}
unsigned int Robots::envIndex() const
{
  return envIndex_;
}

Robot & Robots::robot()
{
  return robots_[robotIndex_];
}
const Robot & Robots::robot() const
{
  return robots_[robotIndex_];
}

Robot & Robots::env()
{
  return robots_[envIndex_];
}
const Robot & Robots::env() const
{
  return robots_[envIndex_];
}

Robot & Robots::robot(unsigned int idx)
{
  return const_cast<Robot&>(static_cast<const Robots*>(this)->robot(idx));
}
const Robot & Robots::robot(unsigned int idx) const
{
  if(idx >= robots_.size())
  {
    LOG_ERROR("No robot with index " << idx << " (" << robots_.size()  << " robots loaded)")
    throw("Wrong robot index");
  }
  return robots_[idx];
}

void Robots::createRobotWithBase(Robots & robots, unsigned int robots_idx, const Base & base, const Eigen::Vector3d & baseAxis)
{
  createRobotWithBase(robots.robot(robots_idx), base, baseAxis);
}

void Robots::createRobotWithBase(Robot & robot, const Base & base, const Eigen::Vector3d & baseAxis)
{
  this->mbs_.push_back(robot.mbg().makeMultiBody(base.baseName, base.baseType, baseAxis, base.X_0_s, base.X_b0_s));
  this->mbcs_.emplace_back(this->mbs_.back());
  this->mbgs_.push_back(robot.mbg());
  robot.createWithBase(*this, static_cast<unsigned int>(this->mbs_.size()) - 1, base);
}

void Robots::removeRobot(const std::string & name)
{
  auto it = std::find_if(robots_.begin(), robots_.end(),
                         [&name](const Robot & r){ return r.name() == name; });
  if(it != robots_.end())
  {
    removeRobot(it->robots_idx);
  }
  else
  {
    LOG_ERROR("Did not find a robot named " << name << " to remove")
  }
}

void Robots::removeRobot(unsigned int idx)
{
  if(idx >= robots_.size())
  {
    LOG_ERROR("Cannot remove a robot at index " << idx << " because there is " << robots_.size() << " robots loaded")
    return;
  }
  robots_.erase(robots_.begin() + idx);
  mbs_.erase(mbs_.begin() + idx);
  mbcs_.erase(mbcs_.begin() + idx);
  mbgs_.erase(mbgs_.begin() + idx);
  for(unsigned int i = idx; i < robots_.size(); ++i)
  {
    auto & r = robots_[i];
    r.robots_idx--;
  }
}

void Robots::robotCopy(const Robot & robot)
{
  this->mbs_.push_back(robot.mb());
  this->mbcs_.push_back(robot.mbc());
  this->mbgs_.push_back(robot.mbg());
  robot.copy(*this, static_cast<unsigned int>(this->mbs_.size()) - 1);
}

Robot& Robots::load(const RobotModule & module, const std::string &, sva::PTransformd * base,
    const std::string& bName)
{
  mbs_.emplace_back(module.mb);
  mbcs_.emplace_back(module.mbc);
  mbgs_.emplace_back(module.mbg);

  rbd::MultiBody & mb = mbs_.back();
  rbd::MultiBodyConfig & mbc = mbcs_.back();
  rbd::MultiBodyGraph & mbg = mbgs_.back();

  mbc.zero(mb);

  if(base)
  {
    std::string baseName = bName.empty() ? mb.body(0).name() : bName;
    mb = mbg.makeMultiBody(baseName, mb.joint(0).type() == rbd::Joint::Fixed, *base);
    mbc = rbd::MultiBodyConfig(mb);
    mbc.zero(mb);
  }

  auto bodyTransforms = mbg.bodiesBaseTransform(mb.body(0).name());

  auto defBounds = defaultBounds(mb);
  {
    auto rbounds = module.bounds();
    for(size_t i = 0; i < rbounds.size(); ++i)
    {
      for(const std::pair<const std::string, std::vector<double> > & b : rbounds[i])
      {
        defBounds[i][b.first] = b.second;
      }
    }
  }
  auto ql = jointsNameToVector(mb, defBounds[0]);
  auto qu = jointsNameToVector(mb, defBounds[1]);
  auto vl = jointsNameToVector(mb, defBounds[2]);
  auto vu = jointsNameToVector(mb, defBounds[3]);
  auto tl = jointsNameToVector(mb, defBounds[4]);
  auto tu = jointsNameToVector(mb, defBounds[5]);

  std::map< std::string, std::vector<double> > initQByJointsName;
  for(const rbd::Joint & j : mb.joints())
  {
    initQByJointsName[j.name()] = j.zeroParam();
  }
  {
    auto initQ = module.stance();
    for(const auto & qi : initQ)
    {
      initQByJointsName[qi.first] = qi.second;
    }
  }
  auto initQ = jointsNameToVector(mb, initQByJointsName);
  mbc.q = initQ;
  rbd::forwardKinematics(mb, mbc);

  std::map<std::string, Robot::convex_pair_t> convexesByName;
  {
    for(const auto & p : module.convexHull())
    {
      if(module.mb.bodyIndexByName().count(p.second.first))
      {
        std::shared_ptr<sch::S_Polyhedron> poly(sch::mc_rbdyn::Polyhedron(p.second.second));
        convexesByName[p.first] = Robot::convex_pair_t(p.second.first, poly);
      }
    }
    applyTransformToSchById(mb, mbc, convexesByName);
  }

  std::map<std::string, Robot::stpbv_pair_t> stpbvsByName;
  {
    for(const auto & p : module.stpbvHull())
    {
      if(module.mb.bodyIndexByName().count(p.second.first))
      {
        std::shared_ptr<sch::STP_BV> stpbvs(sch::mc_rbdyn::STPBV(p.second.second));
        stpbvsByName[p.first] = Robot::stpbv_pair_t(p.second.first, stpbvs);
      }
    }
    applyTransformToSchById(mb, mbc, stpbvsByName);
  }

  std::map<std::string, sva::PTransformd> collisionTransforms;
  for(const auto & b : mb.bodies())
  {
    collisionTransforms[b.name()] = sva::PTransformd::Identity();
  }
  {
    for(const auto & p : module.collisionTransforms())
    {
      collisionTransforms[p.first] = p.second;
    }
  }

  const std::vector<Flexibility> & flexibility = module.flexibility();

  std::vector<ForceSensor> forceSensors = module.forceSensors();
  for(auto & fs : forceSensors)
  {
    bfs::path calib_file = bfs::path(module.calib_dir) / std::string("calib_data." + fs.name());
    if(bfs::exists(calib_file))
    {
      fs.loadCalibrator(calib_file.string(), mbc.gravity);
    }
  }

  const BodySensorVector & bodySensors = module.bodySensors();

  const Springs & springs = module.springs();

  const auto & refJointOrder = module.ref_joint_order();

  const auto & stance = module.stance();

  std::map<std::string, SurfacePtr> surf;
  std::vector< std::vector<Eigen::VectorXd> > tlPoly;
  std::vector< std::vector<Eigen::VectorXd> > tuPoly;
  robots_.emplace_back(module.name, *this, this->mbs_.size() - 1,
                      bodyTransforms, ql, qu, vl, vu, tl, tu,
                      convexesByName, stpbvsByName, collisionTransforms,
                      surf, forceSensors, refJointOrder, stance, bodySensors, springs,
                      tlPoly, tuPoly, flexibility);
  robots_.back().loadRSDFFromDir(module.rsdf_dir);
  updateIndexes();
  return robots_.back();
}

/*void loadPolyTorqueBoundsData(const std::string & file, Robot & robot)
{
}*/

std::shared_ptr<Robots> loadRobot(const RobotModule & module, const std::string & surfaceDir, sva::PTransformd * base, const std::string& baseName)
{
  auto robots = std::make_shared<Robots>();
  robots->load(module, surfaceDir, base, baseName);
  return robots;
}

void Robots::load(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir)
{
  load(module, surfaceDir, envModule, envSurfaceDir, nullptr, "Root");
}

std::shared_ptr<Robots> loadRobotAndEnv(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir)
{
  auto robots = std::make_shared<Robots>();
  robots->load(module, surfaceDir, envModule, envSurfaceDir);
  return robots;
}

void Robots::load(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, sva::PTransformd * base, const std::string& baseName)
{
  load(module, surfaceDir, base, baseName);
  load(envModule, envSurfaceDir);
}

std::shared_ptr<Robots> loadRobotAndEnv(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, sva::PTransformd * base, const std::string& baseName)
{
  auto robots = std::make_shared<Robots>();
  robots->load(module, surfaceDir, envModule, envSurfaceDir, base, baseName);
  return robots;
}

void Robots::load(const std::vector<std::shared_ptr<RobotModule>> & modules, const std::vector<std::string> & surfaceDirs)
{
  for(size_t i = 0; i < modules.size(); ++i)
  {
    load(*modules[i], surfaceDirs[i]);
  }
}

std::shared_ptr<Robots> loadRobots(const std::vector<std::shared_ptr<RobotModule>> & modules, const std::vector<std::string> & surfaceDirs)
{
  auto robots = std::make_shared<Robots>();
  robots->load(modules, surfaceDirs);
  return robots;
}

Robot& Robots::loadFromUrdf(const std::string & name, const std::string & urdf, bool withVirtualLinks, const std::vector<std::string> & filteredLinks, bool fixed, sva::PTransformd * base, const std::string& baseName)
{
  mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf, fixed, filteredLinks, true, "", withVirtualLinks);
  std::string bName = res.mb.body(0).name();
  if(base)
  {
    bName = baseName.empty() ? bName : baseName;
    res.mb = res.mbg.makeMultiBody(bName, fixed, *base);
    res.mbc = rbd::MultiBodyConfig(res.mb);
    res.mbc.zero(res.mb);
  }
  std::map<std::string, sva::PTransformd> bodyTransforms = res.mbg.bodiesBaseTransform(bName);

  auto defBounds = defaultBounds(res.mb);
  std::map<std::string, std::vector<double> > qlt = res.limits.lower;
  std::map<std::string, std::vector<double> > qut = res.limits.upper;
  std::map<std::string, std::vector<double> > vlt = res.limits.velocity;
  for(auto & vl : vlt)
  {
    for(auto & v : vl.second)
    {
      v = -v;
    }
  }
  std::map<std::string, std::vector<double> > vut = res.limits.velocity;
  std::map<std::string, std::vector<double> > tlt = res.limits.torque;
  for(auto & tl : tlt)
  {
    for(auto & t : tl.second)
    {
      t= -t;
    }
  }
  std::map<std::string, std::vector<double> > tut = res.limits.torque;
  update(defBounds[0], qlt);
  update(defBounds[1], qut);
  update(defBounds[2], vlt);
  update(defBounds[3], vut);
  update(defBounds[4], tlt);
  update(defBounds[5], tut);

  std::vector< std::vector<double> > ql = jointsNameToVector(res.mb, defBounds[0]);
  std::vector< std::vector<double> > qu = jointsNameToVector(res.mb, defBounds[1]);
  std::vector< std::vector<double> > vl = jointsNameToVector(res.mb, defBounds[2]);
  std::vector< std::vector<double> > vu = jointsNameToVector(res.mb, defBounds[3]);
  std::vector< std::vector<double> > tl = jointsNameToVector(res.mb, defBounds[4]);
  std::vector< std::vector<double> > tu = jointsNameToVector(res.mb, defBounds[5]);

  mbs_.push_back(res.mb);
  mbcs_.push_back(res.mbc);
  mbgs_.push_back(res.mbg);

  std::map<std::string, Robot::convex_pair_t> convex;
  std::map<std::string, Robot::stpbv_pair_t> stpbv;
  std::map<std::string, mc_rbdyn::SurfacePtr> surfaces;
  std::vector<ForceSensor> forceSensors;
  std::vector<std::string> refJointOrder;

  robots_.emplace_back(name, *this, mbs_.size() - 1,
               bodyTransforms, ql, qu, vl, vu, tl, tu,
               convex, stpbv, res.collision_tf, surfaces, forceSensors, refJointOrder);
  updateIndexes();
  return robots_.back();
}

std::shared_ptr<Robots> loadRobotFromUrdf(const std::string & name, const std::string & urdf, bool withVirtualLinks, const std::vector<std::string> & filteredLinks, bool fixed, sva::PTransformd * base, const std::string& baseName)
{
  auto robots = std::make_shared<Robots>();
  robots->loadFromUrdf(name, urdf, withVirtualLinks, filteredLinks, fixed, base, baseName);
  return robots;
}

void Robots::updateIndexes()
{
  /* Sets robotIndex_ to the first robot with dofs != 0 and envIndex_ to the
   * last robot with dof == 0 OR the last robot */
  for(unsigned int i = 0; i < robots_.size(); ++i)
  {
    if(robots_[i].mb().nrDof())
    {
      robotIndex_ = i;
      break;
    }
  }
  envIndex_ = static_cast<unsigned int>(robots_.size()) - 1;
  for(size_t i = robots_.size(); i > 0; --i)
  {
    if(robots_[i-1].mb().nrDof() == 0)
    {
      envIndex_ = static_cast<unsigned int>(i)-1;
      break;
    }
  }
}

#pragma GCC diagnostic pop

}
