#include <mc_rbdyn/robot.h>

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rbdyn/surface_utils.h>
#include <mc_rbdyn_urdf/urdf.h>

#include <mc_rtc/logging.h>

#include <RBDyn/FK.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_rbdyn
{

// We can safely ignore those since they are due to different index types and
// our index never go near unsafe territories
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#ifdef __clang__
#pragma clang diagnostic ignored "-Wshorten-64-to-32"
#endif

ForceSensor::ForceSensor()
: ForceSensor("", "", sva::PTransformd())
{
}

ForceSensor::ForceSensor(const std::string & n, const std::string & pn, const sva::PTransformd & xpf)
: sensorName(n), parentBodyName(pn), X_p_f(xpf)
{
}

Robot::Robot(const std::string & name, Robots & robots, unsigned int robots_idx,
        const std::map<int, sva::PTransformd> & bodyTransforms,
        const std::vector< std::vector<double> > & ql, const std::vector< std::vector<double> > & qu,
        const std::vector< std::vector<double> > & vl, const std::vector< std::vector<double> > & vu,
        const std::vector< std::vector<double> > & tl, const std::vector< std::vector<double> > & tu,
        const std::map<std::string, convex_pair_t> & convexes,
        const std::map<std::string, stpbv_pair_t> & stpbvs,
        const std::map<int, sva::PTransformd> & collisionTransforms, const std::map<std::string, std::shared_ptr<mc_rbdyn::Surface> > & surfaces,
        const std::vector<ForceSensor> & forceSensors,
        const std::map<unsigned int, std::vector<double>> stance,
        const std::string & accelerometerBody,
        const Springs & springs, const std::vector< std::vector<Eigen::VectorXd> > & tlPoly,
        const std::vector< std::vector<Eigen::VectorXd> > & tuPoly, const std::vector<Flexibility> & flexibility)
: name_(name), robots(robots), robots_idx(robots_idx),
  bodyTransforms(bodyTransforms), ql_(ql), qu_(qu), vl_(vl), vu_(vu), tl_(tl), tu_(tu),
  convexes(convexes), stpbvs(stpbvs), collisionTransforms(collisionTransforms), surfaces_(),
  forceSensors(forceSensors), stance_(stance), _accelerometerBody(accelerometerBody), springs(springs), tlPoly(tlPoly),
  tuPoly(tuPoly), flexibility_(flexibility)
{
  // Copy the surfaces
  for(const auto & p : surfaces)
  {
    this->surfaces_[p.first] = p.second->copy();
  }
  for(unsigned int i = 0; i < this->mb().joints().size(); ++i)
  {
    jointIndexByNameD[this->mb().joints()[i].name()] = i;
  }
  for(unsigned int i = 0; i < this->mb().bodies().size(); ++i)
  {
    bodyIndexByNameD[this->mb().bodies()[i].name()] = i;
  }
  for(const ForceSensor & sensor : forceSensors)
  {
    forceSensorsParentD[sensor.sensorName] = sensor;
    parentBodyForceSensorD[sensor.parentBodyName] = sensor.sensorName;
  }
  if(this->_accelerometerBody == "" && this->hasBody("Accelerometer"))
  {
    unsigned int index = bodyIndexByName("Accelerometer");
    this->_accelerometerBody = this->mb().body(this->mb().parent(index)).name();
  }
}

std::string Robot::name() const
{
  return name_;
}

const std::string & Robot::accelerometerBody() const
{
  return _accelerometerBody;
}

bool Robot::hasJoint(const std::string & name) const
{
  return jointIndexByNameD.count(name) != 0;
}

bool Robot::hasBody(const std::string & name) const
{
  return bodyIndexByNameD.count(name) != 0;
}

unsigned int Robot::jointIndexByName(const std::string & name) const
{
  return jointIndexByNameD.at(name);
}

unsigned int Robot::bodyIndexByName(const std::string & name) const
{
  return bodyIndexByNameD.at(name);
}

int Robot::jointIdByName(const std::string & name) const
{
  return mb().joint(jointIndexByNameD.at(name)).id();
}

int Robot::bodyIdByName(const std::string & name) const
{
  return mb().body(bodyIndexByNameD.at(name)).id();
}

std::string Robot::forceSensorParentBodyName(const std::string & fs) const
{
  return forceSensorsParentD.at(fs).parentBodyName;
}

const ForceSensor & Robot::forceSensorData(const std::string & fs) const
{
  return forceSensorsParentD.at(fs);
}

bool Robot::hasForceSensor(const std::string & body) const
{
  return parentBodyForceSensorD.count(body) != 0;
}

std::string Robot::forceSensorByBody(const std::string & body) const
{
  return parentBodyForceSensorD.at(body);
}

std::vector<std::string> Robot::forceSensorsByName() const
{
  std::vector<std::string> res;
  for(const auto & fs : forceSensors)
  {
    res.push_back(fs.sensorName);
  }
  return res;
}

rbd::MultiBody & Robot::mb()
{
  return robots.mbs_[robots_idx];
}
const rbd::MultiBody & Robot::mb() const
{
  return robots.mbs_[robots_idx];
}

rbd::MultiBodyConfig & Robot::mbc()
{
  return robots.mbcs_[robots_idx];
}
const rbd::MultiBodyConfig & Robot::mbc() const
{
  return robots.mbcs_[robots_idx];
}

rbd::MultiBodyGraph & Robot::mbg()
{
  return robots.mbgs_[robots_idx];
}
const rbd::MultiBodyGraph & Robot::mbg() const
{
  return robots.mbgs_[robots_idx];
}

const std::vector<std::vector<double>> & Robot::ql() const
{
  return ql_;
}
const std::vector<std::vector<double>> & Robot::qu() const
{
  return qu_;
}
const std::vector<std::vector<double>> & Robot::vl() const
{
  return vl_;
}
const std::vector<std::vector<double>> & Robot::vu() const
{
  return vu_;
}
const std::vector<std::vector<double>> & Robot::tl() const
{
  return tl_;
}
const std::vector<std::vector<double>> & Robot::tu() const
{
  return tu_;
}

const std::vector<Flexibility> & Robot::flexibility() const
{
  return flexibility_;
}

bool Robot::hasSurface(const std::string & surface) const
{
  return surfaces_.count(surface) != 0;
}

mc_rbdyn::Surface & Robot::surface(const std::string & sName)
{
  return const_cast<mc_rbdyn::Surface&>(static_cast<const Robot*>(this)->surface(sName));
}
const mc_rbdyn::Surface & Robot::surface(const std::string & sName) const
{
  if(!hasSurface(sName))
  {
    LOG_ERROR("No surface named " << sName << " found in this robot")
    throw("Surface does not exist");
  }
  return *(surfaces_.at(sName));
}

const std::map<std::string, SurfacePtr> & Robot::surfaces() const
{
  return surfaces_;
}

Robot::convex_pair_t & Robot::convex(const std::string & cName)
{
  return const_cast<Robot::convex_pair_t &>(static_cast<const Robot*>(this)->convex(cName));
}
const Robot::convex_pair_t & Robot::convex(const std::string & cName) const
{
  if(convexes.count(cName) == 0)
  {
    LOG_ERROR("No convex named " << cName << " found in this robot (" << this->name_ << ")")
    LOG_WARNING("Convexes available are:")
    for(const auto & cv : convexes)
    {
      LOG_WARNING(cv.first)
    }
    throw("Convex does not exist");
  }
  return convexes.at(cName);
}

const sva::PTransformd & Robot::bodyTransform(int id) const
{
  if(bodyTransforms.count(id) == 0)
  {
    LOG_ERROR("No body transform with id " << id << " found in this robot")
    throw("Body transform does not exist");
  }
  return bodyTransforms.at(id);
}

const sva::PTransformd & Robot::collisionTransform(int id) const
{
  if(collisionTransforms.count(id) == 0)
  {
    LOG_ERROR("No collision transform with id " << id << " found in this robot")
    throw("Collision transform does not exist");
  }
  return collisionTransforms.at(id);
}

void Robot::fixSurfaces()
{
  for(auto & s : surfaces_)
  {
    unsigned int bodyId = bodyIdByName(s.second->bodyName());
    const sva::PTransformd & trans = bodyTransforms[bodyId];
    s.second->X_b_s(s.second->X_b_s()*trans);
  }
}

void Robot::loadRSDFFromDir(const std::string & surfaceDir)
{
  std::vector<SurfacePtr> surfacesIn = readRSDFFromDir(surfaceDir);
  for(const auto & sp : surfacesIn)
  {
    /* Check coherence of surface with mb */
    if(hasBody(sp->bodyName()))
    {
      surfaces_[sp->name()] = sp;
    }
    else
    {
      LOG_WARNING("Loaded surface " << sp->name() << " attached to body " << sp->bodyName() << " from RSDF but the robot " << name() << " has no such body, discard this surface to avoid future problems...")
    }
  }
}

std::map<unsigned int, std::vector<double>> Robot::stance() const
{
  return stance_;
}

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
  this->mbs_.push_back(robots.robot(robots_idx).mbg().makeMultiBody(base.baseId, base.baseType, baseAxis, base.X_0_s, base.X_b0_s));
  this->mbcs_.emplace_back(this->mbs_.back());
  this->mbgs_.push_back(robots.robot(robots_idx).mbg());
  robots.robot(robots_idx).createWithBase(*this, static_cast<unsigned int>(this->mbs_.size()) - 1, base);
}

void Robot::createWithBase(Robots & robots, unsigned int robots_idx, const Base & base) const
{
  rbd::MultiBody & mb = robots.mbs_[robots_idx];
  rbd::MultiBodyConfig & mbc = robots.mbcs_[robots_idx];
  rbd::MultiBodyGraph & mbg = robots.mbgs_[robots_idx];
  mbc.zero(mb);
  std::map<int, sva::PTransformd> bodyTransforms = mbg.bodiesBaseTransform(base.baseId, base.X_b0_s);

  typedef std::vector< std::vector<double> > bound_t;
  auto convertBound = [](const rbd::MultiBody & oldMb, const rbd::MultiBody & newMb, const bound_t & oldBound, const std::vector<double> & baseBound)
  {
    bound_t newBound;
    newBound.resize(oldBound.size());
    newBound[0] = baseBound;
    for(size_t i = 1; i < oldBound.size(); ++i)
    {
      newBound[static_cast<size_t>(newMb.jointIndexById(oldMb.joint(static_cast<int>(i)).id()))] = oldBound[i];
    }
    return newBound;
  };

  int jParam = mb.joint(0).params();
  int jDof = mb.joint(0).dof();
  bound_t ql = convertBound(this->mb(), mb, this->ql(), std::vector<double>(jParam, -INFINITY));
  bound_t qu = convertBound(this->mb(), mb, this->qu(), std::vector<double>(jParam, INFINITY));
  bound_t vl = convertBound(this->mb(), mb, this->vl(), std::vector<double>(jDof, -INFINITY));
  bound_t vu = convertBound(this->mb(), mb, this->vu(), std::vector<double>(jDof, INFINITY));
  bound_t tl = convertBound(this->mb(), mb, this->tl(), std::vector<double>(jDof, -INFINITY));
  bound_t tu = convertBound(this->mb(), mb, this->tu(), std::vector<double>(jDof, INFINITY));

  robots.robots_.emplace_back(this->name_, robots, robots_idx, bodyTransforms,
              ql, qu, vl, vu, tl, tu,
              this->convexes, this->stpbvs, this->collisionTransforms,
              this->surfaces_, this->forceSensors, this->stance_,
              this->_accelerometerBody, this->springs,
              this->tlPoly, this->tuPoly,
              this->flexibility());
  robots.robot(robots_idx).fixSurfaces();
}

void Robots::robotCopy(const Robots & robots, unsigned int robots_idx)
{
  this->mbs_.push_back(robots.robot(robots_idx).mb());
  this->mbcs_.push_back(robots.robot(robots_idx).mbc());
  this->mbgs_.push_back(robots.robot(robots_idx).mbg());
  robots.robot(robots_idx).copy(*this, static_cast<unsigned int>(this->mbs_.size()) - 1);
}

void Robot::copy(Robots & robots, unsigned int robots_idx) const
{
  robots.robots_.emplace_back(this->name_, robots, robots_idx, this->bodyTransforms, this->ql(), this->qu(), this->vl(), this->vu(), this->tl(), this->tu(), this->convexes, this->stpbvs, this->collisionTransforms, this->surfaces_, this->forceSensors, this->stance_, this->_accelerometerBody, this->springs, this->tlPoly, this->tuPoly, this->flexibility());
}

std::vector< std::vector<double> > jointsParameters(const rbd::MultiBody & mb, const double & coeff)
{
  std::vector< std::vector<double> > res;
  for(const rbd::Joint & j : mb.joints())
  {
    res.push_back(std::vector<double>(j.params(), coeff));
  }
  return res;
}

std::vector< std::vector<double> > jointsDof(const rbd::MultiBody & mb, const double & coeff)
{
  std::vector< std::vector<double> > res;
  for(const rbd::Joint & j : mb.joints())
  {
    res.push_back(std::vector<double>(j.dof(), coeff));
  }
  return res;
}

std::map<int, std::vector<double> > jointsVectorToId(const rbd::MultiBody & mb, const std::vector< std::vector<double> > & jointsVec,
                                                              const std::function<bool(const rbd::Joint &, const std::vector<double> &)> & filter)
{
  std::map<int, std::vector<double> > res;
  for(size_t i = 0; i < std::min(jointsVec.size(), mb.joints().size()); ++i)
  {
    if(filter(mb.joints()[i], jointsVec[i]))
    {
      res[mb.joints()[i].id()] = jointsVec[i];
    }
  }
  return res;
}

std::vector< std::vector<double> > jointsIdToVector(const rbd::MultiBody & mb, std::map<int, std::vector<double> > & jointsId, const std::vector<double> & def, const std::function<bool (const rbd::Joint &)> & filter)
{
  std::vector< std::vector<double> > res;
  for(const rbd::Joint & j : mb.joints())
  {
    if(filter(j))
    {
      // Mimic python setdefault behaviour
      if(jointsId.count(j.id()) == 0)
      {
        jointsId[j.id()] = def;
      }
      res.push_back(jointsId[j.id()]);
    }
  }
  return res;
}

// Return [ql, qu, vl, vu, tl, tu]
std::vector< std::map< int, std::vector<double> > > defaultBounds(const rbd::MultiBody & mb)
{
  std::vector< std::map< int, std::vector<double> > > res;
  auto jParam = [mb](const double & coeff) { return jointsVectorToId(mb, jointsParameters(mb, coeff)); };
  auto jDof = [mb](const double & coeff) { return jointsVectorToId(mb, jointsDof(mb, coeff)); };
  res.push_back(jParam(-INFINITY));
  res.push_back(jParam(INFINITY));
  res.push_back(jDof(-INFINITY));
  res.push_back(jDof(INFINITY));
  res.push_back(jDof(-0));
  res.push_back(jDof(0));
  return res;
}

Robot& Robots::load(const RobotModule & module, const std::string &, sva::PTransformd * base, int bId)
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
    unsigned int baseId = bId < 0 ? mb.body(0).id() : bId;
    mb = mbg.makeMultiBody(baseId, mb.joint(0).type() == rbd::Joint::Fixed, *base);
    mbc = rbd::MultiBodyConfig(mb);
    mbc.zero(mb);
  }

  auto bodyTransforms = mbg.bodiesBaseTransform(mb.body(0).id());

  auto defBounds = defaultBounds(mb);
  {
    auto rbounds = module.bounds();
    for(size_t i = 0; i < rbounds.size(); ++i)
    {
      for(const std::pair<const unsigned int, std::vector<double> > & b : rbounds[i])
      {
        defBounds[i][b.first] = b.second;
      }
    }
  }
  auto ql = jointsIdToVector(mb, defBounds[0]);
  auto qu = jointsIdToVector(mb, defBounds[1]);
  auto vl = jointsIdToVector(mb, defBounds[2]);
  auto vu = jointsIdToVector(mb, defBounds[3]);
  auto tl = jointsIdToVector(mb, defBounds[4]);
  auto tu = jointsIdToVector(mb, defBounds[5]);

  std::map< int, std::vector<double> > initQByJointsId;
  for(const rbd::Joint & j : mb.joints())
  {
    initQByJointsId[j.id()] = j.zeroParam();
  }
  {
    auto initQ = module.stance();
    for(const auto & qi : initQ)
    {
      initQByJointsId[qi.first] = qi.second;
    }
  }
  auto initQ = jointsIdToVector(mb, initQByJointsId);
  mbc.q = initQ;
  rbd::forwardKinematics(mb, mbc);

  std::map<std::string, unsigned int> bodyIdByName;
  for(const rbd::Body & b : mb.bodies())
  {
    bodyIdByName[b.name()] = b.id();
  }

  std::map<std::string, Robot::convex_pair_t> convexesByName;
  {
    for(const auto & p : module.convexHull())
    {
      if(bodyIdByName.count(p.second.first))
      {
        std::shared_ptr<sch::S_Polyhedron> poly(sch::mc_rbdyn::Polyhedron(p.second.second));
        convexesByName[p.first] = Robot::convex_pair_t(bodyIdByName[p.second.first], poly);
      }
    }
    applyTransformToSchById(mb, mbc, convexesByName);
  }

  std::map<std::string, Robot::stpbv_pair_t> stpbvsByName;
  {
    for(const auto & p : module.stpbvHull())
    {
      if(bodyIdByName.count(p.second.first))
      {
        std::shared_ptr<sch::STP_BV> stpbvs(sch::mc_rbdyn::STPBV(p.second.second));
        stpbvsByName[p.first] = Robot::stpbv_pair_t(bodyIdByName[p.second.first], stpbvs);
      }
    }
    applyTransformToSchById(mb, mbc, stpbvsByName);
  }

  std::map<int, sva::PTransformd> collisionTransforms;
  for(const auto & b : mb.bodies())
  {
    collisionTransforms[b.id()] = sva::PTransformd::Identity();
  }
  {
    for(const auto & p : module.collisionTransforms())
    {
      collisionTransforms[p.first] = p.second;
    }
  }

  const std::vector<Flexibility> & flexibility = module.flexibility();

  const std::vector<ForceSensor> & forceSensors = module.forceSensors();

  const std::string & accelBody = module.accelerometerBody();

  const Springs & springs = module.springs();

  const auto & stance = module.stance();

  std::map<std::string, SurfacePtr> surf;
  std::vector< std::vector<Eigen::VectorXd> > tlPoly;
  std::vector< std::vector<Eigen::VectorXd> > tuPoly;
  robots_.emplace_back(module.name, *this, this->mbs_.size() - 1,
                      bodyTransforms, ql, qu, vl, vu, tl, tu,
                      convexesByName, stpbvsByName, collisionTransforms,
                      surf, forceSensors, stance, accelBody, springs,
                      tlPoly, tuPoly, flexibility);
  robots_.back().loadRSDFFromDir(module.rsdf_dir);
  updateIndexes();
  return robots_.back();
}

/*void loadPolyTorqueBoundsData(const std::string & file, Robot & robot)
{
}*/

std::shared_ptr<Robots> loadRobot(const RobotModule & module, const std::string & surfaceDir, sva::PTransformd * base, int bId)
{
  auto robots = std::make_shared<Robots>();
  robots->load(module, surfaceDir, base, bId);
  return robots;
}

void Robots::load(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir)
{
  load(module, surfaceDir, envModule, envSurfaceDir, 0, -1);
}

std::shared_ptr<Robots> loadRobotAndEnv(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir)
{
  auto robots = std::make_shared<Robots>();
  robots->load(module, surfaceDir, envModule, envSurfaceDir);
  return robots;
}

void Robots::load(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, sva::PTransformd * base, int bId)
{
  load(module, surfaceDir, base, bId);
  load(envModule, envSurfaceDir);
}

std::shared_ptr<Robots> loadRobotAndEnv(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, sva::PTransformd * base, int bId)
{
  auto robots = std::make_shared<Robots>();
  robots->load(module, surfaceDir, envModule, envSurfaceDir, base, bId);
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

template<typename X, typename Y>
inline void update(std::map<X,Y> & oldData, const std::map<X,Y> & nData)
{
  for(const auto & p : nData)
  {
    oldData[p.first] = p.second;
  }
}

Robot& Robots::loadFromUrdf(const std::string & name, const std::string & urdf, bool withVirtualLinks, const std::vector<std::string> & filteredLinks, bool fixed, sva::PTransformd * base, int bId)
{
  mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf, fixed, filteredLinks, true, "", withVirtualLinks);
  int baseId = res.mb.body(0).id();
  if(base)
  {
    baseId = bId >= 0 ? bId : baseId;
    res.mb = res.mbg.makeMultiBody(baseId, fixed, *base);
    res.mbc = rbd::MultiBodyConfig(res.mb);
    res.mbc.zero(res.mb);
  }
  std::map<int, sva::PTransformd> bodyTransforms = res.mbg.bodiesBaseTransform(baseId);

  auto defBounds = defaultBounds(res.mb);
  std::map< int, std::vector<double> > qlt = res.limits.lower;
  std::map< int, std::vector<double> > qut = res.limits.upper;
  std::map< int, std::vector<double> > vlt = res.limits.velocity;
  for(auto & vl : vlt)
  {
    for(auto & v : vl.second)
    {
      v = -v;
    }
  }
  std::map< int, std::vector<double> > vut = res.limits.velocity;
  std::map< int, std::vector<double> > tlt = res.limits.torque;
  for(auto & tl : tlt)
  {
    for(auto & t : tl.second)
    {
      t= -t;
    }
  }
  std::map< int, std::vector<double> > tut = res.limits.torque;
  update(defBounds[0], qlt);
  update(defBounds[1], qut);
  update(defBounds[2], vlt);
  update(defBounds[3], vut);
  update(defBounds[4], tlt);
  update(defBounds[5], tut);

  std::vector< std::vector<double> > ql = jointsIdToVector(res.mb, defBounds[0]);
  std::vector< std::vector<double> > qu = jointsIdToVector(res.mb, defBounds[1]);
  std::vector< std::vector<double> > vl = jointsIdToVector(res.mb, defBounds[2]);
  std::vector< std::vector<double> > vu = jointsIdToVector(res.mb, defBounds[3]);
  std::vector< std::vector<double> > tl = jointsIdToVector(res.mb, defBounds[4]);
  std::vector< std::vector<double> > tu = jointsIdToVector(res.mb, defBounds[5]);

  mbs_.push_back(res.mb);
  mbcs_.push_back(res.mbc);
  mbgs_.push_back(res.mbg);

  std::map<std::string, Robot::convex_pair_t> convex;
  std::map<std::string, Robot::stpbv_pair_t> stpbv;
  std::map<std::string, mc_rbdyn::SurfacePtr> surfaces;
  std::vector<ForceSensor> forceSensors;

  robots_.emplace_back(name, *this, mbs_.size() - 1,
               bodyTransforms, ql, qu, vl, vu, tl, tu,
               convex, stpbv, res.collision_tf, surfaces, forceSensors);
  updateIndexes();
  return robots_.back();
}

std::shared_ptr<Robots> loadRobotFromUrdf(const std::string & name, const std::string & urdf, bool withVirtualLinks, const std::vector<std::string> & filteredLinks, bool fixed, sva::PTransformd * base, int bId)
{
  auto robots = std::make_shared<Robots>();
  robots->loadFromUrdf(name, urdf, withVirtualLinks, filteredLinks, fixed, base, bId);
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
