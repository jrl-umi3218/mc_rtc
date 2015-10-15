#include <mc_rbdyn/robot.h>

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rbdyn/surface_utils.h>
#include <mc_rbdyn_urdf/urdf.h>

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

Robot::Robot(const std::shared_ptr<rbd::MultiBody> & mb, const std::shared_ptr<rbd::MultiBodyConfig> & mbc, const std::shared_ptr<rbd::MultiBodyGraph> & mbg,
        const std::map<int, sva::PTransformd> & bodyTransforms,
        const std::vector< std::vector<double> > & ql, const std::vector< std::vector<double> > & qu,
        const std::vector< std::vector<double> > & vl, const std::vector< std::vector<double> > & vu,
        const std::vector< std::vector<double> > & tl, const std::vector< std::vector<double> > & tu,
        const std::map<std::string, std::pair< unsigned int, std::shared_ptr<sch::S_Polyhedron> > > & convexes,
        const std::map<std::string, std::pair< unsigned int, std::shared_ptr<sch::STP_BV> > > & stpbvs,
        const std::map<int, sva::PTransformd> & collisionTransforms, const std::map<std::string, std::shared_ptr<mc_rbdyn::Surface> > & surfaces,
        const std::vector<ForceSensor> & forceSensors, const std::string & accelerometerBody,
        const Springs & springs, const std::vector< std::vector<Eigen::VectorXd> > & tlPoly,
        const std::vector< std::vector<Eigen::VectorXd> > & tuPoly, const std::vector<Flexibility> & flexibility)
: mb_(mb), mbc_(mbc), mbg_(mbg),
  bodyTransforms(bodyTransforms), ql_(ql), qu_(qu), vl_(vl), vu_(vu), tl_(tl), tu_(tu),
  convexes(convexes), stpbvs(stpbvs), collisionTransforms(collisionTransforms), surfaces_(),
  forceSensors(forceSensors), accelerometerBody(accelerometerBody), springs(springs), tlPoly(tlPoly),
  tuPoly(tuPoly), flexibility_(flexibility)
{
  // Copy the surfaces
  for(const auto & p : surfaces)
  {
    this->surfaces_[p.first] = p.second->copy();
  }
  for(size_t i = 0; i < this->mb_->joints().size(); ++i)
  {
    jointIndexByNameD[this->mb_->joints()[i].name()] = i;
  }
  for(size_t i = 0; i < this->mb_->bodies().size(); ++i)
  {
    bodyIndexByNameD[this->mb_->bodies()[i].name()] = i;
  }
  for(const ForceSensor & sensor : forceSensors)
  {
    forceSensorsParentD[sensor.sensorName] = sensor;
    parentBodyForceSensorD[sensor.parentBodyName] = sensor.sensorName;
  }
  if(this->accelerometerBody == "" and this->hasBody("Accelerometer"))
  {
    unsigned int index = bodyIndexByName("Accelerometer");
    this->accelerometerBody = this->mb_->body(this->mb_->parent(index)).name();
  }
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

unsigned int Robot::jointIdByName(const std::string & name) const
{
  return mb_->joint(jointIndexByNameD.at(name)).id();
}

unsigned int Robot::bodyIdByName(const std::string & name) const
{
  return mb_->body(bodyIndexByNameD.at(name)).id();
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

rbd::MultiBody & Robot::mb()
{
  return *mb_;
}
const rbd::MultiBody & Robot::mb() const
{
  return *mb_;
}

rbd::MultiBodyConfig & Robot::mbc()
{
  return *mbc_;
}
const rbd::MultiBodyConfig & Robot::mbc() const
{
  return *mbc_;
}

rbd::MultiBodyGraph & Robot::mbg()
{
  return *mbg_;
}
const rbd::MultiBodyGraph & Robot::mbg() const
{
  return *mbg_;
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

mc_rbdyn::Surface & Robot::surface(const std::string & sName)
{
  return const_cast<mc_rbdyn::Surface&>(static_cast<const Robot*>(this)->surface(sName));
}
const mc_rbdyn::Surface & Robot::surface(const std::string & sName) const
{
  if(surfaces_.count(sName) == 0)
  {
    std::cerr << "No surface named " << sName << " found in this robot" << std::endl;
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
    std::cerr << "No convex named " << cName << " found in this robot" << std::endl;
    throw("Convex does not exist");
  }
  return convexes.at(cName);
}

const sva::PTransformd & Robot::bodyTransform(int id) const
{
  if(bodyTransforms.count(id) == 0)
  {
    std::cerr << "No body transform with id " << id << " found in this robot" << std::endl;
    throw("Body transform does not exist");
  }
  return bodyTransforms.at(id);
}

const sva::PTransformd & Robot::collisionTransform(int id) const
{
  if(collisionTransforms.count(id) == 0)
  {
    std::cerr << "No collision transform with id " << id << " found in this robot" << std::endl;
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


Robots::Robots(const std::vector<mc_rbdyn::Robot> & robots, int robotIndex, int envIndex)
: robots(robots), mbs(0), mbcs(0), robotIndex(0), envIndex(0)
{
  if(robotIndex >= 0) { this->robotIndex = static_cast<unsigned int>(robotIndex); }
  if(envIndex >= 0) { this->envIndex = static_cast<unsigned int>(envIndex); }
  for(size_t i = 0; i < robots.size(); ++i)
  {
    mbs.emplace_back(this->robots[i].mb());
    mbcs.emplace_back(this->robots[i].mbc());
    if(robotIndex < 0 and this->robots[i].mb().nrDof()) { this->robotIndex = i; robotIndex = 0; }
    if(envIndex < 0 and this->robots[i].mb().nrDof() == 0) { this->envIndex = i; envIndex = 0; }
  }
  /* Should be done here, because vector adresses can change */
  for(size_t i = 0; i < this->robots.size(); ++i)
  {
    /* This creates shared_ptr that have "empty" destructors */
    this->robots[i].mb_ = std::shared_ptr<rbd::MultiBody>(&mbs[i], [](const rbd::MultiBody *){});
    this->robots[i].mbc_ = std::shared_ptr<rbd::MultiBodyConfig>(&mbcs[i], [](const rbd::MultiBodyConfig *){});
  }
}

Robots::Robots(const Robots & rhs)
: robots(rhs.robots), mbs(rhs.mbs), mbcs(rhs.mbcs), robotIndex(rhs.robotIndex), envIndex(rhs.envIndex)
{
  for(size_t i = 0; i < robots.size(); ++i)
  {
    this->robots[i].mb_ = std::shared_ptr<rbd::MultiBody>(&mbs[i], [](const rbd::MultiBody *){});
    this->robots[i].mbc_ = std::shared_ptr<rbd::MultiBodyConfig>(&mbcs[i], [](const rbd::MultiBodyConfig *){});
  }
}

Robots & Robots::operator=(const Robots & rhs)
{
  if(&rhs == this) { return *this; }
  robots = rhs.robots;
  mbs = rhs.mbs;
  mbcs = rhs.mbcs;
  robotIndex = rhs.robotIndex;
  envIndex = rhs.envIndex;
  for(size_t i = 0; i < robots.size(); ++i)
  {
    this->robots[i].mb_ = std::shared_ptr<rbd::MultiBody>(&mbs[i], [](const rbd::MultiBody *){});
    this->robots[i].mbc_ = std::shared_ptr<rbd::MultiBodyConfig>(&mbcs[i], [](const rbd::MultiBodyConfig *){});
  }
  return *this;
}

Robot & Robots::robot()
{
  return robots[robotIndex];
}

const Robot & Robots::robot() const
{
  return robots[robotIndex];
}

const Robot & Robots::env() const
{
  return robots[envIndex];
}

Robot & Robots::env()
{
  return robots[envIndex];
}

Robot Robot::createWithBase(const Base & base, const Eigen::Vector3d & baseAxis) const
{
  std::shared_ptr<rbd::MultiBody> mb = std::make_shared<rbd::MultiBody>(mbg_->makeMultiBody(base.baseId, base.baseType, baseAxis, base.X_0_s, base.X_b0_s));
  std::shared_ptr<rbd::MultiBodyConfig> mbc = std::make_shared<rbd::MultiBodyConfig>(*mb);
  mbc->zero(*mb);
  std::map<int, sva::PTransformd> bodyTransforms = mbg_->bodiesBaseTransform(base.baseId, base.X_b0_s);

  typedef std::vector< std::vector<double> > bound_t;
  auto convertBound = [](const rbd::MultiBody & oldMb, const rbd::MultiBody & newMb, const bound_t & oldBound, const std::vector<double> & baseBound)
  {
    bound_t newBound;
    newBound.resize(oldBound.size());
    newBound[0] = baseBound;
    for(size_t i = 1; i < oldBound.size(); ++i)
    {
      newBound[newMb.jointIndexById(oldMb.joint(i).id())] = oldBound[i];
    }
    return newBound;
  };

  int jParam = mb->joint(0).params();
  int jDof = mb->joint(0).dof();
  bound_t ql = convertBound(this->mb(), *mb, this->ql(), std::vector<double>(jParam, -INFINITY));
  bound_t qu = convertBound(this->mb(), *mb, this->qu(), std::vector<double>(jParam, INFINITY));
  bound_t vl = convertBound(this->mb(), *mb, this->vl(), std::vector<double>(jDof, -INFINITY));
  bound_t vu = convertBound(this->mb(), *mb, this->vu(), std::vector<double>(jDof, INFINITY));
  bound_t tl = convertBound(this->mb(), *mb, this->tl(), std::vector<double>(jDof, -INFINITY));
  bound_t tu = convertBound(this->mb(), *mb, this->tu(), std::vector<double>(jDof, INFINITY));

  Robot ret(mb, mbc, std::make_shared<rbd::MultiBodyGraph>(mbg()), bodyTransforms,
              ql, qu, vl, vu, tl, tu,
              this->convexes, this->stpbvs, this->collisionTransforms,
              this->surfaces_, this->forceSensors, this->accelerometerBody,
              this->springs, this->tlPoly, this->tuPoly,
              this->flexibility());
  ret.fixSurfaces();
  return ret;
}

Robot Robot::copy() const
{
  return Robot(std::make_shared<rbd::MultiBody>(mb()), std::make_shared<rbd::MultiBodyConfig>(mbc()), std::make_shared<rbd::MultiBodyGraph>(mbg()), this->bodyTransforms, this->ql(), this->qu(), this->vl(), this->vu(), this->tl(), this->tu(), this->convexes, this->stpbvs, this->collisionTransforms, this->surfaces_, this->forceSensors, this->accelerometerBody, this->springs, this->tlPoly, this->tuPoly, this->flexibility());
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

Robot loadRobot(const RobotModule & module, const std::string & surfaceDir, sva::PTransformd * base, int bId)
{
  rbd::MultiBody mb = module.mb;
  rbd::MultiBodyConfig mbc = module.mbc;
  rbd::MultiBodyGraph mbg = module.mbg;

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

  std::map<std::string, std::pair<unsigned int, std::shared_ptr<sch::S_Polyhedron> > > convexesByName;
  {
    for(const auto & p : module.convexHull())
    {
      if(bodyIdByName.count(p.second.first))
      {
        std::shared_ptr<sch::S_Polyhedron> poly(new sch::S_Polyhedron);
        poly->constructFromFile(p.second.second);
        convexesByName[p.first] = std::pair<unsigned int, std::shared_ptr<sch::S_Polyhedron> > (bodyIdByName[p.second.first], poly);
      }
    }
    applyTransformToSchById(mb, mbc, convexesByName);
  }

  std::map<std::string, std::pair<unsigned int, std::shared_ptr<sch::STP_BV> > > stpbvsByName;
  {
    for(const auto & p : module.stpbvHull())
    {
      if(bodyIdByName.count(p.second.first))
      {
        std::shared_ptr<sch::STP_BV> stpbvs(new sch::STP_BV);
        stpbvs->constructFromFile(p.second.second);
        stpbvsByName[p.first] = std::pair<unsigned int, std::shared_ptr<sch::STP_BV> > (bodyIdByName[p.second.first], stpbvs);
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

  std::vector<SurfacePtr> surfaces = readRSDFFromDir(surfaceDir);
  std::map<std::string, SurfacePtr> surf;
  for(const auto & sp : surfaces)
  {
    surf[sp->name()] = sp;
  }

  const std::vector<Flexibility> & flexibility = module.flexibility();

  const std::vector<ForceSensor> & forceSensors = module.forceSensors();

  const std::string & accelBody = module.accelerometerBody();

  const Springs & springs = module.springs();

  return Robot(std::make_shared<rbd::MultiBody>(mb),
               std::make_shared<rbd::MultiBodyConfig>(mbc),
               std::make_shared<rbd::MultiBodyGraph>(mbg),
               bodyTransforms, ql, qu, vl, vu, tl, tu,
               convexesByName, stpbvsByName, collisionTransforms,
               surf, forceSensors, accelBody, springs,
               {}, {}, flexibility);
}

/*void loadPolyTorqueBoundsData(const std::string & file, Robot & robot)
{
}*/

void loadRobotAndEnv(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, Robot & robot, Robot & env)
{
  loadRobotAndEnv(module, surfaceDir, envModule, envSurfaceDir, 0, -1, robot, env);
}

void loadRobotAndEnv(const RobotModule & module, const std::string & surfaceDir, const RobotModule & envModule, const std::string & envSurfaceDir, sva::PTransformd * base, int bId, Robot & robot, Robot & env)
{
  robot = loadRobot(module, surfaceDir, base, bId);
  env = loadRobot(envModule, envSurfaceDir);
}

Robots loadRobots(const std::vector<std::shared_ptr<RobotModule>> & modules, const std::vector<std::string> & surfaceDirs)
{
  std::vector<Robot> res;
  for(size_t i = 0; i < modules.size(); ++i)
  {
    res.push_back(loadRobot(*(modules[i]), surfaceDirs[i]));
  }
  return Robots(res);
}

template<typename X, typename Y>
inline void update(std::map<X,Y> & oldData, const std::map<X,Y> & nData)
{
  for(const auto & p : nData)
  {
    oldData[p.first] = p.second;
  }
}

Robot loadRobotFromUrdf(const std::string & urdf, bool withVirtualLinks, const std::vector<std::string> & filteredLinks, bool fixed, sva::PTransformd * base, int bId)
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

  return Robot(std::make_shared<rbd::MultiBody>(res.mb),
               std::make_shared<rbd::MultiBodyConfig>(res.mbc),
               std::make_shared<rbd::MultiBodyGraph>(res.mbg),
               bodyTransforms, ql, qu, vl, vu, tl, tu,
               {}, {}, res.collision_tf, {}, {});
}

#pragma GCC diagnostic pop

}
