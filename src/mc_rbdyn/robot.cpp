#include <mc_rbdyn/robot.h>

#include <mc_rbdyn/RobotModule.h>
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

Robot::Robot(const rbd::MultiBody & mb, const rbd::MultiBodyConfig & mbc, const rbd::MultiBodyGraph & mbg,
        const std::map<int, sva::PTransformd> & bodyTransforms,
        const std::vector< std::vector<double> > & ql, const std::vector< std::vector<double> > & qu,
        const std::vector< std::vector<double> > & vl, const std::vector< std::vector<double> > & vu,
        const std::vector< std::vector<double> > & tl, const std::vector< std::vector<double> > & tu,
        const std::map<std::string, std::pair< unsigned int, std::shared_ptr<sch::S_Polyhedron> > > & convex,
        const std::map<std::string, std::pair< unsigned int, std::shared_ptr<sch::STP_BV> > > & stpbv,
        const std::map<int, sva::PTransformd> & collisionTransforms, const std::map<std::string, std::shared_ptr<mc_rbdyn::Surface> > & surfaces,
        const std::vector<ForceSensor> & forceSensors, const std::string & accelerometerBody,
        const Springs & springs, const std::vector< std::vector<Eigen::VectorXd> > & tlPoly,
        const std::vector< std::vector<Eigen::VectorXd> > & tuPoly, const std::vector<Flexibility> & flexibility)
: mb(new rbd::MultiBody(mb)), mbc(new rbd::MultiBodyConfig(mbc)), mbg(new rbd::MultiBodyGraph(mbg)),
  bodyTransforms(bodyTransforms), ql(ql), qu(qu), vl(vl), vu(vu), tl(tl), tu(tu),
  convex(convex), stpbv(stpbv), collisionTransforms(collisionTransforms), surfaces(),
  forceSensors(forceSensors), accelerometerBody(accelerometerBody), springs(springs), tlPoly(tlPoly),
  tuPoly(tuPoly), flexibility(flexibility)
{
  // Copy the surfaces
  for(const auto & p : surfaces)
  {
    this->surfaces[p.first] = p.second->copy();
  }
  for(size_t i = 0; i < this->mb->joints().size(); ++i)
  {
    jointIndexByNameD[this->mb->joints()[i].name()] = i;
  }
  for(size_t i = 0; i < this->mb->bodies().size(); ++i)
  {
    bodyIndexByNameD[this->mb->bodies()[i].name()] = i;
  }
  for(const ForceSensor & sensor : forceSensors)
  {
    forceSensorsParentD[sensor.sensorName] = sensor;
    parentBodyForceSensorD[sensor.parentBodyName] = sensor.sensorName;
  }
  if(this->accelerometerBody == "" and this->hasBody("Accelerometer"))
  {
    unsigned int index = bodyIndexByName("Accelerometer");
    this->accelerometerBody = this->mb->body(this->mb->parent(index)).name();
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
  return mb->joint(jointIndexByNameD.at(name)).id();
}

unsigned int Robot::bodyIdByName(const std::string & name) const
{
  return mb->body(bodyIndexByNameD.at(name)).id();
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

Robots::Robots(const std::vector<mc_rbdyn::Robot> & robots, int robotIndex, int envIndex)
: robots(robots), mbs(0), mbcs(0), robotIndex(0), envIndex(0)
{
  if(robotIndex >= 0) { this->robotIndex = static_cast<unsigned int>(robotIndex); }
  if(envIndex >= 0) { this->envIndex = static_cast<unsigned int>(envIndex); }
  for(size_t i = 0; i < robots.size(); ++i)
  {
    mbs.push_back(*(this->robots[i].mb));
    mbcs.push_back(*(this->robots[i].mbc));
    if(robotIndex < 0 and this->robots[i].mb->nrDof()) { this->robotIndex = i; robotIndex = 0; }
    if(envIndex < 0 and this->robots[i].mb->nrDof() == 0) { this->envIndex = i; envIndex = 0; }
    /*FIXME Hackish to keep things coherent in Robots between the Robot vector and the mb/mbc vectors */
    delete this->robots[i].mb;
    delete this->robots[i].mbc;
  }
  /* Should be done here, because vector adresses can change */
  for(size_t i = 0; i < this->robots.size(); ++i)
  {
    this->robots[i].mb = &(mbs[i]);
    this->robots[i].mbc = &(mbcs[i]);
  }
}

Robots::Robots(const Robots & rhs)
: robots(rhs.robots), mbs(rhs.mbs), mbcs(rhs.mbcs), robotIndex(rhs.robotIndex), envIndex(rhs.envIndex)
{
  for(size_t i = 0; i < robots.size(); ++i)
  {
    this->robots[i].mb = &(mbs[i]);
    this->robots[i].mbc = &(mbcs[i]);
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
    this->robots[i].mb = &(mbs[i]);
    this->robots[i].mbc = &(mbcs[i]);
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

void fixRobotSurfaces(Robot & robot)
{
  for(std::pair<const std::string, std::shared_ptr<Surface> > & s : robot.surfaces)
  {
    unsigned int bodyId = robot.bodyIdByName(s.second->bodyName);
    const sva::PTransformd & trans = robot.bodyTransforms[bodyId];
    s.second->X_b_s(s.second->X_b_s()*trans);
  }
}

Robot createRobotWithBase(Robot & robot, const Base & base, const Eigen::Vector3d & baseAxis)
{
  rbd::MultiBody mb = robot.mbg->makeMultiBody(base.baseId, base.baseType, baseAxis, base.X_0_s, base.X_b0_s);
  rbd::MultiBodyConfig mbc = rbd::MultiBodyConfig(mb);
  mbc.zero(mb);
  std::map<int, sva::PTransformd> bodyTransforms = robot.mbg->bodiesBaseTransform(base.baseId, base.X_b0_s);

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

  int jParam = mb.joint(0).params();
  int jDof = mb.joint(0).dof();
  bound_t ql = convertBound(*(robot.mb), mb, robot.ql, std::vector<double>(jParam, -INFINITY));
  bound_t qu = convertBound(*(robot.mb), mb, robot.qu, std::vector<double>(jParam, INFINITY));
  bound_t vl = convertBound(*(robot.mb), mb, robot.vl, std::vector<double>(jDof, -INFINITY));
  bound_t vu = convertBound(*(robot.mb), mb, robot.vu, std::vector<double>(jDof, INFINITY));
  bound_t tl = convertBound(*(robot.mb), mb, robot.tl, std::vector<double>(jDof, -INFINITY));
  bound_t tu = convertBound(*(robot.mb), mb, robot.tu, std::vector<double>(jDof, INFINITY));

  Robot ret(mb, mbc, *(robot.mbg), bodyTransforms,
              ql, qu, vl, vu, tl, tu,
              robot.convex, robot.stpbv, robot.collisionTransforms,
              robot.surfaces, robot.forceSensors, robot.accelerometerBody,
              robot.springs, robot.tlPoly, robot.tuPoly,
              robot.flexibility);
  fixRobotSurfaces(ret);
  return ret;
}

Robot robotCopy(const Robot & robot)
{
  return Robot(rbd::MultiBody(*(robot.mb)), rbd::MultiBodyConfig(*(robot.mbc)), *(robot.mbg), robot.bodyTransforms, robot.ql, robot.qu, robot.vl, robot.vu, robot.tl, robot.tu, robot.convex, robot.stpbv, robot.collisionTransforms, robot.surfaces, robot.forceSensors, robot.accelerometerBody, robot.springs, robot.tlPoly, robot.tuPoly, robot.flexibility);
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

  std::map<std::string, std::pair<unsigned int, std::shared_ptr<sch::S_Polyhedron> > > convexByName;
  {
    for(const auto & p : module.convexHull())
    {
      if(bodyIdByName.count(p.second.first))
      {
        std::shared_ptr<sch::S_Polyhedron> poly(new sch::S_Polyhedron);
        poly->constructFromFile(p.second.second);
        convexByName[p.first] = std::pair<unsigned int, std::shared_ptr<sch::S_Polyhedron> > (bodyIdByName[p.second.first], poly);
      }
    }
    applyTransformToSchById(mb, mbc, convexByName);
  }

  std::map<std::string, std::pair<unsigned int, std::shared_ptr<sch::STP_BV> > > stpbvByName;
  {
    for(const auto & p : module.stpbvHull())
    {
      if(bodyIdByName.count(p.second.first))
      {
        std::shared_ptr<sch::STP_BV> stpbv(new sch::STP_BV);
        stpbv->constructFromFile(p.second.second);
        stpbvByName[p.first] = std::pair<unsigned int, std::shared_ptr<sch::STP_BV> > (bodyIdByName[p.second.first], stpbv);
      }
    }
    applyTransformToSchById(mb, mbc, stpbvByName);
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

  std::vector< std::shared_ptr<Surface> > surfaces = readRSDFFromDir(surfaceDir);
  std::map<std::string, std::shared_ptr<Surface> > surf;
  for(const auto & sp : surfaces)
  {
    surf[sp->name] = sp;
  }

  const std::vector<Flexibility> & flexibility = module.flexibility();

  const std::vector<ForceSensor> & forceSensors = module.forceSensors();

  const std::string & accelBody = module.accelerometerBody();

  const Springs & springs = module.springs();

  return Robot(mb, mbc, mbg, bodyTransforms, ql, qu, vl, vu, tl, tu,
               convexByName, stpbvByName, collisionTransforms,
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

Robots loadRobots(const std::vector<RobotModule> & modules, const std::vector<std::string> & surfaceDirs)
{
  std::vector<Robot> res;
  for(size_t i = 0; i < modules.size(); ++i)
  {
    res.push_back(loadRobot(modules[i], surfaceDirs[i]));
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

  return Robot(res.mb, res.mbc, res.mbg, bodyTransforms, ql, qu, vl, vu, tl, tu, {}, {}, res.collision_tf, {}, {});
}

#pragma GCC diagnostic pop

}
