#include <mc_rbdyn/Robot.h>

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rbdyn/surface_utils.h>

#include <mc_rbdyn_urdf/urdf.h>

#include <mc_rtc/logging.h>

namespace mc_rbdyn
{

// We can safely ignore those since they are due to different index types and
// our index never go near unsafe territories
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#ifdef __clang__
#pragma clang diagnostic ignored "-Wshorten-64-to-32"
#endif

Robot::Robot(const std::string & name, Robots & robots, unsigned int robots_idx,
        const std::map<std::string, sva::PTransformd> & bodyTransforms,
        const std::vector< std::vector<double> > & ql, const std::vector< std::vector<double> > & qu,
        const std::vector< std::vector<double> > & vl, const std::vector< std::vector<double> > & vu,
        const std::vector< std::vector<double> > & tl, const std::vector< std::vector<double> > & tu,
        const std::map<std::string, convex_pair_t> & convexes,
        const std::map<std::string, stpbv_pair_t> & stpbvs,
        const std::map<std::string, sva::PTransformd> & collisionTransforms, const std::map<std::string, std::shared_ptr<mc_rbdyn::Surface> > & surfaces,
        const std::vector<ForceSensor> & forceSensors,
        const std::vector<std::string> & refJointOrder,
        const std::map<std::string, std::vector<double>> stance,
        const BodySensorVector & bodySensors,
        const Springs & springs, const std::vector< std::vector<Eigen::VectorXd> > & tlPoly,
        const std::vector< std::vector<Eigen::VectorXd> > & tuPoly, const std::vector<Flexibility> & flexibility)
: name_(name), robots(&robots), robots_idx(robots_idx),
  bodyTransforms(bodyTransforms), ql_(ql), qu_(qu), vl_(vl), vu_(vu), tl_(tl), tu_(tu),
  convexes(convexes), stpbvs(stpbvs), collisionTransforms(collisionTransforms), surfaces_(),
  forceSensors_(forceSensors), stance_(stance), refJointOrder_(refJointOrder),
  bodySensors_(bodySensors), springs(springs), tlPoly(tlPoly),
  tuPoly(tuPoly), flexibility_(flexibility)
{
  //check whether bounds are valid
  auto throwIfBoundsInvalid = [&](
    const std::vector< std::vector<double> > & l,
    const std::vector< std::vector<double> > & u,
    const std::string& bname,
    int(rbd::Joint::*JointMethod)()const)
  {
    std::size_t expectedSize = mb().joints().size();
    if(l.size() != expectedSize)
    {
      LOG_ERROR_AND_THROW(std::invalid_argument,
        "Robot '" << name << "' has invalid " << bname
        << " bounds. The bound vector for lower bounds has the wrong size! number of entries in lower = "
        << l.size() << ", number of joints = " << expectedSize);
    }

    if(u.size() != expectedSize)
    {
        LOG_ERROR_AND_THROW(std::invalid_argument,
          "Robot '" << name << "' has invalid " << bname
          << " bounds. The bound vector for upper bounds has the wrong size! number of entries in upper = "
          << u.size() << ", number of joints = " << expectedSize);
    }

    for(int i = 0; i < static_cast<int>(l.size()); ++ i)
    {
      const rbd::Joint& joint = mb().joint(i);
      std::size_t expectedSize = static_cast<std::size_t>((joint.*JointMethod)());
      const auto& subL = l.at(i);
      const auto& subU = u.at(i);
      if(subL.size() != expectedSize)
      {
        LOG_ERROR_AND_THROW(std::invalid_argument,
          "Robot '" << name << "' has invalid " << bname << " bounds. The lower bound vector for "
          << "joint " << i << " '" << joint.name() << "' has the wrong size! number of entries in lower = "
          << subL.size() << ", number of entries expected = " << expectedSize);
      }
      if(subU.size() != expectedSize)
      {
        LOG_ERROR_AND_THROW(std::invalid_argument,
          "Robot '" << name << "' has invalid " << bname << " bounds. The upper bound vector for "
          << "joint " << i << " '" << joint.name() << "' has the wrong size! number of entries in upper = "
          << subU.size() << ", number of entries expected = " << expectedSize);
      }

      for(std::size_t j = 0; j < subL.size(); ++ j)
      {
          if(subL.at(j) > subU.at(j))
          {
            LOG_ERROR_AND_THROW(std::invalid_argument,
              "Robot '" << name << "' has invalid " << bname << " bounds. The lower bound for "
              << " joint " << i << " '" << joint.name()
              << "' / " << j << " is not lower than or equal the upper bound! lower = "
              << subL.at(j) << ", upper = " << subU.at(j));
          }
      }
    }
  };
  throwIfBoundsInvalid(ql_, qu_, "position", &rbd::Joint::params);
  throwIfBoundsInvalid(vl_, vu_, "velocity", &rbd::Joint::dof);
  throwIfBoundsInvalid(tl_, tu_, "torque", &rbd::Joint::dof);
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
  /* Add a single default sensor if no sensor is available */
  if(bodySensors_.size() == 0)
  {
    bodySensors_.emplace_back();
  }
  for(size_t i = 0; i < forceSensors_.size(); ++i)
  {
    const auto & fs = forceSensors_[i];
    forceSensorsIndex_[fs.name()] = i;
    bodyForceSensors_[fs.parentBody()] = i;
  }
}

std::string Robot::name() const
{
  return name_;
}

void Robot::name(const std::string & name)
{
  name_ = name;
}

BodySensor & Robot::bodySensor()
{
  return bodySensors_[0];
}

const BodySensor & Robot::bodySensor() const
{
  return bodySensors_[0];
}

bool Robot::hasBodySensor(const std::string & name) const
{
  return bodySensorsIndex_.count(name) != 0;
}

bool Robot::bodyHasBodySensor(const std::string & body) const
{
  return bodyBodySensors_.count(body) != 0;
}

BodySensor & Robot::bodySensor(const std::string & name)
{
  return const_cast<BodySensor&>(static_cast<const Robot*>(this)->bodySensor(name));
}

const BodySensor & Robot::bodySensor(const std::string & name) const
{
  return bodySensors_[bodySensorsIndex_.at(name)];
}

BodySensor & Robot::bodyBodySensor(const std::string & body)
{
  return const_cast<BodySensor&>(static_cast<const Robot*>(this)->bodyBodySensor(body));
}

const BodySensor & Robot::bodyBodySensor(const std::string & body) const
{
  return bodySensors_[bodyBodySensors_.at(body)];
}

BodySensorVector & Robot::bodySensors()
{
  return bodySensors_;
}

const BodySensorVector & Robot::bodySensors() const
{
  return bodySensors_;
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

rbd::MultiBody & Robot::mb()
{
  return robots->mbs_[robots_idx];
}
const rbd::MultiBody & Robot::mb() const
{
  return robots->mbs_[robots_idx];
}

rbd::MultiBodyConfig & Robot::mbc()
{
  return robots->mbcs_[robots_idx];
}
const rbd::MultiBodyConfig & Robot::mbc() const
{
  return robots->mbcs_[robots_idx];
}

rbd::MultiBodyGraph & Robot::mbg()
{
  return robots->mbgs_[robots_idx];
}
const rbd::MultiBodyGraph & Robot::mbg() const
{
  return robots->mbgs_[robots_idx];
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
std::vector<std::vector<double>> & Robot::ql()
{
  return ql_;
}
std::vector<std::vector<double>> & Robot::qu()
{
  return qu_;
}
std::vector<std::vector<double>> & Robot::vl()
{
  return vl_;
}
std::vector<std::vector<double>> & Robot::vu()
{
  return vu_;
}
std::vector<std::vector<double>> & Robot::tl()
{
  return tl_;
}
std::vector<std::vector<double>> & Robot::tu()
{
  return tu_;
}

const std::vector<Flexibility> & Robot::flexibility() const
{
  return flexibility_;
}

std::vector<Flexibility> & Robot::flexibility()
{
  return flexibility_;
}

const std::vector<double> & Robot::encoderValues() const
{
  return encoderValues_;
}

void Robot::encoderValues(const std::vector<double> & encoderValues)
{
  encoderValues_ = encoderValues;
}

const std::vector<double> & Robot::jointTorques() const
{
  return jointTorques_;
}

void Robot::jointTorques(const std::vector<double> & jointTorques)
{
  jointTorques_ = jointTorques;
}

const std::vector<std::string> & Robot::refJointOrder() const
{
  return refJointOrder_;
}

bool Robot::hasForceSensor(const std::string & name) const
{
  return forceSensorsIndex_.count(name) != 0;
}

bool Robot::bodyHasForceSensor(const std::string & body) const
{
  return bodyForceSensors_.count(body) != 0;
}

ForceSensor & Robot::forceSensor(const std::string & name)
{
  return const_cast<ForceSensor&>(static_cast<const Robot*>(this)->forceSensor(name));
}

const ForceSensor & Robot::forceSensor(const std::string & name) const
{
  return forceSensors_[forceSensorsIndex_.at(name)];
}

ForceSensor & Robot::bodyForceSensor(const std::string & body)
{
  return const_cast<ForceSensor&>(static_cast<const Robot*>(this)->bodyForceSensor(body));
}

const ForceSensor & Robot::bodyForceSensor(const std::string & body) const
{
  return forceSensors_.at(bodyForceSensors_.at(body));
}

bool Robot::hasSurface(const std::string & surface) const
{
  return surfaces_.count(surface) != 0;
}

std::vector<ForceSensor> & Robot::forceSensors()
{
  return forceSensors_;
}

const std::vector<ForceSensor> & Robot::forceSensors() const
{
  return forceSensors_;
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

std::vector<std::string> Robot::availableSurfaces() const
{
  std::vector<std::string> ret;
  ret.reserve(surfaces_.size());
  for(const auto & s : surfaces_)
  {
    ret.push_back(s.first);
  }
  return ret;
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

const sva::PTransformd & Robot::bodyTransform(const std::string& bName) const
{
  if(bodyTransforms.count(bName) == 0)
  {
    LOG_ERROR("No body transform with name " << bName << " found in this robot")
    throw("Body transform does not exist");
  }
  return bodyTransforms.at(bName);
}

const sva::PTransformd & Robot::collisionTransform(const std::string& cName) const
{
  if(collisionTransforms.count(cName) == 0)
  {
    LOG_ERROR("No collision transform with name " << cName << " found in this robot")
    throw("Collision transform does not exist");
  }
  return collisionTransforms.at(cName);
}

void Robot::fixSurfaces()
{
  for(auto & s : surfaces_)
  {
    const sva::PTransformd & trans = bodyTransforms[s.second->bodyName()];
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

std::map<std::string, std::vector<double>> Robot::stance() const
{
  return stance_;
}

unsigned int mc_rbdyn::Robot::robotIndex() const
{
  return robots_idx;
}

void Robot::createWithBase(Robots & robots, unsigned int robots_idx, const Base & base) const
{
  rbd::MultiBody & mb = robots.mbs_[robots_idx];
  rbd::MultiBodyConfig & mbc = robots.mbcs_[robots_idx];
  rbd::MultiBodyGraph & mbg = robots.mbgs_[robots_idx];
  mbc.zero(mb);
  std::map<std::string, sva::PTransformd> bodyTransforms = mbg.bodiesBaseTransform(base.baseName, base.X_b0_s);

  typedef std::vector< std::vector<double> > bound_t;
  auto convertBound = [](const rbd::MultiBody & oldMb, const rbd::MultiBody & newMb, const bound_t & oldBound, const std::vector<double> & baseBound)
  {
    bound_t newBound;
    newBound.resize(oldBound.size());
    newBound[0] = baseBound;
    for(size_t i = 1; i < oldBound.size(); ++i)
    {
      newBound[static_cast<size_t>(newMb.jointIndexByName(oldMb.joint(static_cast<int>(i)).name()))] = oldBound[i];
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
              this->surfaces_, this->forceSensors_,
              this->refJointOrder_, this->stance_,
              this->bodySensors_, this->springs,
              this->tlPoly, this->tuPoly,
              this->flexibility());
  robots.robot(robots_idx).fixSurfaces();
}

void Robot::copy(Robots & robots, unsigned int robots_idx) const
{
  robots.robots_.emplace_back(this->name_, robots, robots_idx, this->bodyTransforms, this->ql(), this->qu(), this->vl(), this->vu(), this->tl(), this->tu(), this->convexes, this->stpbvs, this->collisionTransforms, this->surfaces_, this->forceSensors_, this->refJointOrder_, this->stance_, this->bodySensors_, this->springs, this->tlPoly, this->tuPoly, this->flexibility());
}

mc_rbdyn::Surface & Robot::copySurface(const std::string & sName, const std::string & name)
{
  if(hasSurface(name))
  {
    LOG_ERROR(name << " already exists within this robot. Cannot overwrite an existing surface")
    throw("Target surface already exists");
  }
  const Surface & surf = surface(sName);
  SurfacePtr nSurf = surf.copy();
  nSurf->name(name);
  surfaces_[name] = nSurf;
  return *nSurf;
}

void mc_rbdyn::Robot::addSurface(SurfacePtr surface, bool doNotReplace)
{
    if(!hasBody(surface->bodyName()))
    {
      LOG_WARNING("Surface " << surface->name() << " attached to body " << surface->bodyName() << " but the robot " << name() << " has no such body.")
      return;
    }
    if(hasSurface(surface->name()) && doNotReplace)
    {
      LOG_WARNING("Surface " << surface->name() << " already exists for the robot " << name() << ".")
      return;
    }
    surfaces_[surface->name()] = std::move(surface);
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

std::map<std::string, std::vector<double> > jointsVectorToName(const rbd::MultiBody & mb, const std::vector< std::vector<double> > & jointsVec,
                                                              const std::function<bool(const rbd::Joint &, const std::vector<double> &)> & filter)
{
  std::map<std::string, std::vector<double> > res;
  for(size_t i = 0; i < std::min(jointsVec.size(), mb.joints().size()); ++i)
  {
    if(filter(mb.joints()[i], jointsVec[i]))
    {
      res[mb.joints()[i].name()] = jointsVec[i];
    }
  }
  return res;
}

std::vector< std::vector<double> > jointsNameToVector(const rbd::MultiBody & mb, std::map<std::string, std::vector<double> > & jointsName, const std::vector<double> & def, const std::function<bool (const rbd::Joint &)> & filter)
{
  std::vector< std::vector<double> > res;
  for(const rbd::Joint & j : mb.joints())
  {
    if(filter(j))
    {
      // Mimic python setdefault behaviour
      if(jointsName.count(j.name()) == 0)
      {
        jointsName[j.name()] = def;
      }
      res.push_back(jointsName[j.name()]);
    }
  }
  return res;
}

// Return [ql, qu, vl, vu, tl, tu]
std::vector< std::map< std::string, std::vector<double> > > defaultBounds(const rbd::MultiBody & mb)
{
  std::vector< std::map< std::string, std::vector<double> > > res;
  auto jParam = [mb](const double & coeff) { return jointsVectorToName(mb, jointsParameters(mb, coeff)); };
  auto jDof = [mb](const double & coeff) { return jointsVectorToName(mb, jointsDof(mb, coeff)); };
  res.push_back(jParam(-INFINITY));
  res.push_back(jParam(INFINITY));
  res.push_back(jDof(-INFINITY));
  res.push_back(jDof(INFINITY));
  res.push_back(jDof(-0));
  res.push_back(jDof(0));
  return res;
}

#pragma GCC diagnostic pop

}
