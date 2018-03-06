#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rbdyn/surface_utils.h>
#include <mc_rtc/logging.h>

#include <mc_rbdyn_urdf/urdf.h>

#include <RBDyn/CoM.h>
#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <tuple>

namespace
{

using bound_t = std::vector<std::vector<double>>;
using bounds_t = std::tuple<bound_t, bound_t, bound_t, bound_t, bound_t, bound_t>;
using rm_bounds_t = mc_rbdyn::RobotModule::bounds_t;
using rm_bound_t = rm_bounds_t::value_type;

/** Generate bounds compatible with the given MultiBody
 *
 * If bounds is provided, use the values provided to build the bounds.
 *
 * Otherwise, default bounds are returned.
 */
bounds_t bounds(const rbd::MultiBody & mb, const rm_bounds_t & bounds)
{
  using jt_method = int (rbd::Joint::*)() const;
  auto fill_bound = [&mb](const std::string & name, const rm_bound_t & bound_in, jt_method def_size, double def_value,
                          double ff_def_value) {
    bound_t res;
    res.reserve(mb.nrJoints());
    for(const auto & j : mb.joints())
    {
      res.emplace_back(((j).*(def_size))(), j.type() == rbd::Joint::Free ? ff_def_value : def_value);
      if(bound_in.count(j.name()))
      {
        const auto & b_ref = bound_in.at(j.name());
        auto & b = res.back();
        if(b_ref.size() != b.size())
        {
          LOG_ERROR_AND_THROW(std::runtime_error, name << " provided bound size (" << b_ref.size() << ")"
                                                       << " different from expected size (" << b.size() << ")")
        }
        res.back() = bound_in.at(j.name());
      }
    }
    return res;
  };
  return std::make_tuple(fill_bound("lower position", bounds.at(0), &rbd::Joint::params, -INFINITY, -INFINITY),
                         fill_bound("upper position", bounds.at(1), &rbd::Joint::params, INFINITY, INFINITY),
                         fill_bound("lower velocity", bounds.at(2), &rbd::Joint::dof, -INFINITY, -INFINITY),
                         fill_bound("upper velocity", bounds.at(3), &rbd::Joint::dof, INFINITY, INFINITY),
                         fill_bound("lower torque", bounds.at(4), &rbd::Joint::dof, -INFINITY, 0),
                         fill_bound("upper torque", bounds.at(5), &rbd::Joint::dof, INFINITY, 0));
}

template<typename schT, typename mapT>
void loadSCH(const mc_rbdyn::Robot & robot,
             const std::map<std::string, std::pair<std::string, std::string>> & urls,
             schT * (*sch_load_fn)(const std::string &),
             mapT & data_)
{
  for(const auto & cH : urls)
  {
    const std::string & cHName = cH.first;
    const std::string & parent = cH.second.first;
    const std::string & cHURL = cH.second.second;
    if(robot.hasBody(parent))
    {
      auto poly = std::shared_ptr<schT>(sch_load_fn(cHURL));
      sch::mc_rbdyn::transform(*poly, robot.bodyPosW()[robot.bodyIndexByName(parent)]);
      data_[cHName] = {parent, poly};
    }
  }
}

template<typename mapT>
void fixSCH(const mc_rbdyn::Robot & robot, mapT & data_)
{
  for(const auto & d : data_)
  {
    sch::mc_rbdyn::transform(*d.second.second, robot.bodyPosW()[robot.bodyIndexByName(d.second.first)]);
  }
}

} // namespace

namespace mc_rbdyn
{

// We can safely ignore those since they are due to different index types and
// our index never go near unsafe territories
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#ifdef __clang__
#  pragma clang diagnostic ignored "-Wshorten-64-to-32"
#endif

Robot::Robot(Robots & robots,
             unsigned int robots_idx,
             bool loadFiles,
             const sva::PTransformd * base,
             const std::string & bName)
: robots_(&robots), robots_idx_(robots_idx)
{
  const auto & module_ = module();

  if(base)
  {
    std::string baseName = bName.empty() ? mb().body(0).name() : bName;
    mb() = mbg().makeMultiBody(baseName, mb().joint(0).type() == rbd::Joint::Fixed, *base);
    mbc() = rbd::MultiBodyConfig(mb());
  }

  mbc().gravity = Eigen::Vector3d{0, 0, 9.81};
  mbc().zero(mb());
  {
    auto initQ = mbc().q;
    const auto & stance = module_.stance();
    for(size_t i = 0; i < mb().joints().size(); ++i)
    {
      const auto & j = mb().joint(static_cast<int>(i));
      if(stance.count(j.name()))
      {
        const auto & jQ = stance.at(j.name());
        if(initQ[i].size() != jQ.size())
        {
          LOG_ERROR_AND_THROW(std::runtime_error, "Missmatch between RobotModule stance for joint "
                                                      << j.name() << std::endl
                                                      << "Stance provides " << jQ.size() << " values but should be "
                                                      << initQ[i].size())
        }
        initQ[i] = jQ;
      }
    }
    if(initQ[0].size())
    {
      const auto & attitude = module_.default_attitude();
      initQ[0] = {std::begin(attitude), std::end(attitude)};
    }
    mbc().q = initQ;
    forwardKinematics();
  }

  name_ = module_.name;

  bodyTransforms_.resize(mb().bodies().size());
  const auto & bbts =
      base ? mbg().bodiesBaseTransform(mb().body(0).name(), *base) : mbg().bodiesBaseTransform(mb().body(0).name());
  for(size_t i = 0; i < mb().bodies().size(); ++i)
  {
    const auto & b = mb().body(static_cast<int>(i));
    bodyTransforms_[i] = bbts.at(b.name());
  }

  if(module_.bounds().size() != 6)
  {
    LOG_ERROR_AND_THROW(std::invalid_argument, "The bounds of robotmodule '"
                                                   << module_.name << "' have a size of " << module_.bounds().size()
                                                   << " instead of 6 (ql, qu, vl, vu, tl, tu).")
  }
  std::tie(ql_, qu_, vl_, vu_, tl_, tu_) = bounds(mb(), module_.bounds());

  if(loadFiles)
  {
    loadSCH(*this, module_.convexHull(), &sch::mc_rbdyn::Polyhedron, convexes_);
    loadSCH(*this, module_.stpbvHull(), &sch::mc_rbdyn::STPBV, stpbvs_);
  }

  for(const auto & b : mb().bodies())
  {
    collisionTransforms_[b.name()] = sva::PTransformd::Identity();
  }
  for(const auto & p : module_.collisionTransforms())
  {
    collisionTransforms_[p.first] = p.second;
  }
  fixCollisionTransforms();

  if(loadFiles)
  {
    if(bfs::exists(module_.rsdf_dir))
    {
      loadRSDFFromDir(module_.rsdf_dir);
    }
    else if(module_.rsdf_dir.size())
    {
      LOG_ERROR("RSDF directory (" << module_.rsdf_dir << ") specified by RobotModule for " << module_.name
                                   << " does not exist")
    }
  }

  forceSensors_ = module_.forceSensors();
  for(auto & fs : forceSensors_)
  {
    bfs::path calib_file = bfs::path(module_.calib_dir) / std::string("calib_data." + fs.name());
    if(bfs::exists(calib_file))
    {
      fs.loadCalibrator(calib_file.string(), mbc().gravity);
    }
  }
  for(size_t i = 0; i < forceSensors_.size(); ++i)
  {
    const auto & fs = forceSensors_[i];
    forceSensorsIndex_[fs.name()] = i;
    bodyForceSensors_[fs.parentBody()] = i;
  }

  stance_ = module_.stance();

  bodySensors_ = module_.bodySensors();
  // Add a single default sensor if no sensor on the robot
  if(bodySensors_.size() == 0)
  {
    bodySensors_.emplace_back();
  }
  for(size_t i = 0; i < bodySensors_.size(); ++i)
  {
    const auto & bS = bodySensors_[i];
    bodySensorsIndex_[bS.name()] = i;
    bodyBodySensors_[bS.parentBody()] = i;
  }

  refJointOrder_ = module_.ref_joint_order();

  springs_ = module_.springs();
  flexibility_ = module_.flexibility();

  zmp_ = Eigen::Vector3d::Zero();
}

std::string Robot::name() const
{
  return name_;
}

void Robot::name(const std::string & name)
{
  name_ = name;
}

const RobotModule & Robot::module() const
{
  return robots_->robotModule(robots_idx_);
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
  return const_cast<BodySensor &>(static_cast<const Robot *>(this)->bodySensor(name));
}

const BodySensor & Robot::bodySensor(const std::string & name) const
{
  return bodySensors_[bodySensorsIndex_.at(name)];
}

BodySensor & Robot::bodyBodySensor(const std::string & body)
{
  return const_cast<BodySensor &>(static_cast<const Robot *>(this)->bodyBodySensor(body));
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
  return mb().jointIndexByName().count(name) != 0;
}

bool Robot::hasBody(const std::string & name) const
{
  return mb().bodyIndexByName().count(name) != 0;
}

unsigned int Robot::jointIndexByName(const std::string & name) const
{
  return mb().jointIndexByName().at(name);
}

unsigned int Robot::bodyIndexByName(const std::string & name) const
{
  return mb().bodyIndexByName().at(name);
}

rbd::MultiBody & Robot::mb()
{
  return robots_->mbs_[robots_idx_];
}
const rbd::MultiBody & Robot::mb() const
{
  return robots_->mbs_[robots_idx_];
}

rbd::MultiBodyConfig & Robot::mbc()
{
  return robots_->mbcs_[robots_idx_];
}
const rbd::MultiBodyConfig & Robot::mbc() const
{
  return robots_->mbcs_[robots_idx_];
}

rbd::MultiBodyGraph & Robot::mbg()
{
  return robots_->mbgs_[robots_idx_];
}
const rbd::MultiBodyGraph & Robot::mbg() const
{
  return robots_->mbgs_[robots_idx_];
}

const std::vector<std::vector<double>> & Robot::q() const
{
  return mbc().q;
}
const std::vector<std::vector<double>> & Robot::alpha() const
{
  return mbc().alpha;
}
const std::vector<std::vector<double>> & Robot::alphaD() const
{
  return mbc().alphaD;
}
const std::vector<std::vector<double>> & Robot::jointTorque() const
{
  return mbc().jointTorque;
}
const std::vector<sva::PTransformd> & Robot::bodyPosW() const
{
  return mbc().bodyPosW;
}
const std::vector<sva::MotionVecd> & Robot::bodyVelW() const
{
  return mbc().bodyVelW;
}
const std::vector<sva::MotionVecd> & Robot::bodyVelB() const
{
  return mbc().bodyVelB;
}
const std::vector<sva::MotionVecd> & Robot::bodyAccB() const
{
  return mbc().bodyAccB;
}
std::vector<std::vector<double>> & Robot::q()
{
  return mbc().q;
}
std::vector<std::vector<double>> & Robot::alpha()
{
  return mbc().alpha;
}
std::vector<std::vector<double>> & Robot::alphaD()
{
  return mbc().alphaD;
}
std::vector<std::vector<double>> & Robot::jointTorque()
{
  return mbc().jointTorque;
}
std::vector<sva::PTransformd> & Robot::bodyPosW()
{
  return mbc().bodyPosW;
}
std::vector<sva::MotionVecd> & Robot::bodyVelW()
{
  return mbc().bodyVelW;
}
std::vector<sva::MotionVecd> & Robot::bodyVelB()
{
  return mbc().bodyVelB;
}
std::vector<sva::MotionVecd> & Robot::bodyAccB()
{
  return mbc().bodyAccB;
}

const sva::PTransformd & Robot::bodyPosW(const std::string & name) const
{
  return bodyPosW()[bodyIndexByName(name)];
}

const sva::MotionVecd & Robot::bodyVelW(const std::string & name) const
{
  return bodyVelW()[bodyIndexByName(name)];
}

const sva::MotionVecd & Robot::bodyVelB(const std::string & name) const
{
  return bodyVelB()[bodyIndexByName(name)];
}

const sva::MotionVecd & Robot::bodyAccB(const std::string & name) const
{
  return bodyAccB()[bodyIndexByName(name)];
}

Eigen::Vector3d Robot::com() const
{
  return rbd::computeCoM(mb(), mbc());
}
Eigen::Vector3d Robot::comVelocity() const
{
  return rbd::computeCoMVelocity(mb(), mbc());
}
Eigen::Vector3d Robot::comAcceleration() const
{
  return rbd::computeCoMAcceleration(mb(), mbc());
}

sva::ForceVecd Robot::surfaceWrench(const std::string & surfaceName) const
{
  const auto & bodyName = surface(surfaceName).bodyName();
  const auto & fs = bodyForceSensor(bodyName);
  sva::ForceVecd w_fsactual = fs.wrenchWithoutGravity(*this);
  sva::PTransformd X_fsactual_surf = surface(surfaceName).X_b_s() * fs.X_fsactual_parent();
  return X_fsactual_surf.dualMul(w_fsactual);
}

Eigen::Vector2d Robot::cop(const std::string & surfaceName, double min_pressure) const
{
  const sva::ForceVecd w_surf = surfaceWrench(surfaceName);
  const double pressure = w_surf.force()(2);
  if(pressure < min_pressure)
  {
    return Eigen::Vector2d::Zero();
  }
  const Eigen::Vector3d & tau_surf = w_surf.couple();
  return Eigen::Vector2d(-tau_surf(1) / pressure, +tau_surf(0) / pressure);
}

Eigen::Vector3d Robot::copW(const std::string & surfaceName, double min_pressure) const
{
  Eigen::Vector3d cop_s;
  cop_s << cop(surfaceName, min_pressure), 0.;
  const sva::PTransformd X_0_s = surface(surfaceName).X_0_s(*this);
  return X_0_s.translation() + X_0_s.rotation().inverse() * cop_s;
}

Eigen::Vector3d Robot::zmp(const std::vector<std::string> & sensorsName,
                           const Eigen::Vector3d & plane_p,
                           const Eigen::Vector3d & plane_n,
                           double forceThreshold) const
{
  sva::ForceVecd measuredWrench{Eigen::Vector6d::Zero()};
  for(const auto & sensorName : sensorsName)
  {
    const auto & sensor = forceSensor(sensorName);
    if(sensor.force().norm() > forceThreshold)
    {
      measuredWrench += sensor.worldWrenchWithoutGravity(*this);
    }
  }

  const Eigen::Vector3d & force = measuredWrench.force();
  const Eigen::Vector3d & moment_0 = measuredWrench.couple();

  Eigen::Vector3d moment_p = moment_0 - plane_p.cross(force);
  double floorn_dot_force = plane_n.dot(force);
  // Prevent potential division by zero
  if(floorn_dot_force < 1.)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "ZMP cannot be computed, projected force too small " << floorn_dot_force);
  }
  Eigen::Vector3d zmp = plane_p + plane_n.cross(moment_p) / floorn_dot_force;
  return zmp;
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

const std::vector<double> & Robot::flexibilityValues() const
{
  return flexibilityValues_;
}

void Robot::flexibilityValues(const std::vector<double> & flexibilityValues)
{
  flexibilityValues_ = flexibilityValues;
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
  return const_cast<ForceSensor &>(static_cast<const Robot *>(this)->forceSensor(name));
}

const ForceSensor & Robot::forceSensor(const std::string & name) const
{
  return forceSensors_[forceSensorsIndex_.at(name)];
}

ForceSensor & Robot::bodyForceSensor(const std::string & body)
{
  return const_cast<ForceSensor &>(static_cast<const Robot *>(this)->bodyForceSensor(body));
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
  return const_cast<mc_rbdyn::Surface &>(static_cast<const Robot *>(this)->surface(sName));
}

sva::PTransformd Robot::surfacePose(const std::string & sName) const
{
  return surface(sName).X_0_s(*this);
}

const mc_rbdyn::Surface & Robot::surface(const std::string & sName) const
{
  if(!hasSurface(sName))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "No surface named " << sName << " found in this robot")
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
  return const_cast<Robot::convex_pair_t &>(static_cast<const Robot *>(this)->convex(cName));
}
const Robot::convex_pair_t & Robot::convex(const std::string & cName) const
{
  if(convexes_.count(cName) == 0)
  {
    LOG_ERROR_AND_THROW(std::runtime_error,
                        "No convex named " << cName << " found in this robot (" << this->name_ << ")")
  }
  return convexes_.at(cName);
}

const sva::PTransformd & Robot::bodyTransform(const std::string & bName) const
{
  if(!hasBody(bName))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "No body transform with name " << bName << " found in this robot")
  }
  return bodyTransforms_[bodyIndexByName(bName)];
}

const sva::PTransformd & Robot::bodyTransform(int bodyIndex) const
{
  return bodyTransforms_[bodyIndex];
}

const std::vector<sva::PTransformd> & Robot::bodyTransforms() const
{
  return bodyTransforms_;
}

const sva::PTransformd & Robot::collisionTransform(const std::string & cName) const
{
  if(collisionTransforms_.count(cName) == 0)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "No collision transform with name " << cName << " found in this robot")
  }
  return collisionTransforms_.at(cName);
}

void Robot::fixSurfaces()
{
  for(auto & s : surfaces_)
  {
    const sva::PTransformd & trans = bodyTransform(s.second->bodyName());
    s.second->X_b_s(s.second->X_b_s() * trans);
  }
}

void Robot::fixCollisionTransforms()
{
  for(auto & ct : collisionTransforms_)
  {
    // assert(bodyTransforms_.size(ct.first));
    const auto & trans = bodyTransform(ct.first);
    ct.second = ct.second * trans;
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
      LOG_WARNING("Loaded surface " << sp->name() << " attached to body " << sp->bodyName()
                                    << " from RSDF but the robot " << name()
                                    << " has no such body, discard this surface to avoid future problems...")
    }
  }
  fixSurfaces();
}

std::map<std::string, std::vector<double>> Robot::stance() const
{
  return stance_;
}

unsigned int mc_rbdyn::Robot::robotIndex() const
{
  return robots_idx_;
}

void Robot::forwardKinematics()
{
  rbd::forwardKinematics(mb(), mbc());
}
void Robot::forwardKinematics(rbd::MultiBodyConfig & mbc) const
{
  rbd::forwardKinematics(mb(), mbc);
}

void Robot::forwardVelocity()
{
  rbd::forwardVelocity(mb(), mbc());
}
void Robot::forwardVelocity(rbd::MultiBodyConfig & mbc) const
{
  rbd::forwardVelocity(mb(), mbc);
}

void Robot::forwardAcceleration(const sva::MotionVecd & A_0)
{
  rbd::forwardAcceleration(mb(), mbc(), A_0);
}
void Robot::forwardAcceleration(rbd::MultiBodyConfig & mbc, const sva::MotionVecd & A_0) const
{
  rbd::forwardAcceleration(mb(), mbc, A_0);
}

void mc_rbdyn::Robot::eulerIntegration(double step)
{
  rbd::eulerIntegration(mb(), mbc(), step);
}

void mc_rbdyn::Robot::eulerIntegration(rbd::MultiBodyConfig & mbc, double step) const
{
  rbd::eulerIntegration(mb(), mbc, step);
}

const sva::PTransformd & Robot::posW() const
{
  return bodyPosW().at(0);
}

void Robot::posW(const sva::PTransformd & pt)
{
  if(mb().joint(0).type() == rbd::Joint::Type::Free)
  {
    const sva::Quaterniond rotation{pt.rotation().transpose()};
    q()[0] = {rotation.w(),         rotation.x(),         rotation.y(),        rotation.z(),
              pt.translation().x(), pt.translation().y(), pt.translation().z()};
    forwardKinematics();
  }
  else if(mb().joint(0).type() == rbd::Joint::Type::Fixed)
  {
    mb().transform(0, pt);
    forwardKinematics();
    fixSCH(*this, this->convexes_);
    fixSCH(*this, this->stpbvs_);
  }
  else
  {
    LOG_ERROR_AND_THROW(std::logic_error,
                        "The root pose can only be changed for robots with a free flyer or a fixed joint as joint(0)");
  }
}

void Robot::setBaseLinkVelocity(const Eigen::Vector6d & alpha_)
{
  alpha()[0][0] = alpha_(0);
  alpha()[0][1] = alpha_(1);
  alpha()[0][2] = alpha_(2);
  alpha()[0][3] = alpha_(3);
  alpha()[0][4] = alpha_(4);
  alpha()[0][5] = alpha_(5);
  forwardVelocity();
}

void Robot::copy(Robots & robots, unsigned int robots_idx, const Base & base) const
{
  robots.robots_.emplace_back(Robot(robots, robots_idx, false, &base.X_0_s, base.baseName));
  auto & robot = robots.robots_.back();
  for(const auto & s : surfaces_)
  {
    robot.surfaces_[s.first] = s.second->copy();
  }
  robot.fixSurfaces();
  for(const auto & cH : convexes_)
  {
    robot.convexes_[cH.first] = {cH.second.first, std::make_shared<sch::S_Polyhedron>(*cH.second.second)};
  }
  fixSCH(robot, robot.convexes_);
  for(const auto & stpbv : stpbvs_)
  {
    robot.stpbvs_[stpbv.first] = {stpbv.second.first, std::make_shared<sch::STP_BV>(*stpbv.second.second)};
  }
  fixSCH(robot, robot.stpbvs_);
}

void Robot::copy(Robots & robots, unsigned int robots_idx) const
{
  robots.robots_.emplace_back(Robot(robots, robots_idx, false));
  auto & robot = robots.robots_.back();
  for(const auto & s : surfaces_)
  {
    robot.surfaces_[s.first] = s.second->copy();
  }
}

mc_rbdyn::Surface & Robot::copySurface(const std::string & sName, const std::string & name)
{
  if(hasSurface(name))
  {
    LOG_ERROR_AND_THROW(std::runtime_error,
                        name << " already exists within this robot. Cannot overwrite an existing surface")
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
    LOG_WARNING("Surface " << surface->name() << " attached to body " << surface->bodyName() << " but the robot "
                           << name() << " has no such body.")
    return;
  }
  if(hasSurface(surface->name()) && doNotReplace)
  {
    LOG_WARNING("Surface " << surface->name() << " already exists for the robot " << name() << ".")
    return;
  }
  surfaces_[surface->name()] = std::move(surface);
}

#pragma GCC diagnostic pop

double mc_rbdyn::Robot::mass() const
{
  double mass = 0.;
  for(const auto & b : mb().bodies())
  {
    mass += b.inertia().mass();
  }
  return mass;
}

void mc_rbdyn::Robot::zmpTarget(const Eigen::Vector3d & zmp)
{
  zmp_ = zmp;
}

const Eigen::Vector3d & mc_rbdyn::Robot::zmpTarget() const
{
  return zmp_;
}

} // namespace mc_rbdyn
