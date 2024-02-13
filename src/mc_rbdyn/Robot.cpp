/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rbdyn/ZMP.h>
#include <mc_rbdyn/surface_utils.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/pragma.h>

#include <mc_tvm/Convex.h>
#include <mc_tvm/Robot.h>

#include <RBDyn/CoM.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/NumericalIntegration.h>

#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Superellipsoid.h>

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

#include <fstream>
#include <tuple>

namespace
{

using bound_t = std::vector<std::vector<double>>;
using bounds_t = std::tuple<bound_t, bound_t, bound_t, bound_t, bound_t, bound_t>;
using accelerationBounds_t = std::tuple<bound_t, bound_t>;
using jerkBounds_t = std::tuple<bound_t, bound_t>;
using torqueDerivativeBounds_t = std::tuple<bound_t, bound_t>;
using rm_bounds_t = mc_rbdyn::RobotModule::bounds_t;
using rm_bound_t = rm_bounds_t::value_type;

using jt_method = int (rbd::Joint::*)() const;

bound_t fill_bound(const rbd::MultiBody & mb,
                   const std::string & name,
                   const rm_bound_t & bound_in,
                   jt_method def_size,
                   double def_value,
                   double ff_def_value)
{
  bound_t res;
  res.reserve(static_cast<size_t>(mb.nrJoints()));
  for(const auto & j : mb.joints())
  {
    res.emplace_back(((j).*(def_size))(), j.type() == rbd::Joint::Free ? ff_def_value : def_value);
    if(bound_in.count(j.name()))
    {
      const auto & b_ref = bound_in.at(j.name());
      auto & b = res.back();
      if(b_ref.size() != b.size())
      {
        mc_rtc::log::error_and_throw("{} provided bound size ({}) different from expected size ({}) for joint {}", name,
                                     b_ref.size(), b.size(), j.name());
      }
      res.back() = bound_in.at(j.name());
    }
  }
  return res;
}

/** Generate bounds compatible with the given MultiBody
 *
 * If bounds is provided, use the values provided to build the bounds.
 *
 * Otherwise, default bounds are returned.
 */
bounds_t bounds(const rbd::MultiBody & mb, const rm_bounds_t & bounds)
{
  return std::make_tuple(fill_bound(mb, "lower position", bounds.at(0), &rbd::Joint::params, -INFINITY, -INFINITY),
                         fill_bound(mb, "upper position", bounds.at(1), &rbd::Joint::params, INFINITY, INFINITY),
                         fill_bound(mb, "lower velocity", bounds.at(2), &rbd::Joint::dof, -INFINITY, -INFINITY),
                         fill_bound(mb, "upper velocity", bounds.at(3), &rbd::Joint::dof, INFINITY, INFINITY),
                         fill_bound(mb, "lower torque", bounds.at(4), &rbd::Joint::dof, -INFINITY, 0),
                         fill_bound(mb, "upper torque", bounds.at(5), &rbd::Joint::dof, INFINITY, 0));
}

/** Generate acceleration bounds compatible with the given MultiBody
 *
 * If bounds is provided, use the values provided to build the bounds.
 *
 * Otherwise, default bounds are returned.
 */
accelerationBounds_t acceleration_bounds(const rbd::MultiBody & mb, const rm_bounds_t & bounds)
{
  rm_bound_t default_bound = {};
  auto safe_bounds = [&bounds, &default_bound](size_t idx) -> const rm_bound_t &
  {
    if(idx < bounds.size()) { return bounds[idx]; }
    return default_bound;
  };
  return std::make_tuple(fill_bound(mb, "lower acceleration", safe_bounds(0), &rbd::Joint::dof, -INFINITY, -INFINITY),
                         fill_bound(mb, "upper acceleration", safe_bounds(1), &rbd::Joint::dof, INFINITY, INFINITY));
}

/** Generate jerk bounds compatible with the given MultiBody
 *
 * If bounds is provided, use the values provided to build the bounds.
 *
 * Otherwise, default bounds are returned.
 */
jerkBounds_t jerk_bounds(const rbd::MultiBody & mb, const rm_bounds_t & bounds)
{
  rm_bound_t default_bound = {};
  auto safe_bounds = [&bounds, &default_bound](size_t idx) -> const rm_bound_t &
  {
    if(idx < bounds.size()) { return bounds[idx]; }
    return default_bound;
  };
  return std::make_tuple(fill_bound(mb, "lower jerk", safe_bounds(0), &rbd::Joint::dof, -INFINITY, -INFINITY),
                         fill_bound(mb, "upper jerk", safe_bounds(1), &rbd::Joint::dof, INFINITY, INFINITY));
}

/** Generate torque-derivative bounds compatible with the given MultiBody
 *
 * If bounds is provided, use the values provided to build the bounds.
 *
 * Otherwise, default bounds are returned.
 */
torqueDerivativeBounds_t torqueDerivative_bounds(const rbd::MultiBody & mb, const rm_bounds_t & bounds)
{
  rm_bound_t default_bound = {};
  auto safe_bounds = [&bounds, &default_bound](size_t idx) -> const rm_bound_t &
  {
    if(idx < bounds.size()) { return bounds[idx]; }
    return default_bound;
  };
  return std::make_tuple(
      fill_bound(mb, "lower torque-derivative", safe_bounds(0), &rbd::Joint::dof, -INFINITY, -INFINITY),
      fill_bound(mb, "upper torque-derivative", safe_bounds(1), &rbd::Joint::dof, INFINITY, INFINITY));
}

template<typename schT, typename mapT>
void loadSCH(const mc_rbdyn::Robot & robot,
             const std::map<std::string, std::pair<std::string, std::string>> & urls,
             schT * (*sch_load_fn)(const std::string &),
             mapT & data_,
             std::map<std::string, sva::PTransformd> & cTfs)
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
      cTfs[cHName] = sva::PTransformd::Identity();
    }
  }
}

template<typename mapT>
void fixSCH(const mc_rbdyn::Robot & robot, mapT & data_, const std::map<std::string, sva::PTransformd> & tfs)
{
  for(const auto & d : data_)
  {
    const auto & pos = robot.bodyPosW(d.second.first);
    if(tfs.count(d.first)) { sch::mc_rbdyn::transform(*d.second.second, tfs.at(d.first) * pos); }
    else { sch::mc_rbdyn::transform(*d.second.second, pos); }
  }
}

bool VisualToConvex(const std::string & robot,
                    const std::string & cName,
                    const std::string & bName,
                    const rbd::parsers::Visual & visual,
                    std::map<std::string, mc_rbdyn::Robot::convex_pair_t> & convexes,
                    std::map<std::string, sva::PTransformd> & collisionTransforms)
{
  // Ignore visual types that we cannot easily map to SCH
  if(visual.geometry.type == rbd::parsers::Geometry::Type::UNKNOWN
     || visual.geometry.type == rbd::parsers::Geometry::Type::MESH)
  {
    return false;
  }
  // If we already have a convex with the same name, discard loading
  if(convexes.count(cName) != 0)
  {
    mc_rtc::log::warning("While loading {}, a convex was already provided for collision geometry specified in URDF",
                         robot);
    return false;
  }
  auto fromBox = [&]()
  {
    const auto & box = boost::get<rbd::parsers::Geometry::Box>(visual.geometry.data);
    convexes[cName] = {bName, std::make_shared<sch::S_Box>(box.size.x(), box.size.y(), box.size.z())};
  };
  auto fromCylinder = [&]()
  {
    const auto & cyl = boost::get<rbd::parsers::Geometry::Cylinder>(visual.geometry.data);
    convexes[cName] = {bName, std::make_shared<sch::S_Cylinder>(sch::Point3(0, 0, -cyl.length / 2),
                                                                sch::Point3(0, 0, cyl.length / 2), cyl.radius)};
  };
  auto fromSphere = [&]()
  {
    const auto & sph = boost::get<rbd::parsers::Geometry::Sphere>(visual.geometry.data);
    convexes[cName] = {bName, std::make_shared<sch::S_Sphere>(sph.radius)};
  };
  auto fromSuperEllipsoid = [&]()
  {
    const auto & sel = boost::get<rbd::parsers::Geometry::Superellipsoid>(visual.geometry.data);
    convexes[cName] = {bName, std::make_shared<sch::S_Superellipsoid>(sel.size.x(), sel.size.y(), sel.size.z(),
                                                                      sel.epsilon1, sel.epsilon2)};
  };
  switch(visual.geometry.type)
  {
    case rbd::parsers::Geometry::Type::BOX:
      fromBox();
      break;
    case rbd::parsers::Geometry::Type::CYLINDER:
      fromCylinder();
      break;
    case rbd::parsers::Geometry::Type::SPHERE:
      fromSphere();
      break;
    case rbd::parsers::Geometry::Type::SUPERELLIPSOID:
      fromSuperEllipsoid();
      break;
    default:
      return false;
  }
  collisionTransforms[cName] = visual.origin;
  return true;
}

} // namespace

namespace mc_rbdyn
{

// We can safely ignore those since they are due to different index types and
// our index never go near unsafe territories
MC_RTC_diagnostic_push
MC_RTC_diagnostic_ignored(GCC, "-Wsign-conversion", ClangOnly, "-Wshorten-64-to-32")

Robot::Robot(NewRobotToken,
             const std::string & name,
             Robots & robots,
             unsigned int robots_idx,
             bool loadFiles,
             const LoadRobotParameters & params)
: robots_(&robots), robots_idx_(robots_idx), name_(name), load_params_(params)
{
  if(params.data_) { data_ = params.data_; }
  else { data_ = std::make_shared<RobotData>(); }
  data_->robots.push_back(this);
  const auto & module_ = module();

  sva::PTransformd base_tf = params.base_tf_.value_or(sva::PTransformd::Identity());
  std::string base_name = params.base_.value_or(mb().body(0).name());
  if(params.base_tf_ || params.base_)
  {
    mb() = mbg().makeMultiBody(base_name, mb().joint(0).type() == rbd::Joint::Fixed, base_tf);
    mbc() = rbd::MultiBodyConfig(mb());
  }

  mbc().gravity = mc_rtc::constants::gravity;
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
          mc_rtc::log::error_and_throw(
              "Mismatch between RobotModule stance for joint {}\nStance provides {} values but should be {}", j.name(),
              jQ.size(), initQ[i].size());
        }
        initQ[i] = jQ;
      }
    }
    mbc().q = initQ;
    const auto & attitude = module_.default_attitude();
    posW({Eigen::Quaterniond(attitude[0], attitude[1], attitude[2], attitude[3]).inverse(),
          Eigen::Vector3d(attitude[4], attitude[5], attitude[6])});
    forwardVelocity();
    forwardAcceleration();
  }

  mass_ = 0.;
  for(const auto & b : mb().bodies()) { mass_ += b.inertia().mass(); }

  bodyTransforms_.resize(mb().bodies().size());
  const auto & bbts = mbg().bodiesBaseTransform(base_name, base_tf);
  for(size_t i = 0; i < mb().bodies().size(); ++i)
  {
    const auto & b = mb().body(static_cast<int>(i));
    bodyTransforms_[i] = bbts.at(b.name());
  }

  if(module_.bounds().size() != 6)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("The (urdf)-bounds of RobotModule \"{}\" have a size of {} "
                                                        "instead of 6 (ql, qu, vl, vu, tl, tu).",
                                                        module_.name, module_.bounds().size());
  }
  std::tie(ql_, qu_, vl_, vu_, tl_, tu_) = bounds(mb(), module_.bounds());

  if(module_.accelerationBounds().size() != 0 && module_.accelerationBounds().size() != 2)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>(
        "The additional acceleration bounds of RobotModule \"{}\" have a size of {} "
        "instead of 2 ([al, au]).",
        module_.name, module_.accelerationBounds().size());
  }
  std::tie(al_, au_) = acceleration_bounds(mb(), module_.accelerationBounds());

  if(module_.jerkBounds().size() != 0 && module_.jerkBounds().size() != 2)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>(
        "The additional jerk bounds of RobotModule \"{}\" have a size of {} "
        "instead of 2 ([jl, ju]).",
        module_.name, module_.jerkBounds().size());
  }
  std::tie(jl_, ju_) = jerk_bounds(mb(), module_.jerkBounds());

  if(module_.torqueDerivativeBounds().size() != 0 && module_.torqueDerivativeBounds().size() != 2)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>(
        "The additional torque-derivative bounds of RobotModule \"{}\" have a size of {} "
        "instead of 2 ([tdl, tdu]).",
        module_.name, module_.torqueDerivativeBounds().size());
  }
  std::tie(tdl_, tdu_) = torqueDerivative_bounds(mb(), module_.torqueDerivativeBounds());

  if(loadFiles)
  {
    loadSCH(*this, module_.convexHull(), &sch::mc_rbdyn::Polyhedron, convexes_, collisionTransforms_);
    for(const auto & c : module_._collision)
    {
      const auto & body = c.first;
      const auto & collisions = c.second;
      if(collisions.size() == 1)
      {
        VisualToConvex(name_, body, body, collisions[0], convexes_, collisionTransforms_);
        continue;
      }
      size_t added = 0;
      for(const auto & col : collisions)
      {
        if(VisualToConvex(name_, body + "_" + std::to_string(added), body, col, convexes_, collisionTransforms_))
        {
          added++;
        }
      }
    }
    for(const auto & o : module_.collisionObjects())
    {
      if(convexes_.count(o.first) != 0)
      {
        mc_rtc::log::warning("While loading {}, another object named {} was already loaded, the object specified in "
                             "collisionObjects will be ignored",
                             name_, o.first);
        continue;
      }
      convexes_[o.first] = {o.second.first, S_ObjectPtr(o.second.second->clone())};
      auto it = module_.collisionTransforms().find(o.first);
      if(it != module_.collisionTransforms().end()) { collisionTransforms_[o.first] = it->second; }
      else { collisionTransforms_[o.first] = sva::PTransformd::Identity(); }
    }
    for(const auto & b : mb().bodies()) { collisionTransforms_[b.name()] = sva::PTransformd::Identity(); }
    for(const auto & [body, visuals] : module_._visual)
    {
      if(visuals.size() && hasBody(body)) { collisionTransforms_[body] = visuals[0].origin; }
    }
    for(const auto & p : module_.collisionTransforms()) { collisionTransforms_[p.first] = p.second; }
    fixCollisionTransforms();
    fixSCH(*this, this->convexes_, this->collisionTransforms_);
  }

  if(!params.data_)
  {
    data_->forceSensors = module_.forceSensors();
    for(size_t i = 0; i < data_->forceSensors.size(); ++i)
    {
      const auto & fs = data_->forceSensors[i];
      data_->forceSensorsIndex[fs.name()] = i;
      data_->bodyForceSensors_[fs.parentBody()] = i;
    }
  }
  auto & forceSensors_ = data_->forceSensors;
  if(loadFiles)
  {
    for(auto & fs : forceSensors_)
    {
      bfs::path calib_file = bfs::path(module_.calib_dir) / std::string("calib_data." + fs.name());
      if(!bfs::exists(calib_file))
      {
        if(params.warn_on_missing_files_)
        {
          mc_rtc::log::warning("No calibration file {} found for force sensor {}", calib_file.string(), fs.name());
        }
      }
      else { fs.loadCalibrator(calib_file.string(), mbc().gravity); }
    }
  }

  for(const auto & b : mb().bodies())
  {
    frames_[b.name()] = std::make_shared<RobotFrame>(RobotFrame::NewRobotFrameToken{}, b.name(), *this, b.name());
  }

  if(loadFiles)
  {
    if(bfs::exists(module_.rsdf_dir)) { loadRSDFFromDir(module_.rsdf_dir); }
    else if(module_.rsdf_dir.size() && params.warn_on_missing_files_)
    {
      mc_rtc::log::error("RSDF directory ({}) specified by RobotModule for {} does not exist.", module_.rsdf_dir,
                         module_.name);
    }
  }

  if(loadFiles) { makeFrames(module().frames()); }

  stance_ = module_.stance();

  if(!params.data_)
  {
    data_->bodySensors = module_.bodySensors();
    auto & bodySensors_ = data_->bodySensors;
    // Add a single default sensor if no sensor on the robot
    if(bodySensors_.size() == 0)
    {
      bodySensors_.emplace_back("Default", mb().body(0).name(), sva::PTransformd::Identity());
    }
    for(size_t i = 0; i < data_->bodySensors.size(); ++i)
    {
      const auto & bS = data_->bodySensors[i];
      data_->bodySensorsIndex[bS.name()] = i;
    }
    for(size_t i = 0; i < bodySensors_.size(); ++i)
    {
      const auto & bS = bodySensors_[i];
      data_->bodyBodySensors[bS.parentBody()] = i;
    }
  }

  if(!params.data_)
  {
    data_->jointSensors = module_.jointSensors();
    const auto & jointSensors_ = data_->jointSensors;
    for(size_t i = 0; i < jointSensors_.size(); ++i)
    {
      const auto & js = jointSensors_[i];
      data_->jointJointSensors[js.joint()] = i;
    }
  }

  if(!params.data_)
  {
    data_->devices = module_.devices();
    for(size_t i = 0; i < data_->devices.size(); ++i)
    {
      auto & d = data_->devices[i];
      if(d->parent() == "") { d->parent(mb().body(0).name()); }
      data_->devicesIndex[d->name()] = i;
    }
  }

  if(!params.data_) { data_->refJointOrder = module_.ref_joint_order(); }
  const auto & refJointOrder_ = data_->refJointOrder;
  refJointIndexToMBCIndex_.resize(refJointOrder_.size());
  for(size_t i = 0; i < refJointOrder_.size(); ++i)
  {
    const auto & jN = refJointOrder_[i];
    if(hasJoint(jN))
    {
      auto jIndex = mb().jointIndexByName(jN);
      refJointIndexToMBCIndex_[i] = mb().joint(jIndex).dof() != 0 ? jIndex : -1;
    }
    else { refJointIndexToMBCIndex_[i] = -1; }
  }

  springs_ = module_.springs();
  flexibility_ = module_.flexibility();

  zmp_ = Eigen::Vector3d::Zero();
}

Robot::~Robot()
{
  if(data_)
  {
    auto it = std::find(data_->robots.begin(), data_->robots.end(), this);
    if(it != data_->robots.end()) { data_->robots.erase(it); }
  }
}

Robot::Robot(Robot &&) = default;

Robot & Robot::operator=(Robot &&) = default;

const std::string & Robot::name() const
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

void Robot::addBodySensor(const BodySensor & sensor)
{
  if(!hasBodySensor(sensor.name()))
  {
    data_->bodySensors.push_back(sensor);
    data_->bodySensorsIndex.insert({sensor.name(), data_->bodySensors.size() - 1});

    if(!bodyHasBodySensor(sensor.parentBody()))
      data_->bodyBodySensors.insert({sensor.parentBody(), data_->bodySensors.size() - 1});
  }

  else { mc_rtc::log::error_and_throw("Body sensor named {} already attached to {}", sensor.name(), this->name()); }
}

const BodySensor & Robot::bodySensor(const std::string & name) const
{
  auto it = data_->bodySensorsIndex.find(name);
  if(it == data_->bodySensorsIndex.end())
  {
    mc_rtc::log::error_and_throw("No body sensor named {} in {}", name, this->name());
  }
  return data_->bodySensors[it->second];
}

const BodySensor & Robot::bodyBodySensor(const std::string & body) const
{
  auto it = data_->bodyBodySensors.find(body);
  if(it == data_->bodyBodySensors.end())
  {
    mc_rtc::log::error_and_throw("No body sensor attached to {} in {}", body, this->name());
  }
  return data_->bodySensors[it->second];
}

const JointSensor & Robot::jointJointSensor(const std::string & joint) const
{
  auto it = data_->jointJointSensors.find(joint);
  if(it == data_->jointJointSensors.end())
  {
    mc_rtc::log::error_and_throw("No JointSensor attached to {} joint in {}", joint, name());
  }
  return data_->jointSensors[it->second];
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

int Robot::jointIndexInMBC(size_t jointIndex) const
{
  return refJointIndexToMBCIndex_.at(jointIndex);
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

sva::PTransformd Robot::X_b1_b2(const std::string & b1, const std::string & b2) const
{
  return bodyPosW()[bodyIndexByName(b2)] * bodyPosW()[bodyIndexByName(b1)].inv();
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

sva::ForceVecd Robot::bodyWrench(const std::string & bodyName) const
{
  return frame(bodyName).wrench();
}

sva::ForceVecd Robot::surfaceWrench(const std::string & surfaceName) const
{
  return frame(surfaceName).wrench();
}

Eigen::Vector2d Robot::cop(const std::string & name, double min_pressure) const
{
  return frame(name).cop(min_pressure);
}

Eigen::Vector3d Robot::copW(const std::string & name, double min_pressure) const
{
  return frame(name).copW(min_pressure);
}

sva::ForceVecd Robot::netWrench(const std::vector<std::string> & sensorNames) const
{
  // Compute net total wrench from all sensors in contact
  sva::ForceVecd netTotalWrench{sva::ForceVecd::Zero()};
  for(const auto & sensorName : sensorNames)
  {
    const auto & sensor = forceSensor(sensorName);
    netTotalWrench += sensor.worldWrenchWithoutGravity(*this);
  }
  return netTotalWrench;
}

Eigen::Vector3d Robot::zmp(const sva::ForceVecd & netTotalWrench,
                           const Eigen::Vector3d & plane_p,
                           const Eigen::Vector3d & plane_n,
                           double minimalNetNormalForce) const
{
  return mc_rbdyn::zmp(netTotalWrench, plane_p, plane_n, minimalNetNormalForce);
}

bool Robot::zmp(Eigen::Vector3d & zmpOut,
                const sva::ForceVecd & netTotalWrench,
                const Eigen::Vector3d & plane_p,
                const Eigen::Vector3d & plane_n,
                double minimalNetNormalForce) const noexcept
{
  return mc_rbdyn::zmp(zmpOut, netTotalWrench, plane_p, plane_n, minimalNetNormalForce);
}

Eigen::Vector3d Robot::zmp(const sva::ForceVecd & netWrench,
                           const sva::PTransformd & zmpFrame,
                           double minimalNetNormalForce) const
{
  return mc_rbdyn::zmp(netWrench, zmpFrame, minimalNetNormalForce);
}

bool Robot::zmp(Eigen::Vector3d & zmpOut,
                const sva::ForceVecd & netWrench,
                const sva::PTransformd & zmpFrame,
                double minimalNetNormalForce) const noexcept
{
  return mc_rbdyn::zmp(zmpOut, netWrench, zmpFrame, minimalNetNormalForce);
}

Eigen::Vector3d Robot::zmp(const std::vector<std::string> & sensorNames,
                           const Eigen::Vector3d & plane_p,
                           const Eigen::Vector3d & plane_n,
                           double minimalNetNormalForce) const
{
  return zmp(netWrench(sensorNames), plane_p, plane_n, minimalNetNormalForce);
}

bool Robot::zmp(Eigen::Vector3d & zmpOut,
                const std::vector<std::string> & sensorNames,
                const Eigen::Vector3d & plane_p,
                const Eigen::Vector3d & plane_n,
                double minimalNetNormalForce) const noexcept
{
  return zmp(zmpOut, netWrench(sensorNames), plane_p, plane_n, minimalNetNormalForce);
}

Eigen::Vector3d Robot::zmp(const std::vector<std::string> & sensorNames,
                           const sva::PTransformd & zmpFrame,
                           double minimalNetNormalForce) const
{
  Eigen::Vector3d n = zmpFrame.rotation().row(2);
  Eigen::Vector3d p = zmpFrame.translation();
  return zmp(sensorNames, p, n, minimalNetNormalForce);
}

bool Robot::zmp(Eigen::Vector3d & zmpOut,
                const std::vector<std::string> & sensorNames,
                const sva::PTransformd & zmpFrame,
                double minimalNetNormalForce) const noexcept
{
  Eigen::Vector3d n = zmpFrame.rotation().row(2);
  Eigen::Vector3d p = zmpFrame.translation();
  return zmp(zmpOut, sensorNames, p, n, minimalNetNormalForce);
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
const std::vector<std::vector<double>> & Robot::al() const
{
  return al_;
}
const std::vector<std::vector<double>> & Robot::au() const
{
  return au_;
}
const std::vector<std::vector<double>> & Robot::jl() const
{
  return jl_;
}
const std::vector<std::vector<double>> & Robot::ju() const
{
  return ju_;
}
const std::vector<std::vector<double>> & Robot::tl() const
{
  return tl_;
}
const std::vector<std::vector<double>> & Robot::tu() const
{
  return tu_;
}
const std::vector<std::vector<double>> & Robot::tdl() const
{
  return tdl_;
}
const std::vector<std::vector<double>> & Robot::tdu() const
{
  return tdu_;
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
std::vector<std::vector<double>> & Robot::al()
{
  return al_;
}
std::vector<std::vector<double>> & Robot::au()
{
  return au_;
}
std::vector<std::vector<double>> & Robot::jl()
{
  return jl_;
}
std::vector<std::vector<double>> & Robot::ju()
{
  return ju_;
}
std::vector<std::vector<double>> & Robot::tl()
{
  return tl_;
}
std::vector<std::vector<double>> & Robot::tu()
{
  return tu_;
}
std::vector<std::vector<double>> & Robot::tdl()
{
  return tdl_;
}
std::vector<std::vector<double>> & Robot::tdu()
{
  return tdu_;
}

const std::vector<Flexibility> & Robot::flexibility() const
{
  return flexibility_;
}

std::vector<Flexibility> & Robot::flexibility()
{
  return flexibility_;
}

void Robot::addForceSensor(const mc_rbdyn::ForceSensor & fs)
{
  auto it = data_->forceSensorsIndex.find(fs.name());
  if(it != data_->forceSensorsIndex.end())
  {
    mc_rtc::log::error_and_throw("Cannot add a force sensor named {} since {} already has one", fs.name(),
                                 this->name());
  }
  data_->forceSensors.push_back(fs);
  data_->forceSensorsIndex[fs.name()] = data_->forceSensors.size() - 1;
  auto bfs_it = data_->bodyForceSensors_.find(fs.parentBody());
  if(bfs_it == data_->bodyForceSensors_.end())
  {
    data_->bodyForceSensors_[fs.parentBody()] = data_->forceSensors.size() - 1;
  }
  auto updateFrames = [](const mc_rbdyn::Robot & robot)
  {
    for(const auto & f : robot.frames_) { f.second->resetForceSensor(); }
  };
  for(auto & r : data_->robots) { updateFrames(*r); }
}

const ForceSensor & Robot::forceSensor(const std::string & name) const
{
  auto it = data_->forceSensorsIndex.find(name);
  if(it == data_->forceSensorsIndex.end())
  {
    mc_rtc::log::error_and_throw("No force sensor named {} in {}", name, this->name());
  }
  return data_->forceSensors[it->second];
}

const ForceSensor & Robot::bodyForceSensor(const std::string & body) const
{
  auto it = data_->bodyForceSensors_.find(body);
  if(it == data_->bodyForceSensors_.end())
  {
    mc_rtc::log::error_and_throw("No force sensor directly attached to {} in {}", body, name());
  }
  return data_->forceSensors[it->second];
}

const ForceSensor & Robot::surfaceForceSensor(const std::string & surfaceName) const
{
  return bodyForceSensor(surface(surfaceName).bodyName());
}

std::string Robot::findIndirectForceSensorBodyName(const std::string & body) const
{
  int nextIndex = mb().bodyIndexByName().at(body);
  while(nextIndex >= 0)
  {
    const auto & b = mb().body(nextIndex);
    if(bodyHasForceSensor(b.name())) { return b.name(); }
    nextIndex = mb().parent(nextIndex);
  }
  return std::string{};
}

const ForceSensor & Robot::indirectBodyForceSensor(const std::string & body) const
{
  const auto bodyName = findIndirectForceSensorBodyName(body);
  if(bodyName.empty())
  {
    mc_rtc::log::error_and_throw("No force sensor (directly or indirectly) attached to body {} in {}", body, name());
  }
  return bodyForceSensor(bodyName);
}

const ForceSensor & Robot::indirectSurfaceForceSensor(const std::string & surfaceName) const
{
  return indirectBodyForceSensor(surface(surfaceName).bodyName());
}

bool Robot::hasSurface(const std::string & surface) const
{
  return surfaces_.count(surface) != 0;
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
  if(!hasSurface(sName)) { mc_rtc::log::error_and_throw("No surface named {} found in robot {}", sName, this->name()); }
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
  for(const auto & s : surfaces_) { ret.push_back(s.first); }
  return ret;
}

bool Robot::hasConvex(const std::string & name) const
{
  return convexes_.count(name);
}

Robot::convex_pair_t & Robot::convex(const std::string & cName)
{
  return const_cast<Robot::convex_pair_t &>(static_cast<const Robot *>(this)->convex(cName));
}
const Robot::convex_pair_t & Robot::convex(const std::string & cName) const
{
  if(convexes_.count(cName) == 0)
  {
    mc_rtc::log::error_and_throw("No convex named {} found in robot {}", cName, this->name_);
  }
  return convexes_.at(cName);
}

const std::map<std::string, Robot::convex_pair_t> & Robot::convexes() const
{
  return convexes_;
}

void Robot::addConvex(const std::string & cName,
                      const std::string & body,
                      S_ObjectPtr convex,
                      const sva::PTransformd & X_b_c)
{
  if(convexes_.count(cName))
  {
    mc_rtc::log::error("Attempted to add a convex named {} that already exists in {}", cName, name());
    return;
  }
  convexes_[cName] = {body, convex};
  collisionTransforms_[cName] = X_b_c;
  sch::mc_rbdyn::transform(*convex, X_b_c * bodyPosW(body));
}

void Robot::removeConvex(const std::string & cName)
{
  if(convexes_.count(cName))
  {
    convexes_.erase(cName);
    collisionTransforms_.erase(cName);
  }
}

const sva::PTransformd & Robot::bodyTransform(const std::string & bName) const
{
  if(!hasBody(bName)) { mc_rtc::log::error_and_throw("No body transform with name {} found in this robot", bName); }
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
    mc_rtc::log::error_and_throw("No collision transform with name {} found in this robot", cName);
  }
  return collisionTransforms_.at(cName);
}

void Robot::fixSurfaces()
{
  for(auto & surface : surfaces_) { fixSurface(*surface.second); }
}

void Robot::fixSurface(Surface & surface)
{
  const sva::PTransformd & trans = bodyTransform(surface.bodyName());
  surface.X_b_s(surface.X_b_s() * trans);
  makeFrame(surface.name(), frame(surface.bodyName()), surface.X_b_s());
}

void Robot::makeFrames(std::vector<mc_rbdyn::RobotModule::FrameDescription> frames)
{
  size_t added_frames = 0;
  do {
    added_frames = 0;
    for(auto it = frames.begin(); it != frames.end();)
    {
      const auto & desc = *it;
      auto frame_it = frames_.find(desc.parent);
      if(frame_it != frames_.end())
      {
        makeFrame(desc.name, *frame_it->second, desc.X_p_f, desc.baked);
        it = frames.erase(it);
        added_frames++;
      }
      else { ++it; }
    }
  } while(added_frames != 0);
  if(frames.size())
  {
    mc_rtc::log::error("{} frames could not be loaded from their description (parent missing or cycles)",
                       frames.size());
    for(const auto & desc : frames) { mc_rtc::log::warning("- {} (parent: {})", desc.name, desc.parent); }
  }
}

void Robot::fixCollisionTransforms()
{
  for(auto & ct : collisionTransforms_)
  {
    if(convexes_.count(ct.first))
    {
      const auto & trans = bodyTransform(convexes_.at(ct.first).first);
      ct.second = ct.second * trans;
    }
    else
    {
      const auto & trans = bodyTransform(ct.first);
      ct.second = ct.second * trans;
    }
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
      if(hasSurface(sp->name()))
      {
        mc_rtc::log::warning("This robot already has a surface named {}, ignoring loading from {}", sp->name(),
                             surfaceDir);
      }
      else
      {
        surfaces_[sp->name()] = sp;
        fixSurface(*sp);
      }
    }
    else
    {
      mc_rtc::log::warning("Loaded surface {} attached to body {} from RSDF but the robot {} has no such body, discard "
                           "this surface to avoid future problems...",
                           sp->name(), sp->bodyName(), name());
    }
  }
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
  forwardKinematics(mbc());
}
void Robot::forwardKinematics(rbd::MultiBodyConfig & mbc) const
{
  rbd::forwardKinematics(mb(), mbc);

  for(const auto & cvx : convexes_)
  {
    auto get_cvx_tf = [&]()
    {
      unsigned int index = static_cast<unsigned int>(mb().bodyIndexByName(cvx.second.first));
      auto tfs_it = collisionTransforms_.find(cvx.first);
      if(tfs_it != collisionTransforms_.end()) { return tfs_it->second * mbc.bodyPosW[index]; }
      return mbc.bodyPosW[index];
    };
    sch::mc_rbdyn::transform(*cvx.second.second, get_cvx_tf());
  }
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
  rbd::integration(mb(), mbc(), step);
}

void mc_rbdyn::Robot::eulerIntegration(rbd::MultiBodyConfig & mbc, double step) const
{
  rbd::integration(mb(), mbc, step);
}

const sva::PTransformd & Robot::posW() const
{
  return bodyPosW().at(0);
}

void Robot::posW(const sva::PTransformd & pt)
{
  if(mb().joint(0).type() == rbd::Joint::Type::Free)
  {
    Eigen::Quaterniond rotation{pt.rotation().transpose()};
    rotation.normalize();
    q()[0] = {rotation.w(),         rotation.x(),         rotation.y(),        rotation.z(),
              pt.translation().x(), pt.translation().y(), pt.translation().z()};
    forwardKinematics();
  }
  else if(mb().joint(0).type() == rbd::Joint::Type::Fixed)
  {
    sva::PTransformd pt_ = pt;
    pt_.rotation() = Eigen::Quaterniond(pt.rotation()).normalized().toRotationMatrix();
    mb().transform(0, pt_);
    forwardKinematics();
    fixSCH(*this, this->convexes_, this->collisionTransforms_);
  }
  else
  {
    mc_rtc::log::error_and_throw<std::logic_error>(
        "The root pose can only be changed for robots with a free flyer or a fixed joint as joint(0)");
  }
}

void Robot::velW(const sva::MotionVecd & vel)
{
  if(mb().joint(0).type() == rbd::Joint::Type::Free)
  {
    auto vB = sva::PTransformd(mbc().bodyPosW[0].rotation()) * vel;
    alpha()[0][0] = vB.angular().x();
    alpha()[0][1] = vB.angular().y();
    alpha()[0][2] = vB.angular().z();
    alpha()[0][3] = vB.linear().x();
    alpha()[0][4] = vB.linear().y();
    alpha()[0][5] = vB.linear().z();
    forwardVelocity();
  }
  else { mc_rtc::log::warning("You cannot set the base velocity on a fixed-base robot"); }
}

const sva::MotionVecd & Robot::velW() const
{
  return bodyVelW().at(0);
}

void Robot::accW(const sva::MotionVecd & acc)
{
  if(mb().joint(0).type() == rbd::Joint::Type::Free)
  {
    auto aB = sva::PTransformd(mbc().bodyPosW[0].rotation()) * acc;
    alphaD()[0][0] = aB.angular().x();
    alphaD()[0][1] = aB.angular().y();
    alphaD()[0][2] = aB.angular().z();
    alphaD()[0][3] = aB.linear().x();
    alphaD()[0][4] = aB.linear().y();
    alphaD()[0][5] = aB.linear().z();
    forwardAcceleration();
  }
  else { mc_rtc::log::warning("You cannot set the base acceleration on a fixed-base robot"); }
}

const sva::MotionVecd Robot::accW() const
{
  Eigen::Matrix3d rot = posW().rotation().transpose();
  return sva::PTransformd{rot} * mbc().bodyAccB[0];
}

void Robot::copyLoadedData(Robot & robot) const
{
  for(const auto & s : surfaces_) { robot.surfaces_[s.first] = s.second->copy(); }
  robot.fixSurfaces();
  robot.makeFrames(module().frames());
  for(const auto & cH : convexes_)
  {
    robot.convexes_[cH.first] = {cH.second.first, S_ObjectPtr(cH.second.second->clone())};
  }
  robot.collisionTransforms_ = collisionTransforms_;
  robot.fixCollisionTransforms();
  fixSCH(robot, robot.convexes_, robot.collisionTransforms_);
  for(size_t i = 0; i < data_->forceSensors.size(); ++i)
  {
    robot.data_->forceSensors[i].copyCalibrator(data_->forceSensors[i]);
  }
}

mc_rbdyn::Surface & Robot::copySurface(const std::string & sName, const std::string & name)
{
  if(hasSurface(name))
  {
    mc_rtc::log::error_and_throw("{} already exists within robot {}. Cannot overwrite an existing surface", name,
                                 this->name_);
  }
  const Surface & surf = surface(sName);
  SurfacePtr nSurf = surf.copy();
  nSurf->name(name);
  makeFrame(name, frame(surf.bodyName()), surf.X_b_s());
  surfaces_[name] = nSurf;
  return *nSurf;
}

void mc_rbdyn::Robot::addSurface(SurfacePtr surface, bool doNotReplace)
{
  if(!hasBody(surface->bodyName()))
  {
    mc_rtc::log::warning("Surface {} attached to body {} but robot {} has no such body.", surface->name(),
                         surface->bodyName(), name());
    return;
  }
  bool has_surface = hasSurface(surface->name());
  if(has_surface && doNotReplace)
  {
    mc_rtc::log::warning("Surface {} already exists for the robot {}.", surface->name(), name());
    return;
  }
  if(has_surface) { frames_.erase(frames_.find(surface->name())); }
  makeFrame(surface->name(), frame(surface->bodyName()), surface->X_b_s());
  surfaces_[surface->name()] = std::move(surface);
}

MC_RTC_diagnostic_pop

void mc_rbdyn::Robot::zmpTarget(const Eigen::Vector3d & zmp)
{
  zmp_ = zmp;
}

const Eigen::Vector3d & mc_rbdyn::Robot::zmpTarget() const
{
  return zmp_;
}

mc_control::Gripper & Robot::gripper(const std::string & gripper)
{
  return const_cast<mc_control::Gripper &>(static_cast<const Robot *>(this)->gripper(gripper));
}

const mc_control::Gripper & Robot::gripper(const std::string & gripper) const
{
  auto it = data_->grippers.find(gripper);
  if(it == data_->grippers.end()) { mc_rtc::log::error_and_throw("No gripper named {} in robot {}", gripper, name()); }
  return *it->second;
}

bool Robot::hasGripper(const std::string & gripper) const
{
  return data_->grippers.count(gripper);
}

unsigned int robotIndexFromConfig(const mc_rtc::Configuration & config,
                                  const mc_rbdyn::Robots & robots,
                                  const std::string & prefix,
                                  bool required,
                                  const std::string & robotIndexKey,
                                  const std::string & robotNameKey,
                                  const std::string & defaultRobotName)
{
  const auto & robot = robotFromConfig(config, robots, prefix, required, robotIndexKey, robotNameKey, defaultRobotName);
  return robot.robotIndex();
}

std::string robotNameFromConfig(const mc_rtc::Configuration & config,
                                const mc_rbdyn::Robots & robots,
                                const std::string & prefix,
                                bool required,
                                const std::string & robotIndexKey,
                                const std::string & robotNameKey,
                                const std::string & defaultRobotName)
{
  const auto & robot = robotFromConfig(config, robots, prefix, required, robotIndexKey, robotNameKey, defaultRobotName);
  return robot.name();
}

const mc_rbdyn::Robot & robotFromConfig(const mc_rtc::Configuration & config,
                                        const mc_rbdyn::Robots & robots,
                                        const std::string & prefix,
                                        bool required,
                                        const std::string & robotIndexKey,
                                        const std::string & robotNameKey,
                                        const std::string & defaultRobotName)
{
  auto p = std::string{""};
  if(prefix.size()) { p = "[" + prefix + "] "; }
  if(config.has(robotNameKey))
  {
    const std::string & robotName = config(robotNameKey);
    if(robots.hasRobot(robotName)) { return robots.robot(robotName); }
    else { mc_rtc::log::error_and_throw("{} No robot named {} in this controller", p, robotName); }
  }
  else if(config.has(robotIndexKey))
  {
    mc_rtc::log::warning("[MC_RTC_DEPRECATED]{} \"robotIndex\" will be deprecated in future versions, use \"robot: "
                         "<robot name>\" instead",
                         p);
    const unsigned int robotIndex = config(robotIndexKey);
    if(robotIndex < robots.size()) { return robots.robot(robotIndex); }
    else
    {
      mc_rtc::log::error_and_throw("{}No robot with index {} in this controller ({} robots loaded)", p, robotIndex,
                                   robots.size());
    }
  }
  else
  {
    if(!required) { return defaultRobotName.size() ? robots.robot(defaultRobotName) : robots.robot(); }
    else { mc_rtc::log::error_and_throw("{} \"robotName\" is required.", p); }
  }
}

void Robot::addDevice(DevicePtr device)
{
  if(data_->devicesIndex.count(device->name()))
  {
    mc_rtc::log::error_and_throw("You cannot have multiple generic sensor with the same name in a robot");
  }
  data_->devices.push_back(std::move(device));
  auto & d = data_->devices.back();
  if(d->parent() == "") { d->parent(mb().body(0).name()); }
  data_->devicesIndex[device->name()] = data_->devices.size() - 1;
}

std::vector<std::string> Robot::frames() const
{
  std::vector<std::string> ret;
  ret.reserve(frames_.size());
  for(const auto & f : frames_) { ret.push_back(f.first); }
  return ret;
}

RobotFrame & Robot::makeFrame(const std::string & name, RobotFrame & parent, sva::PTransformd X_p_f, bool baked)
{
  if(hasFrame(name)) { mc_rtc::log::error_and_throw("{} already has a frame named {}", name_, name); }
  auto frame = std::make_shared<RobotFrame>(RobotFrame::NewRobotFrameToken{}, name, parent, X_p_f, baked);
  frames_[name] = frame;
  return *frame;
}

RobotFramePtr Robot::makeTemporaryFrame(const std::string & name,
                                        const RobotFrame & parent,
                                        sva::PTransformd X_p_f,
                                        bool baked) const
{
  /* const_cast is OK here because we never created const RobotFrame objects */
  return std::make_shared<RobotFrame>(RobotFrame::NewRobotFrameToken{}, name, const_cast<RobotFrame &>(parent), X_p_f,
                                      baked);
}

const ForceSensor * Robot::findBodyForceSensor(const std::string & body) const
{
  auto it = data_->bodyForceSensors_.find(body);
  if(it != data_->bodyForceSensors_.end()) { return &data_->forceSensors[it->second]; }
  auto bodyName = findIndirectForceSensorBodyName(body);
  if(!bodyName.empty()) { return &data_->forceSensors[data_->bodyForceSensors_.find(bodyName)->second]; }
  return nullptr;
}

mc_tvm::Robot & Robot::tvmRobot() const
{
  if(!tvm_robot_) { tvm_robot_.reset(new mc_tvm::Robot(mc_tvm::Robot::NewRobotToken{}, *this)); }
  return *tvm_robot_;
}

mc_tvm::Convex & Robot::tvmConvex(const std::string & name) const
{
  auto it = tvm_convexes_.find(name);
  if(it == tvm_convexes_.end())
  {
    const auto & cvx = convex(name);
    std::tie(it, std::ignore) = tvm_convexes_.insert(
        {name, std::unique_ptr<mc_tvm::Convex>{new mc_tvm::Convex(mc_tvm::Convex::NewConvexToken{}, cvx.second,
                                                                  frame(cvx.first), collisionTransform(name))}});
  }
  return *it->second;
}

} // namespace mc_rbdyn
