/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#include <mc_filter/utils/clamp.h>
#include <mc_rbdyn/ZMP.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/constants.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

#include <chrono>

namespace mc_tasks
{
namespace lipm_stabilizer
{

using internal::Contact;
using ::mc_filter::utils::clamp;
using ::mc_filter::utils::clampInPlaceAndWarn;
namespace constants = ::mc_rtc::constants;

StabilizerTask::StabilizerTask(const mc_rbdyn::Robots & robots,
                               const mc_rbdyn::Robots & realRobots,
                               unsigned int robotIndex,
                               const std::string & leftSurface,
                               const std::string & rightSurface,
                               const std::string & torsoBodyName,
                               double dt)
: robots_(robots), realRobots_(realRobots), robotIndex_(robotIndex), dcmEstimator_(dt),
  extWrenchSumLowPass_(dt, /* cutoffPeriod = */ 0.05), comOffsetLowPass_(dt, /* cutoffPeriod = */ 0.05),
  comOffsetLowPassCoM_(dt, /* cutoffPeriod = */ 1.0), comOffsetDerivator_(dt, /* timeConstant = */ 1.),
  dcmIntegrator_(dt, /* timeConstant = */ 15.), dcmDerivator_(dt, /* timeConstant = */ 1.), dt_(dt),
  mass_(robots.robot(robotIndex).mass())
{
  type_ = "lipm_stabilizer";
  name_ = type_ + "_" + robots.robot(robotIndex).name();

  comTask.reset(new mc_tasks::CoMTask(robots, robotIndex_));
  auto leftCoP = std::allocate_shared<mc_tasks::force::CoPTask>(Eigen::aligned_allocator<mc_tasks::force::CoPTask>{},
                                                                leftSurface, robots, robotIndex_);
  auto rightCoP = std::allocate_shared<mc_tasks::force::CoPTask>(Eigen::aligned_allocator<mc_tasks::force::CoPTask>{},
                                                                 rightSurface, robots, robotIndex_);
  footTasks[ContactState::Left] = leftCoP;
  footTasks[ContactState::Right] = rightCoP;

  std::string pelvisBodyName = robot().mb().body(0).name();
  pelvisTask = std::make_shared<mc_tasks::OrientationTask>(pelvisBodyName, robots_, robotIndex_);
  torsoTask = std::make_shared<mc_tasks::OrientationTask>(torsoBodyName, robots_, robotIndex_);

  // Rename the tasks managed by the stabilizer
  name(name_);
}

StabilizerTask::StabilizerTask(const mc_rbdyn::Robots & robots,
                               const mc_rbdyn::Robots & realRobots,
                               unsigned int robotIndex,
                               double dt)
: StabilizerTask(robots,
                 realRobots,
                 robotIndex,
                 robots.robot(robotIndex).module().defaultLIPMStabilizerConfiguration().leftFootSurface,
                 robots.robot(robotIndex).module().defaultLIPMStabilizerConfiguration().rightFootSurface,
                 robots.robot(robotIndex).module().defaultLIPMStabilizerConfiguration().torsoBodyName,
                 dt)
{
  configure(robots.robot(robotIndex).module().defaultLIPMStabilizerConfiguration());
  setContacts({ContactState::Left, ContactState::Right});
  reset();
}

void StabilizerTask::reset()
{
  t_ = 0;
  comTask->reset();
  comTarget_ = comTask->com();
  comTargetRaw_ = comTarget_;
  zmpTarget_ = Eigen::Vector3d{comTarget_.x(), comTarget_.y(), 0.};
  zmpdTarget_ = Eigen::Vector3d::Zero();

  for(auto footTask : footTasks)
  {
    footTask.second->reset();
  }

  pelvisTask->reset();
  torsoTask->reset();

  dcmAverageError_ = Eigen::Vector3d::Zero();
  dcmError_ = Eigen::Vector3d::Zero();
  dcmVelError_ = Eigen::Vector3d::Zero();
  dfzForceError_ = 0.;
  dfzHeightError_ = 0.;
  desiredWrench_ = sva::ForceVecd::Zero();
  distribWrench_ = sva::ForceVecd::Zero();
  vdcHeightError_ = 0.;

  zmpcc_.reset();

  dcmEstimatorNeedsReset_ = true;

  extWrenches_.clear();
  extWrenchSumTarget_ = sva::ForceVecd::Zero();
  extWrenchSumMeasured_ = sva::ForceVecd::Zero();
  comOffsetTarget_ = Eigen::Vector3d::Zero();
  comOffsetMeasured_ = Eigen::Vector3d::Zero();
  comOffsetErr_ = Eigen::Vector3d::Zero();
  comOffsetErrCoM_ = Eigen::Vector3d::Zero();
  comOffsetErrZMP_ = Eigen::Vector3d::Zero();
  extWrenchSumLowPass_.reset(sva::ForceVecd::Zero());
  comOffsetLowPass_.reset(Eigen::Vector3d::Zero());
  comOffsetLowPassCoM_.reset(Eigen::Vector3d::Zero());
  comOffsetDerivator_.reset(Eigen::Vector3d::Zero());

  dcmDerivator_.reset(Eigen::Vector3d::Zero());
  dcmIntegrator_.reset(Eigen::Vector3d::Zero());

  omega_ = std::sqrt(constants::gravity.z() / robot().com().z());
  commitConfig();
}

void StabilizerTask::dimWeight(const Eigen::VectorXd & /* dim */)
{
  mc_rtc::log::error_and_throw<std::runtime_error>("dimWeight not implemented for task {}", type_);
}

Eigen::VectorXd StabilizerTask::dimWeight() const
{
  mc_rtc::log::error_and_throw<std::runtime_error>("dimWeight not implemented for task {}", type_);
}

void StabilizerTask::selectActiveJoints(mc_solver::QPSolver & /* solver */,
                                        const std::vector<std::string> & /* activeJointsName */,
                                        const std::map<std::string, std::vector<std::array<int, 2>>> & /* activeDofs */)
{
  mc_rtc::log::error_and_throw<std::runtime_error>("Task {} does not implement selectActiveJoints. Please configure it "
                                                   "through the stabilizer configuration instead",
                                                   name_);
}

void StabilizerTask::selectUnactiveJoints(
    mc_solver::QPSolver & /* solver */,
    const std::vector<std::string> & /* unactiveJointsName */,
    const std::map<std::string, std::vector<std::array<int, 2>>> & /* unactiveDofs */)
{
  mc_rtc::log::error_and_throw<std::runtime_error>(
      "Task {} does not implement selectUnactiveJoints. Please configure it "
      "through the stabilizer configuration instead.",
      name_);
}

void StabilizerTask::resetJointsSelector(mc_solver::QPSolver & /* solver */)
{
  mc_rtc::log::error_and_throw<std::runtime_error>(
      "Task {} does not implement resetJointsSelector. Please configure it "
      "through the stabilizer configuration instead.",
      name_);
}

Eigen::VectorXd StabilizerTask::eval() const
{
  Eigen::VectorXd res(3 + 3 * contactTasks.size());
  res.head(3) = comTask->eval();
  int i = 0;
  for(const auto & task : contactTasks)
  {
    res.segment(3 + 3 * i++, 3) = task->eval();
  }
  return res;
}

Eigen::VectorXd StabilizerTask::speed() const
{
  Eigen::VectorXd res(3 + 3 * contactTasks.size());
  res.head(3) = comTask->speed();
  int i = 0;
  for(const auto & task : contactTasks)
  {
    res.segment(3 + 3 * i++, 3) = task->speed();
  }
  return res;
}

void StabilizerTask::addToSolver(mc_solver::QPSolver & solver)
{
  // Feet tasks are added in update() instead, add all other tasks now
  MetaTask::addToSolver(*comTask, solver);
  MetaTask::addToSolver(*pelvisTask, solver);
  MetaTask::addToSolver(*torsoTask, solver);
}

void StabilizerTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  MetaTask::removeFromSolver(*comTask, solver);
  MetaTask::removeFromSolver(*pelvisTask, solver);
  MetaTask::removeFromSolver(*torsoTask, solver);
  for(const auto footTask : contactTasks)
  {
    MetaTask::removeFromSolver(*footTask, solver);
  }
}

void StabilizerTask::updateContacts(mc_solver::QPSolver & solver)
{
  if(!addContacts_.empty())
  {
    // Remove previous contacts
    for(const auto & contactT : contactTasks)
    {
      if(c_.verbose)
      {
        mc_rtc::log::info("{}: Removing contact {}", name(), contactT->surface());
      }
      MetaTask::removeFromLogger(*contactT, *solver.logger());
      MetaTask::removeFromSolver(*contactT, solver);
    }
    contactTasks.clear();
    contactSensors.clear();

    // Add new contacts
    for(const auto contactState : addContacts_)
    {
      auto footTask = footTasks[contactState];
      if(c_.verbose)
      {
        mc_rtc::log::info("{}: Adding contact {}", name(), footTask->surface());
      }
      MetaTask::addToSolver(*footTask, solver);
      MetaTask::addToLogger(*footTask, *solver.logger());
      contactTasks.push_back(footTask);
      const auto & fs = robot().indirectSurfaceForceSensor(footTask->surface());
      contactSensors.push_back(fs.name());
    }
    addContacts_.clear();
  }
}

void StabilizerTask::update(mc_solver::QPSolver & solver)
{
  // Prevent configuration changes while the stabilizer is disabled
  if(!enabled_)
  {
    c_ = disableConfig_;
    zmpcc_.configure(c_.zmpcc);
  }
  if(reconfigure_) configure_(solver);

  // Update contacts if they have changed
  updateContacts(solver);

  updateState(realRobots_.robot().com(), realRobots_.robot().comVelocity());

  // Run stabilizer
  run();

  MetaTask::update(*comTask, solver);
  MetaTask::update(*pelvisTask, solver);
  MetaTask::update(*torsoTask, solver);
  for(const auto footTask : contactTasks)
  {
    MetaTask::update(*footTask, solver);
  }

  t_ += dt_;
}

void StabilizerTask::enable()
{
  mc_rtc::log::info("[StabilizerTask] enabled");
  // Reset DCM integrator when enabling the stabilizer.
  // While idle, it will accumulate a lot of error, and would case the robot to
  // move suddently to compensate it otherwise
  dcmIntegrator_.reset(Eigen::Vector3d::Zero());
  dcmDerivator_.reset(Eigen::Vector3d::Zero());

  extWrenchSumLowPass_.reset(sva::ForceVecd::Zero());
  comOffsetLowPass_.reset(Eigen::Vector3d::Zero());
  comOffsetLowPassCoM_.reset(Eigen::Vector3d::Zero());
  comOffsetDerivator_.reset(Eigen::Vector3d::Zero());

  configure(lastConfig_);
  zmpcc_.enabled(true);
  enabled_ = true;
}

void StabilizerTask::disable()
{
  mc_rtc::log::info("[StabilizerTask] disabled");
  // Save current configuration to be reused when re-enabling
  lastConfig_ = c_;
  disableConfig_ = c_;
  // Set the stabilizer gains to zero
  disableConfig_.copAdmittance.setZero();
  disableConfig_.dcmDerivGain = 0.;
  disableConfig_.dcmIntegralGain = 0.;
  disableConfig_.dcmPropGain = 0.;
  disableConfig_.comdErrorGain = 0.;
  disableConfig_.zmpdGain = 0.;
  disableConfig_.dfzAdmittance = 0.;
  disableConfig_.vdcFrequency = 0.;
  disableConfig_.vdcStiffness = 0.;
  zmpcc_.enabled(false);
  enabled_ = false;
}

void StabilizerTask::reconfigure()
{
  mc_rtc::log::info("[StabilizerTask] reconfigured to the last commited configuration");
  configure(defaultConfig_);
  enable();
}

void StabilizerTask::configure(const StabilizerConfiguration & config)
{
  checkConfiguration(config);
  lastConfig_ = config;
  disableConfig_ = config;
  c_ = config;
  c_.clampGains();
  reconfigure_ = true;
}

void StabilizerTask::commitConfig()
{
  defaultConfig_ = c_;
}

void StabilizerTask::configure_(mc_solver::QPSolver & solver)
{
  dcmDerivator_.timeConstant(c_.dcmDerivatorTimeConstant);
  dcmIntegrator_.timeConstant(c_.dcmIntegratorTimeConstant);
  dcmIntegrator_.saturation(c_.safetyThresholds.MAX_AVERAGE_DCM_ERROR);

  extWrenchSumLowPass_.cutoffPeriod(c_.extWrench.extWrenchSumLowPassCutoffPeriod);
  comOffsetLowPass_.cutoffPeriod(c_.extWrench.comOffsetLowPassCutoffPeriod);
  comOffsetLowPassCoM_.cutoffPeriod(c_.extWrench.comOffsetLowPassCoMCutoffPeriod);
  comOffsetDerivator_.timeConstant(c_.extWrench.comOffsetDerivatorTimeConstant);

  // // Configure upper-body tasks
  pelvisTask->stiffness(c_.pelvisStiffness);
  pelvisTask->weight(c_.pelvisWeight);

  torsoTask->stiffness(c_.torsoStiffness);
  torsoTask->weight(c_.torsoWeight);
  torsoTask->orientation(mc_rbdyn::rpyToMat({0, c_.torsoPitch, 0}));

  zmpcc_.configure(c_.zmpcc);

  if(!c_.comActiveJoints.empty())
  {
    comTask->selectActiveJoints(solver, c_.comActiveJoints);
  }
  comTask->setGains(c_.comStiffness, 2 * c_.comStiffness.cwiseSqrt());
  comTask->weight(c_.comWeight);

  for(const auto & footTask : footTasks)
  {
    footTask.second->maxLinearVel(c_.copMaxVel.linear());
    footTask.second->maxAngularVel(c_.copMaxVel.angular());
  }

  dcmBiasEstimatorConfiguration(c_.dcmBias);

  reconfigure_ = false;
}

void StabilizerTask::checkConfiguration(const StabilizerConfiguration & config)
{
  // Check whether feet have force sensors
  auto checkSurface = [&](const std::string & surfaceName) {
    if(!robot().hasSurface(surfaceName))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}] requires a surface named {} in robot {}", name(),
                                                       surfaceName, robot().name());
    }
    if(!robot().surfaceHasIndirectForceSensor(surfaceName))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Surface {} must have an associated force sensor.", name(),
                                                       surfaceName);
    }
  };
  checkSurface(config.rightFootSurface);
  checkSurface(config.leftFootSurface);
}

void StabilizerTask::load(mc_solver::QPSolver &, const mc_rtc::Configuration & config)
{
  double height = 0;
  // Load contacts
  ContactDescriptionVector contactsToAdd;
  if(config.has("contacts"))
  {
    const auto & contacts = config("contacts");
    for(const auto & contactName : contacts)
    {
      ContactState s = contactName;
      sva::PTransformd contactPose = footTasks[s]->surfacePose();
      if(config.has(contactName))
      {
        const auto & c = config(contactName);
        if(c.has("rotation"))
        {
          contactPose.rotation() = c("rotation");
        }
        else if(c.has("overwriteRPY"))
        {
          // Only modify the specified DoF of the rotation
          mc_rtc::overwriteRotationRPY(c("overwriteRPY"), contactPose.rotation());
        }
        if(c.has("translation"))
        {
          contactPose.translation() = c("translation");
        }
        if(c.has("height"))
        {
          double h = c("height");
          contactPose.translation().z() = c("height");
          height = (height + h) / 2;
        }
      }
      contactsToAdd.push_back({s, {robot(), footTasks[s]->surface(), contactPose, c_.friction}});
    }
  }
  this->setContacts(contactsToAdd);

  // Target robot com by default
  Eigen::Vector3d comTarget = robot().com();
  if(config.has("staticTarget"))
  {
    if(config.has("com"))
    {
      comTarget = config("staticTarget")("com");
    }
  }
  this->staticTarget(comTarget, height);

  // Allow to start in disabled state
  if(!config("enabled", true))
  {
    this->disable();
  }
}

const StabilizerConfiguration & StabilizerTask::config() const
{
  return c_;
}

const StabilizerConfiguration & StabilizerTask::commitedConfig() const
{
  return defaultConfig_;
}

void StabilizerTask::setContacts(const std::vector<ContactState> & contacts)
{
  ContactDescriptionVector addContacts;
  for(const auto contact : contacts)
  {
    addContacts.push_back({contact,
                           {robot(), footTasks[contact]->surface(),
                            realRobot().surfacePose(footTasks[contact]->surface()), c_.friction}});
  }
  setContacts(addContacts);
}

void StabilizerTask::setContacts(const std::vector<std::pair<ContactState, sva::PTransformd>> & contacts)
{
  ContactDescriptionVector addContacts;
  for(const auto contact : contacts)
  {
    addContacts.push_back({contact.first, {robot(), footTasks[contact.first]->surface(), contact.second, c_.friction}});
  }
  setContacts(addContacts);
}

void StabilizerTask::setContacts(const ContactDescriptionVector & contacts)
{
  if(contacts.empty())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[StabilizerTask] Cannot set contacts from an empty list, the stabilizer "
        "requires at least one contact to be set.");
  }
  contacts_.clear();

  // Reset support area boundaries
  supportMin_ = std::numeric_limits<double>::max() * Eigen::Vector2d::Ones();
  supportMax_ = -supportMin_;
  supportPolygons_.clear();

  for(const auto & contact : contacts)
  {
    addContact(contact.first, contact.second);
  }
}

void StabilizerTask::addContact(ContactState contactState, const Contact & contact)
{
  auto footTask = footTasks[contactState];
  // Use real robot's surface pose as contact
  contacts_.emplace(std::make_pair(contactState, contact));

  supportMin_.x() = std::min(contact.xmin(), supportMin_.x());
  supportMin_.y() = std::min(contact.ymin(), supportMin_.y());
  supportMax_.x() = std::max(contact.xmax(), supportMax_.x());
  supportMax_.y() = std::max(contact.ymax(), supportMax_.y());

  supportPolygons_.push_back(contact.polygon());

  // Configure support foot task
  footTask->reset();
  footTask->weight(c_.contactWeight);
  footTask->targetPose(contact.surfacePose());

  addContacts_.push_back(contactState);
}

void StabilizerTask::setSupportFootGains()
{
  if(inDoubleSupport())
  {
    for(auto contactT : contactTasks)
    {
      contactT->admittance(contactAdmittance());
      contactT->setGains(c_.contactStiffness, c_.contactDamping);
    }
  }
  else
  {
    sva::MotionVecd vdcContactStiffness = {c_.contactStiffness.angular(),
                                           {c_.vdcStiffness, c_.vdcStiffness, c_.vdcStiffness}};
    for(auto contactT : contactTasks)
    {
      contactT->admittance(contactAdmittance());
      contactT->setGains(vdcContactStiffness, c_.contactDamping);
    }
  }
}

void StabilizerTask::checkInTheAir()
{
  inTheAir_ = true;
  for(const auto footT : footTasks)
  {
    inTheAir_ = inTheAir_ && footT.second->measuredWrench().force().z() < c_.safetyThresholds.MIN_DS_PRESSURE;
  }
}

void StabilizerTask::computeLeftFootRatio()
{
  if(inDoubleSupport())
  {
    // Project desired ZMP in-between foot-sole ankle frames and compute ratio along the line in-beween the two surfaces
    const Eigen::Vector3d & lankle = contacts_.at(ContactState::Left).anklePose().translation();
    const Eigen::Vector3d & rankle = contacts_.at(ContactState::Right).anklePose().translation();
    Eigen::Vector3d t_lankle_com = zmpTarget_ - lankle;
    Eigen::Vector3d t_lankle_rankle = rankle - lankle;
    double d_proj = t_lankle_com.dot(t_lankle_rankle.normalized());
    leftFootRatio_ = clamp(d_proj / t_lankle_rankle.norm(), 0., 1.);
  }
  else if(inContact(ContactState::Left))
  {
    leftFootRatio_ = 0;
  }
  else
  {
    leftFootRatio_ = 1;
  }
}

sva::PTransformd StabilizerTask::anchorFrame(const mc_rbdyn::Robot & robot) const
{
  return sva::interpolate(robot.surfacePose(footTasks.at(ContactState::Left)->surface()),
                          robot.surfacePose(footTasks.at(ContactState::Right)->surface()), leftFootRatio_);
}

void StabilizerTask::updateZMPFrame()
{
  if(inDoubleSupport())
  {
    zmpFrame_ = sva::interpolate(contacts_.at(ContactState::Left).surfacePose(),
                                 contacts_.at(ContactState::Right).surfacePose(), 0.5);
  }
  else if(inContact(ContactState::Left))
  {
    zmpFrame_ = contacts_.at(ContactState::Left).surfacePose();
  }
  else
  {
    zmpFrame_ = contacts_.at(ContactState::Right).surfacePose();
  }
}

void StabilizerTask::staticTarget(const Eigen::Vector3d & com, double zmpHeight)
{
  Eigen::Vector3d zmp = Eigen::Vector3d{com.x(), com.y(), zmpHeight};

  target(com, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), zmp);
}

void StabilizerTask::target(const Eigen::Vector3d & com,
                            const Eigen::Vector3d & comd,
                            const Eigen::Vector3d & comdd,
                            const Eigen::Vector3d & zmp,
                            const Eigen::Vector3d & zmpd)
{
  comTargetRaw_ = com;
  comTarget_ = comTargetRaw_ - comOffsetErrCoM_;
  if(c_.extWrench.addExpectedCoMOffset)
  {
    comTarget_ -= comOffsetTarget_;
  }
  comdTarget_ = comd;
  comddTarget_ = comdd;
  zmpTarget_ = zmp;
  zmpdTarget_ = zmpd;
  double comHeight = comTarget_.z() - zmpTarget_.z();
  omega_ = std::sqrt(constants::gravity.z() / comHeight);
  dcmTarget_ = comTarget_ + comdTarget_ / omega_;
}

void StabilizerTask::setExternalWrenches(const std::vector<std::string> & surfaceNames,
                                         const std::vector<sva::ForceVecd> & targetWrenches,
                                         const std::vector<sva::MotionVecd> & gains)
{
  if(surfaceNames.size() > 0
     && !(c_.extWrench.addExpectedCoMOffset || c_.extWrench.modifyCoMErr || c_.extWrench.modifyZMPErr
          || c_.extWrench.modifyZMPErrD))
  {
    mc_rtc::log::warning(
        "[StabilizerTask] external wrenches are set, but the configurations for handling them are invalid.");
  }

  extWrenches_.clear();
  for(unsigned int i = 0; i < surfaceNames.size(); i++)
  {
    extWrenches_.push_back({targetWrenches[i], sva::ForceVecd::Zero(), gains[i], surfaceNames[i]});
    if(!robot().surfaceHasIndirectForceSensor(surfaceNames[i]))
    {
      mc_rtc::log::warning(
          "[StabilizerTask] surface {} does not have force sensor. The target force is used as the measured force.",
          surfaceNames[i]);
    }
  }

  comOffsetTarget_ = computeCoMOffset<&ExternalWrench::target>(robot());
  comTarget_ = comTargetRaw_ - comOffsetErrCoM_;
  if(c_.extWrench.addExpectedCoMOffset)
  {
    comTarget_ -= comOffsetTarget_;
  }
}

template<sva::ForceVecd StabilizerTask::ExternalWrench::*TargetOrMeasured>
Eigen::Vector3d StabilizerTask::computeCoMOffset(const mc_rbdyn::Robot & robot) const
{
  Eigen::Vector3d comOffset = Eigen::Vector3d::Zero();
  Eigen::Vector3d pos, force, moment;
  for(const auto & extWrench : extWrenches_)
  {
    computeExternalContact(robot, extWrench.surfaceName, extWrench.*TargetOrMeasured, pos, force, moment);

    comOffset.x() += (pos.z() - zmpTarget_.z()) * force.x() - pos.x() * force.z() + moment.y();
    comOffset.y() += (pos.z() - zmpTarget_.z()) * force.y() - pos.y() * force.z() - moment.x();
  }
  double verticalComAcc = comddTarget_.z() + constants::gravity.z();
  double verticalComAccThre = 1e-3;
  if(std::abs(verticalComAcc) < verticalComAccThre)
  {
    mc_rtc::log::warning(
        "[StabilizerTask::computeCoMOffset] overwrite verticalComAcc because it's too close to zero: {}",
        verticalComAcc);
    verticalComAcc = verticalComAcc >= 0 ? verticalComAccThre : -verticalComAccThre;
  }
  comOffset /= robot.mass() * verticalComAcc;

  return comOffset;
}

template<sva::ForceVecd StabilizerTask::ExternalWrench::*TargetOrMeasured>
sva::ForceVecd StabilizerTask::computeExternalWrenchSum(const mc_rbdyn::Robot & robot,
                                                        const Eigen::Vector3d & com) const
{
  sva::ForceVecd extWrenchSum = sva::ForceVecd::Zero();
  Eigen::Vector3d pos, force, moment;
  for(const auto & extWrench : extWrenches_)
  {
    computeExternalContact(robot, extWrench.surfaceName, extWrench.*TargetOrMeasured, pos, force, moment);

    extWrenchSum.force() += force;
    extWrenchSum.moment() += (pos - com).cross(force) + moment;
  }

  return extWrenchSum;
}

void StabilizerTask::computeExternalContact(const mc_rbdyn::Robot & robot,
                                            const std::string & surfaceName,
                                            const sva::ForceVecd & surfaceWrench,
                                            Eigen::Vector3d & pos,
                                            Eigen::Vector3d & force,
                                            Eigen::Vector3d & moment) const
{
  sva::PTransformd surfacePose = robot.surfacePose(surfaceName);
  sva::PTransformd T_s_0(Eigen::Matrix3d(surfacePose.rotation().transpose()));
  // Represent the surface wrench in the frame whose position is same with the surface frame and orientation is same
  // with the world frame
  sva::ForceVecd surfaceWrenchW = T_s_0.dualMul(surfaceWrench);
  pos = surfacePose.translation();
  force = surfaceWrenchW.force();
  moment = surfaceWrenchW.moment();
}

void StabilizerTask::run()
{
  using namespace std::chrono;
  using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                          std::chrono::high_resolution_clock, std::chrono::steady_clock>::type;
  auto startTime = clock::now();

  c_.clampGains();
  checkInTheAir();
  computeLeftFootRatio();
  setSupportFootGains();
  updateZMPFrame();
  if(!inTheAir_)
  {
    measuredNetWrench_ = robots_.robot(robotIndex_).netWrench(contactSensors);
    try
    {
      measuredZMP_ =
          robots_.robot(robotIndex_).zmp(measuredNetWrench_, zmpFrame_, c_.safetyThresholds.MIN_NET_TOTAL_FORCE_ZMP);
    }
    catch(std::runtime_error & e)
    {
      mc_rtc::log::error("[{}] ZMP computation failed, keeping previous value {}", name(), measuredZMP_.transpose());
    }
  }
  else
  {
    measuredNetWrench_ = sva::ForceVecd::Zero();
  }
  desiredWrench_ = computeDesiredWrench();

  if(inDoubleSupport())
  {
    distributeWrench(desiredWrench_);
  }
  else if(inContact(ContactState::Left))
  {
    saturateWrench(desiredWrench_, footTasks[ContactState::Left], contacts_.at(ContactState::Left));
    footTasks[ContactState::Right]->setZeroTargetWrench();
  }
  else
  {
    saturateWrench(desiredWrench_, footTasks[ContactState::Right], contacts_.at(ContactState::Right));
    footTasks[ContactState::Left]->setZeroTargetWrench();
  }

  distribZMP_ = mc_rbdyn::zmp(distribWrench_, zmpFrame_);
  updateCoMTaskZMPCC();
  updateFootForceDifferenceControl();

  comTask->com(comTarget_);
  comTask->refVel(comdTarget_);
  comTask->refAccel(comddTarget_);

  // Update orientation tasks according to feet orientation
  sva::PTransformd X_0_a = anchorFrame(robot());
  Eigen::Matrix3d pelvisOrientation = X_0_a.rotation();
  pelvisTask->orientation(pelvisOrientation);
  torsoTask->orientation(mc_rbdyn::rpyToMat({0, c_.torsoPitch, 0}) * pelvisOrientation);

  auto endTime = clock::now();
  runTime_ = 1000. * duration_cast<duration<double>>(endTime - startTime).count();
}

void StabilizerTask::updateState(const Eigen::Vector3d & com, const Eigen::Vector3d & comd)
{
  measuredCoM_ = com;
  measuredCoMd_ = comd;
  measuredDCM_ = measuredCoM_ + measuredCoMd_ / omega_;
}

sva::ForceVecd StabilizerTask::computeDesiredWrench()
{
  Eigen::Vector3d comError = comTarget_ - measuredCoM_;
  Eigen::Vector3d comdError = comdTarget_ - measuredCoMd_;
  dcmError_ = comError + comdError / omega_;
  dcmError_.z() = 0.;

  if(inTheAir_)
  {
    dcmDerivator_.reset(Eigen::Vector3d::Zero());
    dcmIntegrator_.append(Eigen::Vector3d::Zero());
    dcmEstimatorNeedsReset_ = true;
  }
  else
  {
    Eigen::Vector3d zmpError = zmpTarget_ - measuredZMP_;
    zmpError.z() = 0.;

    if(c_.dcmBias.withDCMBias)
    {
      if(omega_ != dcmEstimator_.getLipmNaturalFrequency())
      {
        dcmEstimator_.setLipmNaturalFrequency(omega_);
      }

      const Eigen::Matrix3d & waistOrientation = robot().posW().rotation().transpose();

      if(dcmEstimatorNeedsReset_)
      {
        dcmEstimator_.resetWithMeasurements(dcmError_.head<2>(), zmpError.head<2>(), waistOrientation, true);
        dcmEstimatorNeedsReset_ = false;
      }
      else
      {
        dcmEstimator_.setInputs(dcmError_.head<2>(), zmpError.head<2>(), waistOrientation);
      }

      /// run the estimation
      dcmEstimator_.update();

      if(c_.dcmBias.withDCMFilter)
      {
        /// the estimators provides a filtered DCM
        dcmError_.head<2>() = dcmEstimator_.getUnbiasedDCM();
      }
      else
      {
        dcmError_.head<2>() -= dcmEstimator_.getBias();
      }
      /// the bias should also correct the CoM
      comError.head<2>() -= dcmEstimator_.getBias();
      /// the unbiased dcm allows also to get the velocity of the CoM
      comdError.head<2>() = omega_ * (dcmError_.head<2>() - comError.head<2>());
      measuredDCMUnbiased_ = dcmTarget_ - dcmError_;
    }
    else
    {
      measuredDCMUnbiased_ = measuredDCM_;
      dcmEstimatorNeedsReset_ = true;
    }

    dcmDerivator_.update(omega_ * (dcmError_ - zmpError));
    dcmIntegrator_.append(dcmError_);
  }
  dcmAverageError_ = dcmIntegrator_.eval();
  dcmVelError_ = dcmDerivator_.eval();

  Eigen::Vector3d desiredCoMAccel = comddTarget_;
  desiredCoMAccel += omega_ * (c_.dcmPropGain * dcmError_ + c_.comdErrorGain * comdError);
  desiredCoMAccel += omega_ * c_.dcmIntegralGain * dcmAverageError_;
  desiredCoMAccel += omega_ * c_.dcmDerivGain * dcmVelError_;
  desiredCoMAccel -= omega_ * omega_ * c_.zmpdGain * zmpdTarget_;

  // Calculate CoM offset from measured wrench
  for(auto & extWrench : extWrenches_)
  {
    if(robot().surfaceHasIndirectForceSensor(extWrench.surfaceName))
    {
      extWrench.measured =
          sva::ForceVecd(extWrench.gain.vector().cwiseProduct(robot().surfaceWrench(extWrench.surfaceName).vector()));
    }
    else
    {
      extWrench.measured = extWrench.target;
    }
  }
  comOffsetMeasured_ = computeCoMOffset<&ExternalWrench::measured>(realRobot());

  // Modify the desired CoM and ZMP depending on the external wrench error
  comOffsetLowPass_.update(comOffsetMeasured_ - comOffsetTarget_);
  comOffsetErr_ = comOffsetLowPass_.eval();
  if(c_.extWrench.modifyCoMErr)
  {
    comOffsetLowPassCoM_.update(comOffsetErr_);
  }
  else
  {
    comOffsetLowPassCoM_.update(Eigen::Vector3d::Zero());
  }
  comOffsetErrCoM_ = comOffsetLowPassCoM_.eval();
  comOffsetErrZMP_ = comOffsetErr_ - comOffsetErrCoM_;
  clampInPlaceAndWarn(comOffsetErrCoM_, Eigen::Vector3d::Constant(-c_.extWrench.comOffsetErrCoMLimit).eval(),
                      Eigen::Vector3d::Constant(c_.extWrench.comOffsetErrCoMLimit).eval(), "comOffsetErrCoM");
  clampInPlaceAndWarn(comOffsetErrZMP_, Eigen::Vector3d::Constant(-c_.extWrench.comOffsetErrZMPLimit).eval(),
                      Eigen::Vector3d::Constant(c_.extWrench.comOffsetErrZMPLimit).eval(), "comOffsetErrZMP");
  comOffsetDerivator_.update(comOffsetErr_);
  if(c_.extWrench.modifyZMPErr)
  {
    desiredCoMAccel -= omega_ * omega_ * comOffsetErrZMP_;
  }
  if(c_.extWrench.modifyZMPErrD)
  {
    desiredCoMAccel -= (omega_ * omega_ / c_.zmpdGain) * comOffsetDerivator_.eval();
  }

  // Calculate the desired force and moment
  Eigen::Vector3d desiredForce = mass_ * (desiredCoMAccel + constants::gravity);
  Eigen::Vector3d desiredMoment = Eigen::Vector3d::Zero();

  // Subtract the external wrenches from the desired force and moment
  extWrenchSumTarget_ = computeExternalWrenchSum<&ExternalWrench::target>(robot(), comTarget_);
  extWrenchSumLowPass_.update(computeExternalWrenchSum<&ExternalWrench::measured>(realRobot(), measuredCoM_));
  extWrenchSumMeasured_ = extWrenchSumLowPass_.eval();
  if(c_.extWrench.subtractMeasuredValue)
  {
    desiredForce -= extWrenchSumMeasured_.force();
    desiredMoment -= extWrenchSumMeasured_.moment();
  }
  else
  {
    desiredForce -= extWrenchSumTarget_.force();
    desiredMoment -= extWrenchSumTarget_.moment();
  }

  // Previous implementation (up to v1.3):
  // return {pendulum_.com().cross(desiredForce), desiredForce};
  // See https://github.com/stephane-caron/lipm_walking_controller/issues/28
  return {measuredCoM_.cross(desiredForce) + desiredMoment, desiredForce};
}

void StabilizerTask::distributeWrench(const sva::ForceVecd & desiredWrench)
{
  // Variables
  // ---------
  // x = [w_l_0 w_r_0] where
  // w_l_0: spatial force vector of left foot contact in inertial frame
  // w_r_0: spatial force vector of right foot contact in inertial frame
  //
  // Objective
  // ---------
  // Weighted minimization of the following tasks:
  // w_l_0 + w_r_0 == desiredWrench  -- realize desired contact wrench
  // w_l_lankle == 0 -- minimize left foot ankle torque (anisotropic weight)
  // w_r_rankle == 0 -- minimize right foot ankle torque (anisotropic weight)
  // (1 - lfr) * w_l_lc.z() == lfr * w_r_rc.z()
  //
  // Constraints
  // -----------
  // CWC X_0_lc* w_l_0 <= 0  -- left foot wrench within contact wrench cone
  // CWC X_0_rc* w_r_0 <= 0  -- right foot wrench within contact wrench cone
  // (X_0_lc* w_l_0).z() > minPressure  -- minimum left foot contact pressure
  // (X_0_rc* w_r_0).z() > minPressure  -- minimum right foot contact pressure

  const auto & leftContact = contacts_.at(ContactState::Left);
  const auto & rightContact = contacts_.at(ContactState::Right);
  const sva::PTransformd & X_0_lc = leftContact.surfacePose();
  const sva::PTransformd & X_0_rc = rightContact.surfacePose();
  const sva::PTransformd & X_0_lankle = leftContact.anklePose();
  const sva::PTransformd & X_0_rankle = rightContact.anklePose();

  constexpr unsigned NB_VAR = 6 + 6;
  constexpr unsigned COST_DIM = 6 + NB_VAR + 1;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  A.setZero(COST_DIM, NB_VAR);
  b.setZero(COST_DIM);

  // |w_l_0 + w_r_0 - desiredWrench|^2
  auto A_net = A.block<6, 12>(0, 0);
  auto b_net = b.segment<6>(0);
  A_net.block<6, 6>(0, 0) = Eigen::Matrix6d::Identity();
  A_net.block<6, 6>(0, 6) = Eigen::Matrix6d::Identity();
  b_net = desiredWrench.vector();

  // |ankle torques|^2
  auto A_lankle = A.block<6, 6>(6, 0);
  auto A_rankle = A.block<6, 6>(12, 6);
  // anisotropic weights:  taux, tauy, tauz,   fx,   fy,   fz;
  A_lankle.diagonal() << 1., 1., 1e-4, 1e-3, 1e-3, 1e-4;
  A_rankle.diagonal() << 1., 1., 1e-4, 1e-3, 1e-3, 1e-4;
  A_lankle *= X_0_lankle.dualMatrix();
  A_rankle *= X_0_rankle.dualMatrix();

  // |(1 - lfr) * w_l_lc.force().z() - lfr * w_r_rc.force().z()|^2
  double lfr = leftFootRatio_;
  auto A_pressure = A.block<1, 12>(18, 0);
  A_pressure.block<1, 6>(0, 0) = (1 - lfr) * X_0_lc.dualMatrix().bottomRows<1>();
  A_pressure.block<1, 6>(0, 6) = -lfr * X_0_rc.dualMatrix().bottomRows<1>();

  // Apply weights
  A_net *= c_.fdqpWeights.netWrenchSqrt;
  b_net *= c_.fdqpWeights.netWrenchSqrt;
  A_lankle *= c_.fdqpWeights.ankleTorqueSqrt;
  A_rankle *= c_.fdqpWeights.ankleTorqueSqrt;
  // b_lankle = 0
  // b_rankle = 0
  A_pressure *= c_.fdqpWeights.pressureSqrt;
  // b_pressure = 0

  Eigen::MatrixXd Q = A.transpose() * A;
  Eigen::VectorXd c = -A.transpose() * b;

  constexpr unsigned NB_CONS = 16 + 16 + 2;
  Eigen::Matrix<double, NB_CONS, NB_VAR> A_ineq;
  Eigen::VectorXd b_ineq;
  A_ineq.setZero(NB_CONS, NB_VAR);
  b_ineq.setZero(NB_CONS);
  // CWC * w_l_lc <= 0
  A_ineq.block<16, 6>(0, 0) = leftContact.wrenchFaceMatrix() * X_0_lc.dualMatrix();
  // b_ineq.segment<16>(0) is already zero
  // CWC * w_r_rc <= 0
  A_ineq.block<16, 6>(16, 6) = rightContact.wrenchFaceMatrix() * X_0_rc.dualMatrix();
  // b_ineq.segment<16>(16) is already zero
  // w_l_lc.force().z() >= MIN_DS_PRESSURE
  A_ineq.block<1, 6>(32, 0) = -X_0_lc.dualMatrix().bottomRows<1>();
  b_ineq(32) = -c_.safetyThresholds.MIN_DS_PRESSURE;
  // w_r_rc.force().z() >= MIN_DS_PRESSURE
  A_ineq.block<1, 6>(33, 6) = -X_0_rc.dualMatrix().bottomRows<1>();
  b_ineq(33) = -c_.safetyThresholds.MIN_DS_PRESSURE;

  qpSolver_.problem(NB_VAR, 0, NB_CONS);
  Eigen::MatrixXd A_eq(0, 0);
  Eigen::VectorXd b_eq;
  b_eq.resize(0);

  bool solutionFound = qpSolver_.solve(Q, c, A_eq, b_eq, A_ineq, b_ineq, /* isDecomp = */ false);
  if(!solutionFound)
  {
    mc_rtc::log::error("[StabilizerTask] DS force distribution QP: solver found no solution");
    return;
  }

  Eigen::VectorXd x = qpSolver_.result();
  sva::ForceVecd w_l_0(x.segment<3>(0), x.segment<3>(3));
  sva::ForceVecd w_r_0(x.segment<3>(6), x.segment<3>(9));
  distribWrench_ = w_l_0 + w_r_0;

  sva::ForceVecd w_l_lc = X_0_lc.dualMul(w_l_0);
  sva::ForceVecd w_r_rc = X_0_rc.dualMul(w_r_0);
  Eigen::Vector2d leftCoP = (constants::vertical.cross(w_l_lc.couple()) / w_l_lc.force()(2)).head<2>();
  Eigen::Vector2d rightCoP = (constants::vertical.cross(w_r_rc.couple()) / w_r_rc.force()(2)).head<2>();
  footTasks[ContactState::Left]->targetCoP(leftCoP);
  footTasks[ContactState::Left]->targetForce(w_l_lc.force());
  footTasks[ContactState::Right]->targetCoP(rightCoP);
  footTasks[ContactState::Right]->targetForce(w_r_rc.force());
}

void StabilizerTask::saturateWrench(const sva::ForceVecd & desiredWrench,
                                    std::shared_ptr<mc_tasks::force::CoPTask> & footTask,
                                    const Contact & contact)
{
  constexpr unsigned NB_CONS = 16;
  constexpr unsigned NB_VAR = 6;

  // Variables
  // ---------
  // x = [w_0] where
  // w_0: spatial force vector of foot contact in inertial frame
  //
  // Objective
  // ---------
  // weighted minimization of |w_c - X_0_c* desiredWrench|^2
  //
  // Constraints
  // -----------
  // F X_0_c* w_0 <= 0    -- contact stability

  const sva::PTransformd & X_0_c = contact.surfacePose();

  Eigen::Matrix6d Q = Eigen::Matrix6d::Identity();
  Eigen::Vector6d c = -desiredWrench.vector();

  Eigen::MatrixXd A_ineq = contact.wrenchFaceMatrix() * X_0_c.dualMatrix();
  Eigen::VectorXd b_ineq;
  b_ineq.setZero(NB_CONS);

  qpSolver_.problem(NB_VAR, 0, NB_CONS);
  Eigen::MatrixXd A_eq(0, 0);
  Eigen::VectorXd b_eq;
  b_eq.resize(0);

  bool solutionFound = qpSolver_.solve(Q, c, A_eq, b_eq, A_ineq, b_ineq, /* isDecomp = */ true);
  if(!solutionFound)
  {
    mc_rtc::log::error("[StabilizerTask] SS force distribution QP: solver found no solution");
    return;
  }

  Eigen::VectorXd x = qpSolver_.result();
  sva::ForceVecd w_0(x.head<3>(), x.tail<3>());
  sva::ForceVecd w_c = X_0_c.dualMul(w_0);
  Eigen::Vector2d cop = (constants::vertical.cross(w_c.couple()) / w_c.force()(2)).head<2>();
  footTask->targetCoP(cop);
  footTask->targetForce(w_c.force());
  distribWrench_ = w_0;
}

void StabilizerTask::updateCoMTaskZMPCC()
{
  c_.zmpcc = zmpcc_.config();
  if(inTheAir_ || (zmpccOnlyDS_ && !inDoubleSupport()))
  {
    zmpcc_.enabled(false); // Leak to zero
  }
  else
  {
    zmpcc_.configure(c_.zmpcc);
    zmpcc_.enabled(enabled_);
    zmpcc_.update(distribZMP_, measuredZMP_, zmpFrame_, dt_);
  }
  zmpcc_.apply(comTarget_, comdTarget_, comddTarget_);
}

void StabilizerTask::updateFootForceDifferenceControl()
{
  auto leftFootTask = footTasks[ContactState::Left];
  auto rightFootTask = footTasks[ContactState::Right];
  if(!inDoubleSupport() || inTheAir_)
  {
    dfzForceError_ = 0.;
    dfzHeightError_ = 0.;
    vdcHeightError_ = 0.;
    leftFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});
    rightFootTask->refVelB({{0., 0., 0.}, {0., 0., 0.}});
    return;
  }

  double LFz_d = leftFootTask->targetWrench().force().z();
  double RFz_d = rightFootTask->targetWrench().force().z();
  double LFz = leftFootTask->measuredWrench().force().z();
  double RFz = rightFootTask->measuredWrench().force().z();
  dfzForceError_ = (LFz_d - RFz_d) - (LFz - RFz);

  double LTz_d = leftFootTask->targetPose().translation().z();
  double RTz_d = rightFootTask->targetPose().translation().z();
  double LTz = leftFootTask->surfacePose().translation().z();
  double RTz = rightFootTask->surfacePose().translation().z();
  dfzHeightError_ = (LTz_d - RTz_d) - (LTz - RTz);
  vdcHeightError_ = (LTz_d + RTz_d) - (LTz + RTz);

  double dz_ctrl = c_.dfzAdmittance * dfzForceError_ - c_.dfzDamping * dfzHeightError_;
  double dz_vdc = c_.vdcFrequency * vdcHeightError_;
  sva::MotionVecd velF = {{0., 0., 0.}, {0., 0., dz_ctrl}};
  sva::MotionVecd velT = {{0., 0., 0.}, {0., 0., dz_vdc}};
  leftFootTask->refVelB(0.5 * (velT - velF));
  rightFootTask->refVelB(0.5 * (velT + velF));
}

template Eigen::Vector3d StabilizerTask::computeCoMOffset<&StabilizerTask::ExternalWrench::target>(
    const mc_rbdyn::Robot &) const;
template Eigen::Vector3d StabilizerTask::computeCoMOffset<&StabilizerTask::ExternalWrench::measured>(
    const mc_rbdyn::Robot &) const;

template sva::ForceVecd StabilizerTask::computeExternalWrenchSum<&StabilizerTask::ExternalWrench::target>(
    const mc_rbdyn::Robot &,
    const Eigen::Vector3d &) const;
template sva::ForceVecd StabilizerTask::computeExternalWrenchSum<&StabilizerTask::ExternalWrench::measured>(
    const mc_rbdyn::Robot &,
    const Eigen::Vector3d &) const;

} // namespace lipm_stabilizer
} // namespace mc_tasks

namespace
{
static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "lipm_stabilizer",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      unsigned robotIndex = robotIndexFromConfig(config, solver.robots(), "lipm_stabilizer");
      const auto & robot = solver.robots().robot(robotIndex);

      // Load default configuration from robot module
      auto stabiConf = robot.module().defaultLIPMStabilizerConfiguration();
      // Load user-specified configuration
      stabiConf.load(config);
      // Load user-specified stabilizer configuration for this robot
      if(config.has(robot.name()))
      {
        stabiConf.load(config(robot.name()));
      }

      auto t = std::allocate_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
          Eigen::aligned_allocator<mc_tasks::lipm_stabilizer::StabilizerTask>{}, solver.robots(), solver.realRobots(),
          robotIndex, stabiConf.leftFootSurface, stabiConf.rightFootSurface, stabiConf.torsoBodyName, solver.dt());
      t->configure(stabiConf);
      t->load(solver, config);
      t->reset();
      return t;
    });
}
