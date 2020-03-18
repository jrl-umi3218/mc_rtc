/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#include <mc_filter/utils/clamp.h>
#include <mc_rbdyn/ZMP.h>
#include <mc_rbdyn/rpy_utils.h>
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

// Repeat static constexpr declarations
// Fixes https://github.com/stephane-caron/lipm_walking_controller/issues/21
// See also https://stackoverflow.com/q/8016780
constexpr double StabilizerTask::MAX_AVERAGE_DCM_ERROR;
constexpr double StabilizerTask::MAX_COP_ADMITTANCE;
constexpr double StabilizerTask::MAX_DCM_D_GAIN;
constexpr double StabilizerTask::MAX_DCM_I_GAIN;
constexpr double StabilizerTask::MAX_DCM_P_GAIN;
constexpr double StabilizerTask::MAX_DFZ_ADMITTANCE;
constexpr double StabilizerTask::MAX_DFZ_DAMPING;
constexpr double StabilizerTask::MAX_FDC_RX_VEL;
constexpr double StabilizerTask::MAX_FDC_RY_VEL;
constexpr double StabilizerTask::MAX_FDC_RZ_VEL;
constexpr double StabilizerTask::MIN_DS_PRESSURE;
constexpr double StabilizerTask::MIN_NET_TOTAL_FORCE_ZMP;

StabilizerTask::StabilizerTask(const mc_rbdyn::Robots & robots,
                               const mc_rbdyn::Robots & realRobots,
                               unsigned int robotIndex,
                               const std::string & leftSurface,
                               const std::string & rightSurface,
                               const std::string & torsoBodyName,
                               double dt)
: robots_(robots), realRobots_(realRobots), robotIndex_(robotIndex), dcmIntegrator_(dt, /* timeConstant = */ 15.),
  dcmDerivator_(dt, /* timeConstant = */ 1.), dt_(dt), mass_(robots.robot(robotIndex).mass())
{
  type_ = "lipm_stabilizer";
  name_ = type_ + "_" + robots.robot(robotIndex).name();

  comTask.reset(new mc_tasks::CoMTask(robots, robotIndex_));
  auto leftCoP = std::make_shared<mc_tasks::force::CoPTask>(leftSurface, robots, robotIndex_);
  auto rightCoP = std::make_shared<mc_tasks::force::CoPTask>(rightSurface, robots, robotIndex_);
  footTasks[ContactState::Left] = leftCoP;
  footTasks[ContactState::Right] = rightCoP;

  std::string pelvisBodyName = robot().mb().body(0).name();
  pelvisTask = std::make_shared<mc_tasks::OrientationTask>(pelvisBodyName, robots_, robotIndex_);
  torsoTask = std::make_shared<mc_tasks::OrientationTask>(torsoBodyName, robots_, robotIndex_);

  // Rename the tasks managed by the stabilizer
  // Doing so helps making the logs more consistent, and having a fixed name
  // allows for predifined custom plots in the log ui.
  const auto n = name_ + "_Tasks";
  comTask->name(n + "_com");
  leftCoP->name(n + "_cop_left");
  rightCoP->name(n + "_cop_right");
  pelvisTask->name(n + "_pelvis");
  torsoTask->name(n + "_torso");
}

void StabilizerTask::reset()
{
  t_ = 0;
  configure(robot().module().defaultLIPMStabilizerConfiguration());
  commitConfig();
  comTask->reset();
  comTarget_ = comTask->com();

  for(auto footTask : footTasks)
  {
    footTask.second->reset();
    footTask.second->maxAngularVel({MAX_FDC_RX_VEL, MAX_FDC_RY_VEL, MAX_FDC_RZ_VEL});
  }

  pelvisTask->reset();
  torsoTask->reset();

  Eigen::Vector3d staticForce = mass_ * constants::gravity;

  dcmAverageError_ = Eigen::Vector3d::Zero();
  dcmError_ = Eigen::Vector3d::Zero();
  dcmVelError_ = Eigen::Vector3d::Zero();
  dfzForceError_ = 0.;
  dfzHeightError_ = 0.;
  distribWrench_ = {comTarget_.cross(staticForce), staticForce};
  vdcHeightError_ = 0.;

  zmpcc_.reset();

  dcmDerivator_.reset(Eigen::Vector3d::Zero());
  dcmIntegrator_.saturation(MAX_AVERAGE_DCM_ERROR);
  dcmIntegrator_.reset(Eigen::Vector3d::Zero());

  omega_ = std::sqrt(constants::gravity.z() / robot().com().z());
}

void StabilizerTask::dimWeight(const Eigen::VectorXd & /* dim */)
{
  LOG_ERROR_AND_THROW(std::runtime_error, "dimWeight not implemented for task " << type_);
}

Eigen::VectorXd StabilizerTask::dimWeight() const
{
  LOG_ERROR_AND_THROW(std::runtime_error, "dimWeight not implemented for task " << type_);
}

void StabilizerTask::selectActiveJoints(mc_solver::QPSolver & /* solver */,
                                        const std::vector<std::string> & /* activeJointsName */,
                                        const std::map<std::string, std::vector<std::array<int, 2>>> & /* activeDofs */)
{
  LOG_ERROR_AND_THROW(std::runtime_error, "Task " << name_
                                                  << " does not implement selectActiveJoints. Please configure it "
                                                     "through the stabilizer configuration instead");
}

void StabilizerTask::selectUnactiveJoints(
    mc_solver::QPSolver & /* solver */,
    const std::vector<std::string> & /* unactiveJointsName */,
    const std::map<std::string, std::vector<std::array<int, 2>>> & /* unactiveDofs */)
{
  LOG_ERROR_AND_THROW(std::runtime_error, "Task " << name_
                                                  << " does not implement selectUnactiveJoints. Please configure it "
                                                     "through the stabilizer configuration instead.");
}

void StabilizerTask::resetJointsSelector(mc_solver::QPSolver & /* solver */)
{
  LOG_ERROR_AND_THROW(std::runtime_error, "Task " << name_
                                                  << " does not implement resetJointsSelector. Please configure it "
                                                     "through the stabilizer configuration instead.");
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
  res.head(3) = comTask->eval();
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
    for(const auto contactT : contactTasks)
    {
      LOG_INFO(name() + ": Removing contact " << contactT->surface());
      MetaTask::removeFromLogger(*contactT, *solver.logger());
      MetaTask::removeFromSolver(*contactT, solver);
    }
    contactTasks.clear();
    contactSensors.clear();

    // Add new contacts
    for(const auto contactState : addContacts_)
    {
      auto footTask = footTasks[contactState];
      LOG_INFO(name() + ": Adding contact " << footTask->surface());
      MetaTask::addToSolver(*footTask, solver);
      MetaTask::addToLogger(*footTask, *solver.logger());
      contactTasks.push_back(footTask);
      const auto & bodyName = robot().surface(footTask->surface()).bodyName();
      const auto & fs = robot().bodyForceSensor(bodyName);
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
    c_ = lastConfig_;
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
  LOG_INFO("[StabilizerTask] enabled");
  // Reset DCM integrator when enabling the stabilizer.
  // While idle, it will accumulate a lot of error, and would case the robot to
  // move suddently to compensate it otherwise
  dcmIntegrator_.reset(Eigen::Vector3d::Zero());
  dcmDerivator_.reset(Eigen::Vector3d::Zero());

  configure(lastConfig_);
  zmpcc_.enabled(true);
  enabled_ = true;
}

void StabilizerTask::disable()
{
  LOG_INFO("[StabilizerTask] disabled");
  // Save current configuration to be reused when re-enabling
  lastConfig_ = c_;
  // Set the stabilizer gains to zero
  c_.copAdmittance.setZero();
  c_.dcmDerivGain = 0.;
  c_.dcmIntegralGain = 0.;
  c_.dcmPropGain = 0.;
  c_.dfzAdmittance = 0.;
  c_.vdcFrequency = 0.;
  c_.vdcStiffness = 0.;
  zmpcc_.enabled(false);
  enabled_ = false;
}

void StabilizerTask::reconfigure()
{
  LOG_INFO("[StabilizerTask] reconfigured to the last commited configuration");
  configure(defaultConfig_);
  enable();
}

void StabilizerTask::configure(const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & config)
{
  lastConfig_ = config;
  c_ = config;
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
  reconfigure_ = false;
}

void StabilizerTask::load(mc_solver::QPSolver &, const mc_rtc::Configuration & config)
{
  double height = 0;
  // Load contacts
  std::vector<std::pair<ContactState, Contact>> contactsToAdd;
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

const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & StabilizerTask::config() const
{
  return c_;
}

const mc_rbdyn::lipm_stabilizer::StabilizerConfiguration & StabilizerTask::commitedConfig() const
{
  return defaultConfig_;
}

void StabilizerTask::checkGains()
{
  clampInPlaceAndWarn(c_.copAdmittance.x(), 0., MAX_COP_ADMITTANCE, "CoP x-admittance");
  clampInPlaceAndWarn(c_.copAdmittance.y(), 0., MAX_COP_ADMITTANCE, "CoP y-admittance");
  clampInPlaceAndWarn(c_.dcmDerivGain, 0., MAX_DCM_D_GAIN, "DCM deriv x-gain");
  clampInPlaceAndWarn(c_.dcmIntegralGain, 0., MAX_DCM_I_GAIN, "DCM integral x-gain");
  clampInPlaceAndWarn(c_.dcmPropGain, 0., MAX_DCM_P_GAIN, "DCM prop x-gain");
  clampInPlaceAndWarn(c_.dfzAdmittance, 0., MAX_DFZ_ADMITTANCE, "DFz admittance");
}

void StabilizerTask::setContacts(const std::vector<ContactState> & contacts)
{
  std::vector<std::pair<ContactState, Contact>> addContacts;
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
  std::vector<std::pair<ContactState, Contact>> addContacts;
  for(const auto contact : contacts)
  {
    addContacts.push_back({contact.first, {robot(), footTasks[contact.first]->surface(), contact.second, c_.friction}});
  }
  setContacts(addContacts);
}

void StabilizerTask::setContacts(const std::vector<std::pair<ContactState, Contact>> & contacts)
{
  if(contacts.empty())
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "[StabilizerTask] Cannot set contacts from an empty list, the stabilizer "
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
    inTheAir_ = inTheAir_ && footT.second->measuredWrench().force().z() < MIN_DS_PRESSURE;
  }
}

void StabilizerTask::computeLeftFootRatio()
{
  if(inDoubleSupport())
  {
    // Project desired CoM in-between foot-sole ankle frames and compute ratio along the line in-beween the two surfaces
    const Eigen::Vector3d & lankle = contacts_.at(ContactState::Left).anklePose().translation();
    const Eigen::Vector3d & rankle = contacts_.at(ContactState::Right).anklePose().translation();
    Eigen::Vector3d t_lankle_com = comTarget_ - lankle;
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

sva::PTransformd StabilizerTask::anchorFrame() const
{
  return sva::interpolate(robot().surfacePose(footTasks.at(ContactState::Left)->surface()),
                          robot().surfacePose(footTasks.at(ContactState::Right)->surface()), leftFootRatio_);
}

sva::PTransformd StabilizerTask::anchorFrameReal() const
{
  return sva::interpolate(realRobot().surfacePose(footTasks.at(ContactState::Left)->surface()),
                          realRobot().surfacePose(footTasks.at(ContactState::Right)->surface()), leftFootRatio_);
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
                            const Eigen::Vector3d & zmp)
{
  comTarget_ = com;
  comdTarget_ = comd;
  comddTarget_ = comdd;
  zmpTarget_ = zmp;
  omega_ = std::sqrt(constants::gravity.z() / comTarget_.z());
  dcmTarget_ = comTarget_ + comdTarget_ / omega_;
}

void StabilizerTask::run()
{
  using namespace std::chrono;
  using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                          std::chrono::high_resolution_clock, std::chrono::steady_clock>::type;
  auto startTime = clock::now();

  checkGains();
  checkInTheAir();
  computeLeftFootRatio();
  setSupportFootGains();
  updateZMPFrame();
  if(!inTheAir_)
  {
    measuredNetWrench_ = robots_.robot(robotIndex_).netWrench(contactSensors);
    measuredZMP_ = robots_.robot(robotIndex_).zmp(measuredNetWrench_, zmpFrame_, MIN_NET_TOTAL_FORCE_ZMP);
  }
  else
  {
    measuredNetWrench_ = sva::ForceVecd::Zero();
  }
  auto desiredWrench = computeDesiredWrench();

  if(inDoubleSupport())
  {
    distributeWrench(desiredWrench);
  }
  else if(inContact(ContactState::Left))
  {
    saturateWrench(desiredWrench, footTasks[ContactState::Left], contacts_.at(ContactState::Left));
    footTasks[ContactState::Right]->setZeroTargetWrench();
  }
  else
  {
    saturateWrench(desiredWrench, footTasks[ContactState::Right], contacts_.at(ContactState::Right));
    footTasks[ContactState::Left]->setZeroTargetWrench();
  }

  updateCoMTaskZMPCC();
  updateFootForceDifferenceControl();

  comTask->com(comTarget_);
  comTask->refVel(comdTarget_);
  comTask->refAccel(comddTarget_);

  // Update orientation tasks according to feet orientation
  sva::PTransformd X_0_a = anchorFrame();
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
  }
  else
  {
    Eigen::Vector3d zmpError = zmpTarget_ - measuredZMP_;
    zmpError.z() = 0.;
    dcmDerivator_.update(omega_ * (dcmError_ - zmpError));
    dcmIntegrator_.append(dcmError_);
  }
  dcmAverageError_ = dcmIntegrator_.eval();
  dcmVelError_ = dcmDerivator_.eval();

  Eigen::Vector3d desiredCoMAccel = comddTarget_;
  desiredCoMAccel += omega_ * (c_.dcmPropGain * dcmError_ + comdError);
  desiredCoMAccel += omega_ * c_.dcmIntegralGain * dcmAverageError_;
  desiredCoMAccel += omega_ * c_.dcmDerivGain * dcmVelError_;
  auto desiredForce = mass_ * (desiredCoMAccel + constants::gravity);

  // Previous implementation (up to v1.3):
  // return {pendulum_.com().cross(desiredForce), desiredForce};
  // See https://github.com/stephane-caron/lipm_walking_controller/issues/28
  return {measuredCoM_.cross(desiredForce), desiredForce};
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
  b_ineq(32) = -MIN_DS_PRESSURE;
  // w_r_rc.force().z() >= MIN_DS_PRESSURE
  A_ineq.block<1, 6>(33, 6) = -X_0_rc.dualMatrix().bottomRows<1>();
  b_ineq(33) = -MIN_DS_PRESSURE;

  qpSolver_.problem(NB_VAR, 0, NB_CONS);
  Eigen::MatrixXd A_eq(0, 0);
  Eigen::VectorXd b_eq;
  b_eq.resize(0);

  bool solutionFound = qpSolver_.solve(Q, c, A_eq, b_eq, A_ineq, b_ineq, /* isDecomp = */ false);
  if(!solutionFound)
  {
    LOG_ERROR("[StabilizerTask] DS force distribution QP: solver found no solution");
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
    LOG_ERROR("[StabilizerTask] SS force distribution QP: solver found no solution");
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
    auto distribZMP = mc_rbdyn::zmp(distribWrench_, zmpFrame_);
    zmpcc_.configure(c_.zmpcc);
    zmpcc_.enabled(enabled_);
    zmpcc_.update(distribZMP, measuredZMP_, zmpFrame_, dt_);
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

} // namespace lipm_stabilizer
} // namespace mc_tasks

namespace
{
static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "lipm_stabilizer",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      unsigned robotIndex = config("robotIndex", 0u);
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

      auto t = std::make_shared<mc_tasks::lipm_stabilizer::StabilizerTask>(
          solver.robots(), solver.realRobots(), robotIndex, stabiConf.leftFootSurface, stabiConf.rightFootSurface,
          stabiConf.torsoBodyName, solver.dt());
      t->reset();
      t->configure(stabiConf);
      t->load(solver, config);
      t->commitConfig();
      return t;
    });
}
