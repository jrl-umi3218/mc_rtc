#include "PreviewControlWalking_Initial.h"

PreviewControlWalking_Initial::PreviewControlWalking_Initial()
: previewControl_({mc_planning::PreviewControl("x"), mc_planning::PreviewControl("y")})
{
}

void PreviewControlWalking_Initial::configure(const mc_rtc::Configuration &) {}

void PreviewControlWalking_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PreviewControlWalking &>(ctl_);

  // Setup StabilizerTask
  stabilizerConfig_ = ctl.robot().module().defaultLIPMStabilizerConfiguration();
  stabilizerTask_.reset(new mc_tasks::lipm_stabilizer::StabilizerTask(
      ctl.solver().robots(), ctl.solver().realRobots(), ctl.robot().robotIndex(), stabilizerConfig_.leftFootSurface,
      stabilizerConfig_.rightFootSurface, stabilizerConfig_.torsoBodyName, ctl.dt_));
  stabilizerTask_->configure(stabilizerConfig_);
  ctl.solver().addTask(stabilizerTask_);

  // Setup SurfaceTransformTask
  swingFootTasks_[mc_planning::Foot::Left].reset(new mc_tasks::SurfaceTransformTask(
      stabilizerConfig_.leftFootSurface, ctl.robots(), ctl.robots().robotIndex(), 2000, 500));
  swingFootTasks_[mc_planning::Foot::Right].reset(new mc_tasks::SurfaceTransformTask(
      stabilizerConfig_.rightFootSurface, ctl.robots(), ctl.robots().robotIndex(), 2000, 500));

  // Setup FootstepManager
  footstepManager_.dt(ctl.dt_);
  mc_planning::Footstance currentFootstance;
  currentFootstance.insert(
      {mc_planning::Foot::Left,
       mc_planning::Footstep(mc_planning::Foot::Left, ctl.robot().surfacePose(stabilizerConfig_.leftFootSurface),
                             ctl.t())});
  currentFootstance.insert(
      {mc_planning::Foot::Right,
       mc_planning::Footstep(mc_planning::Foot::Right, ctl.robot().surfacePose(stabilizerConfig_.rightFootSurface),
                             ctl.t())});
  footstepManager_.reset(ctl.t(), currentFootstance, currentFootstance.midPose().translation());
  footstepManager_.addToLogger(ctl.logger());

  // Setup PreviewControl
  for(auto i : {0, 1})
  {
    previewControl_[i].reset(Eigen::Vector3d(currentFootstance.midPose().translation()[i], 0, 0),
                             currentFootstance.midPose().translation()[i]);
    previewControl_[i].calcGain(stabilizerConfig_.comHeight, horizon_, ctl.dt_);
    previewControl_[i].addToLogger(ctl.logger());
  }

  refZmpTraj_.resize(3, previewSize() + 1);

  // Append target footsteps
  for(int i = 0; i < 10; i++)
  {
    mc_planning::Foot foot = i % 2 == 0 ? mc_planning::Foot::Left : mc_planning::Foot::Right;
    double footPosY = currentFootstance.at(foot).pose.translation().y();
    sva::PTransformd footPose(Eigen::Vector3d((i + 1) * 0.3, footPosY, 0));
    double time = ctl.t() + 2.0 + i;
    footstepManager_.appendFootstep(mc_planning::Footstep(foot, footPose, time));
  }

  // Setup contacts
  setContacts(ctl,
              {{mc_tasks::lipm_stabilizer::ContactState::Left, currentFootstance.at(mc_planning::Foot::Left).pose},
               {mc_tasks::lipm_stabilizer::ContactState::Right, currentFootstance.at(mc_planning::Foot::Right).pose}});
  supportPhase_ = mc_planning::SupportPhase::DoubleSupport;

  // Setup KinematicAnchorFrame
  ctl.datastore().remove("KinematicAnchorFrame::" + ctl.robot().name());
  ctl.datastore().make_call("KinematicAnchorFrame::" + ctl.robot().name(), [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose(stabilizerConfig_.rightFootSurface),
                            robot.surfacePose(stabilizerConfig_.leftFootSurface), footstepManager_.leftFootRatio());
  });

  // Run once to update the logging data
  run(ctl_);
}

bool PreviewControlWalking_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PreviewControlWalking &>(ctl_);

  // Update FootstepManager and get a reference ZMP trajectory
  footstepManager_.update(ctl.t());
  footstepManager_.makeZmpTraj(refZmpTraj_);

  // Update PreviewControl
  for(auto i : {0, 1})
  {
    // The beginning of the reference ZMP trajectory returned by FootstepManager is the current reference value, which
    // is excluded when passing it to PreviewControl
    previewControl_[i].setRefZmpTraj(refZmpTraj_.row(i).segment(1, previewSize()));
    previewControl_[i].update();
  }

  // Update the target of StabilizerTask
  Eigen::Vector3d targetCoM, targetCoMd, targetCoMdd, targetZMP;
  targetCoM << previewControl_[0].comState()[0], previewControl_[1].comState()[0], stabilizerConfig_.comHeight;
  targetCoMd << previewControl_[0].comState()[1], previewControl_[1].comState()[1], 0;
  targetCoMdd << previewControl_[0].comState()[2], previewControl_[1].comState()[2], 0;
  targetZMP << previewControl_[0].zmp(), previewControl_[1].zmp(), 0;
  stabilizerTask_->target(targetCoM, targetCoMd, targetCoMdd, targetZMP);

  // Update contacts and add/remove SurfaceTransformTask of swing foot when support phase is switched
  if(footstepManager_.supportPhase() != supportPhase_)
  {
    mc_planning::SupportPhase prevSupportPhase = supportPhase_;
    supportPhase_ = footstepManager_.supportPhase();
    if(supportPhase_ == mc_planning::SupportPhase::LeftSupport)
    {
      setContacts(
          ctl, {{mc_tasks::lipm_stabilizer::ContactState::Left, footstepManager_.footPose(mc_planning::Foot::Left)}});
      ctl.solver().addTask(swingFootTasks_.at(mc_planning::Foot::Right));
    }
    else if(supportPhase_ == mc_planning::SupportPhase::RightSupport)
    {
      setContacts(
          ctl, {{mc_tasks::lipm_stabilizer::ContactState::Right, footstepManager_.footPose(mc_planning::Foot::Right)}});
      ctl.solver().addTask(swingFootTasks_.at(mc_planning::Foot::Left));
    }
    else // supportPhase_ == mc_planning::SupportPhase::DoubleSupport
    {
      setContacts(
          ctl, {{mc_tasks::lipm_stabilizer::ContactState::Left, footstepManager_.footPose(mc_planning::Foot::Left)},
                {mc_tasks::lipm_stabilizer::ContactState::Right, footstepManager_.footPose(mc_planning::Foot::Right)}});
      if(prevSupportPhase == mc_planning::SupportPhase::LeftSupport)
      {
        ctl.solver().removeTask(swingFootTasks_.at(mc_planning::Foot::Right));
      }
      else
      {
        ctl.solver().removeTask(swingFootTasks_.at(mc_planning::Foot::Left));
      }
    }
  }

  // Update the target of SurfaceTransformTask
  if(supportPhase_ == mc_planning::SupportPhase::LeftSupport)
  {
    swingFootTasks_.at(mc_planning::Foot::Right)->target(footstepManager_.footPose(mc_planning::Foot::Right));
  }
  else if(supportPhase_ == mc_planning::SupportPhase::RightSupport)
  {
    swingFootTasks_.at(mc_planning::Foot::Left)->target(footstepManager_.footPose(mc_planning::Foot::Left));
  }

  return false;
}

void PreviewControlWalking_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<PreviewControlWalking &>(ctl_);

  footstepManager_.removeFromLogger(ctl.logger());
  for(auto i : {0, 1})
  {
    previewControl_[i].removeFromLogger(ctl.logger());
  }

  ctl.datastore().remove("KinematicAnchorFrame::" + ctl.robot().name());
  ctl.datastore().make_call("KinematicAnchorFrame::" + ctl.robot().name(), [this](const mc_rbdyn::Robot & robot) {
    return sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), 0.5);
  });
}

void PreviewControlWalking_Initial::setContacts(
    PreviewControlWalking & ctl,
    const std::vector<std::pair<mc_tasks::lipm_stabilizer::ContactState, sva::PTransformd>> & contacts,
    bool fullDoF)
{
  stabilizerTask_->setContacts(contacts);
  auto rName = ctl.robot().name();
  auto gName = "ground";
  auto gSurface = "AllGround";
  auto friction = stabilizerTask_->config().friction;
  ctl.removeContact(
      {rName, gName, stabilizerTask_->footSurface(mc_tasks::lipm_stabilizer::ContactState::Left), gSurface});
  ctl.removeContact(
      {rName, gName, stabilizerTask_->footSurface(mc_tasks::lipm_stabilizer::ContactState::Right), gSurface});
  for(const auto & contact : contacts)
  {
    const auto & rSurface = stabilizerTask_->footSurface(contact.first);
    Eigen::Vector6d dof = Eigen::Vector6d::Ones();
    if(!fullDoF)
    {
      dof(0) = 0;
      dof(1) = 0;
      dof(5) = 0;
    }
    ctl.addContact({rName, gName, rSurface, gSurface, friction, dof});
  }
}

EXPORT_SINGLE_STATE("PreviewControlWalking_Initial", PreviewControlWalking_Initial)
