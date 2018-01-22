#include <mc_control/fsm/states/SlidingFootContact.h>

#include <mc_control/fsm/Controller.h>

#include <mc_tasks/MetaTaskLoader.h>

namespace mc_control
{

namespace fsm
{

void SlidingFootContactState::configure(const mc_rtc::Configuration & config)
{
  config("kinematic", kinematic_);
  config("SlidingSurface", slidingSurface_);
  config("SupportSurface", supportSurface_);
  config("HandSurface", handSurface_);
  config("SlidingForceTarget", slidingForceTarget_);
  config("HandForceTarget", handForceTarget_);
  config("Move", move_);
}

void SlidingFootContactState::start(Controller & ctl)
{
  comTask_ = std::make_shared<mc_tasks::CoMTask>(ctl.robots(), 0, 5.0, 1000.0);
  ctl.solver().addTask(comTask_);
  copSlidingFootTask_ = std::make_shared<mc_tasks::CoPTask>(slidingSurface_, ctl.robots(), 0, ctl.solver().dt(), 5.0, 1000.0);
  copSupportFootTask_ = std::make_shared<mc_tasks::CoPTask>(supportSurface_, ctl.robots(), 0, ctl.solver().dt(), 5.0, 1000.0);
  if(handSurface_.size())
  {
    copHandTask_ = std::make_shared<mc_tasks::CoPTask>(handSurface_, ctl.robots(), 0, ctl.solver().dt(), 5.0, 1000.0);
    if(!kinematic_ && handForceTarget_ > 0)
    {
      copHandTask_->targetForce({0.,0.,handForceTarget_});
      copHandTask_->admittance({{0.,0.,0.},{0., 0., 1e-4}});
    }
    ctl.solver().addTask(copHandTask_);
  }
  chestOriTask_ = std::make_shared<mc_tasks::OrientationTask>("torso", ctl.robots(), 0, 2.0, 100.0);
  /** Initialize force targets */
  for(const auto & b : ctl.robot().mb().bodies())
  {
    mg_ += b.inertia().mass();
  }
  mg_ *= ctl.robot().mbc().gravity.z();
  supportForceTarget_ = mg_ - slidingForceTarget_ - handForceTarget_;
  /** Setup dof contacts */
  ctl.contactConstraint().contactConstr->resetDofContacts();
  setHandDofContact(ctl);
  slidingContactId_ = getContactId(ctl, slidingSurface_);
  ctl.logger().addLogEntry("Sliding_" + slidingSurface_ + "_com_targetIn",
                           [this]() -> const Eigen::Vector3d &
                           {
                           return com_target0;
                           });
  ctl.logger().addLogEntry("Sliding_" + slidingSurface_ + "_com_sensor",
                           [this]() -> const Eigen::Vector3d &
                           {
                           return com_sensor;
                           });
}

bool SlidingFootContactState::run(Controller & ctl)
{
  controlCoM(ctl);
  switch(phase_)
  {
    case Phase::REACH_SUPPORT:
      tick_++;
      if(tick_ > 300) // FIXME Should be configurable
      {
        phase_ = Phase::ADJUST_SLIDING_FORCE;
        tick_ = 0;
        Eigen::Vector6d dof;
        dof << 1., 1., 1., 1., 1., 0.;
        ctl.contactConstraint().contactConstr->addDofContact(slidingContactId_, dof.asDiagonal());
        ctl.contactConstraint().contactConstr->updateDofContacts();
        ctl.solver().addTask(copSlidingFootTask_);
        LOG_INFO("SlidingFoot::" << slidingSurface_ << " enable adjust sliding force")
      }
      break;
    case Phase::ADJUST_SLIDING_FORCE:
      tick_++;
      controlSlidingForce();
      if(tick_ > 300) // FIXME Should be configurable
      {
        tick_ = 0;
        phase_ = Phase::SLIDE_FOOT;
        Eigen::Vector6d dof;
        dof << 1., 1., 1., 0., 0., 0.;
        ctl.contactConstraint().contactConstr->removeDofContact(slidingContactId_);
        ctl.contactConstraint().contactConstr->addDofContact(slidingContactId_, dof.asDiagonal());
        ctl.contactConstraint().contactConstr->updateDofContacts();
        auto t = copSlidingFootTask_->targetPose();
        t.translation().x() += move_.x();
        t.translation().y() += move_.y();
        forceDistChanged_ = true;
        copSlidingFootTask_->targetPose(t);
        LOG_INFO("SlidingFoot::" << slidingSurface_ << " move foot (force: " << copSlidingFootTask_->measuredWrench().force().z() << "N, target: " << slidingForceTarget_ << "N)")
      }
      break;
    case Phase::SLIDE_FOOT:
      tick_++;
      controlSlidingForce();
      if(tick_ > 100 && copSlidingFootTask_->speed().head(5).norm() < 1e-3)
      {
        ctl.solver().removeTask(copSlidingFootTask_);
        resetAndRestoreBalance(ctl);
        LOG_INFO("SlidingFoot::" << slidingSurface_ << " restore balance")
        tick_ = 0;
        phase_ = Phase::BALANCE;
      }
      break;
    case Phase::BALANCE:
      tick_++;
      if(tick_ > 400)
      {
        output("OK");
        return true;
      }
      break;
    default:
      break;
  };
  return false;
}

void SlidingFootContactState::teardown(Controller & ctl)
{
  ctl.logger().removeLogEntry("Sliding_" + slidingSurface_ + "_com_targetIn");
  ctl.logger().removeLogEntry("Sliding_" + slidingSurface_ + "_com_sensor");
  ctl.solver().removeTask(comTask_);
  if(copHandTask_)
  {
    ctl.solver().removeTask(copHandTask_);
    ctl.contactConstraint().contactConstr->resetDofContacts();
    ctl.contactConstraint().contactConstr->updateDofContacts();
  }
}

tasks::qp::ContactId SlidingFootContactState::getContactId(Controller & ctl,
                                                           const std::string & s)
{
  for(const auto & c : ctl.solver().contacts())
  {
    if( (c.r1Index() == 0 && c.r1Surface()->name() == s) ||
        (c.r2Index() == 0 && c.r2Surface()->name() == s) )
    {
      return c.contactId(ctl.robots());
    }
  }
  LOG_ERROR_AND_THROW(std::runtime_error, "Failed to find contact id for " << s)
}

void SlidingFootContactState::setHandDofContact(Controller & ctl)
{
  if(!kinematic_  && handSurface_.size() && handForceTarget_ > 0)
  {
    auto cId = getContactId(ctl, handSurface_);
    Eigen::Vector6d dof;
    dof << 1., 1., 1., 1., 0., 1.;
    ctl.contactConstraint().contactConstr->addDofContact(cId, dof.asDiagonal());
    ctl.contactConstraint().contactConstr->updateDofContacts();
  }
}

void SlidingFootContactState::controlCoM(Controller &)
{
  if(forceDistChanged_)
  {
    com_target0.x() = copSupportFootTask_->targetPose().translation().x() * supportForceTarget_ / mg_ +
                        copSlidingFootTask_->targetPose().translation().x() * slidingForceTarget_ / mg_;
    com_target0.y() = copSupportFootTask_->targetPose().translation().y() * supportForceTarget_ / mg_ +
                        copSlidingFootTask_->targetPose().translation().y() * slidingForceTarget_ / mg_;
    if(copHandTask_)
    {
      com_target0.x() += copHandTask_->targetPose().translation().x() * handForceTarget_ / mg_;
      com_target0.y() += copHandTask_->targetPose().translation().y() * handForceTarget_ / mg_;
    }
    com_target0.z() = comTask_->com().z();
    comTask_->com(com_target0);
    forceDistChanged_ = false;
  }
  if(kinematic_) { return; }
  com_sensor = Eigen::Vector3d::Zero();
  auto copSupport = copSupportFootTask_->worldMeasuredCoP();
  auto copSliding = copSlidingFootTask_->worldMeasuredCoP();
  auto wrenchSupport = copSupportFootTask_->measuredWrench();
  auto wrenchSliding = copSlidingFootTask_->measuredWrench();
  com_sensor.x() = copSupport.translation().x() * wrenchSupport.force().z() / mg_ +
                   copSliding.translation().x() * wrenchSliding.force().z() / mg_;
  com_sensor.y() = copSupport.translation().y() * wrenchSupport.force().z() / mg_ +
                   copSliding.translation().y() * wrenchSliding.force().z() / mg_;
  if(copHandTask_)
  {
    auto copH = copHandTask_->worldMeasuredCoP();
    auto wrenchH = copHandTask_->measuredWrench();
    com_sensor.x() += copH.translation().x() * wrenchH.force().z() / mg_ -
                      copH.translation().z() * wrenchH.force().x() / mg_;
    com_sensor.y() += copH.translation().y() * wrenchH.force().z() / mg_ -
                      copH.translation().z() * wrenchH.force().y() / mg_;
  }
  double alpha = 1e-3;
  //FIXME Should be current com target?
  Eigen::Vector3d delta_com = alpha*(com_target0 - com_sensor);
  delta_com.z() = 0.0;
  comTask_->move_com(delta_com);
}

void SlidingFootContactState::controlSlidingForce()
{
  if(kinematic_) { return; }
  double forceDiff = copSlidingFootTask_->measuredWrench().force().z() - slidingForceTarget_;
  auto nPose = copSlidingFootTask_->targetPose();
  nPose.translation().z() += 1e-6*forceDiff;
  copSlidingFootTask_->targetPose(nPose);
}

void SlidingFootContactState::resetAndRestoreBalance(Controller & ctl)
{
  auto X_0_ref = ctl.robot().surface(supportSurface_).X_0_s(ctl.robot());
  const auto & encoders = ctl.robot().encoderValues();
  const auto & rjo = ctl.robot().refJointOrder();
  ctl.robot().mbc().zero(ctl.robot().mb());
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    const auto & jName = rjo[i];
    const auto & qi = encoders[i];
    auto jIndex = ctl.robot().jointIndexByName(jName);
    ctl.robot().mbc().q[jIndex][0] = qi;
  }
  ctl.robot().forwardKinematics();
  auto X_0_new = ctl.robot().surface(supportSurface_).X_0_s(ctl.robot());
  auto X_new_ref = X_0_new.inv() * X_0_ref;
  auto q = Eigen::Quaterniond{X_new_ref.rotation().transpose()};
  const auto & t = X_new_ref.translation();
  ctl.robot().mbc().q[0] = {q.w(), q.x(), q.y(), q.z(), t.x(), t.y(), t.z()};
  ctl.robot().forwardKinematics();
  ctl.robot().forwardVelocity();
  ctl.contactConstraint().contactConstr->resetDofContacts();
  ctl.solver().setContacts(ctl.solver().contacts());
  ctl.getPostureTask(ctl.robot().name())->reset();
  comTask_->reset();
  if(copHandTask_)
  {
    copHandTask_->reset();
    copHandTask_->admittance({{0.,0.,0.},{0., 0., 1e-4}});
    setHandDofContact(ctl);
  }
  slidingForceTarget_ = (mg_ - handForceTarget_)/2;
  supportForceTarget_ = (mg_ - handForceTarget_)/2;
  forceDistChanged_ = true;
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("SlidingFootContact", mc_control::fsm::SlidingFootContactState, "OK")
