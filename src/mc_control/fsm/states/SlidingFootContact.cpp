/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/states/SlidingFootContact.h>
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
  config("TickSupport", tickSupport_);
  config("TickAdjust", tickAdjust_);
  config("MoveCoM_Z", move_com_z_);
  config("RotAngle", rot_angle_);
  config("CoMOffset", com_offset_);
  config("CoMOffsetSliding", com_offset_sliding_);
  config("WaitForSlideTrigger", wait_for_slide_trigger_);
  config("Next", next_);
}

void SlidingFootContactState::start(Controller & ctl)
{
  LOG_ERROR("Enter state with CoM offset " << com_offset_.transpose())
  LOG_ERROR("Enter state with CoM sliding offset " << com_offset_sliding_.transpose())
  comTask_ = std::make_shared<mc_tasks::CoMTask>(ctl.robots(), 0, 10.0, 2000.0);
  comTask_->dimWeight(Eigen::Vector3d{1.0, 1.0, 1.0});
  comTask_->move_com({0, 0, move_com_z_});
  ctl.solver().addTask(comTask_);
  copSlidingFootTask_ = std::make_shared<mc_tasks::force::CoPTask>(slidingSurface_, ctl.robots(), 0, 10.0, 500.0);
  Eigen::Vector6d slidingDimW;
  slidingDimW << 4.0, 4.0, 1.0, 1.0, 1.0, 100.0;
  copSlidingFootTask_->dimWeight(slidingDimW);
  copSupportFootTask_ = std::make_shared<mc_tasks::force::CoPTask>(supportSurface_, ctl.robots(), 0, 10.0, 2000.0);
  copSupportFootTask_->admittance({{5e-3, 5e-3, 0}, {0, 0, 0}});
  if(handSurface_.size())
  {
    ctl.getPostureTask(ctl.robot().name())->target({{"R_WRIST_R", {-1.4}}});
    copHandTask_ = std::make_shared<mc_tasks::force::CoPTask>(handSurface_, ctl.robots(), 0, 5.0, 200.0);
    if(handForceTarget_ > 0)
    {
      copHandTask_->targetForce({0., 0., handForceTarget_});
      copHandTask_->admittance({{0., 0., 0.}, {0., 0., 1e-4}});
    }
    ctl.solver().addTask(copHandTask_);
  }
  else
  {
    rhRelEf_ = std::make_shared<mc_tasks::RelativeEndEffectorTask>("r_wrist", ctl.robots(), 0, "", 2.0, 100.0);
    ctl.solver().addTask(rhRelEf_);
  }
  chestOriTask_ = std::make_shared<mc_tasks::OrientationTask>("torso", ctl.robots(), 0, 2.0, 100.0);
  chestOriTask_->dimWeight(Eigen::Vector3d{0., 1., 0.});
  // ctl.solver().addTask(chestOriTask_);
  lhRelEf_ = std::make_shared<mc_tasks::RelativeEndEffectorTask>("l_wrist", ctl.robots(), 0, "", 2.0, 100.0);
  ctl.solver().addTask(lhRelEf_);
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
                           [this]() -> const Eigen::Vector3d & { return com_target0; });
  ctl.logger().addLogEntry("Sliding_" + slidingSurface_ + "_com_sensor",
                           [this]() -> const Eigen::Vector3d & { return com_sensor; });
  if(wait_for_slide_trigger_)
  {
    auto gui = ctl.gui();
    if(!gui)
    {
      return;
    }
    slide_triggered_ = false;
    gui->addElement({"FSM"},
                    mc_rtc::gui::Button("Report offset",
                                        [this]() {
                                          std::cout << "New offset "
                                                    << (com_offset_ + comTask_->com() - com_target0).transpose()
                                                    << std::endl;
                                        }),
                    mc_rtc::gui::Button("Free foot Z",
                                        [this, &ctl]() {
                                          Eigen::Vector6d dof;
                                          dof << 1., 1., 1., 1., 1., 0.;
                                          copSlidingFootTask_->admittance({{0, 0, 0}, {0, 0, 1e-4}});
                                          copSlidingFootTask_->targetForce({0., 0., slidingForceTarget_});
                                          slidingContactId_ = getContactId(ctl, slidingSurface_);
                                          ctl.contactConstraint().contactConstr->removeDofContact(slidingContactId_);
                                          auto res = ctl.contactConstraint().contactConstr->addDofContact(
                                              slidingContactId_, dof.asDiagonal());
                                          if(!res)
                                          {
                                            LOG_ERROR("Failed to set dof contact for " << slidingSurface_)
                                          }
                                          ctl.contactConstraint().contactConstr->updateDofContacts();
                                          ctl.solver().addTask(copSlidingFootTask_);
                                        }),
                    mc_rtc::gui::ArrayInput("Sliding target", {"x", "y"}, [this]() { return move_; },
                                            [this](const Eigen::Vector2d & move) { move_ = move; }),
                    mc_rtc::gui::ComboInput("Next foot", {slidingSurface_, supportSurface_}, [this]() { return next_; },
                                            [this](const std::string & s) { next_ = s; }),
                    mc_rtc::gui::Button("SLIDE!", [this]() {
                      if(phase_ == Phase::REACH_SUPPORT && !slide_triggered_)
                      {
                        if(next_ == "")
                        {
                          LOG_ERROR("SELECT NEXT SLIDING FEET")
                        }
                        else
                        {
                          slide_triggered_ = true;
                        }
                      }
                    }));
  }
}

bool SlidingFootContactState::run(Controller & ctl)
{
  controlCoM(ctl);
  double fZ = 0;
  switch(phase_)
  {
    case Phase::REACH_SUPPORT:
      tick_++;
      if(tick_ > tickSupport_ && slide_triggered_)
      {
        phase_ = Phase::SLIDE_FOOT;
        tick_ = 0;
        ctl.solver().removeTask(copSlidingFootTask_);
        copSlidingFootTask_->admittance({{0, 0, 0}, {0, 0, 0}});
        ctl.solver().addTask(copSlidingFootTask_);
        Eigen::Vector6d dof;
        dof << 1., 1., 1., 0., 0., 1.;
        slidingContactId_ = getContactId(ctl, slidingSurface_);
        if(!kinematic_)
        {
          dof(5) = 0.;
        }
        ctl.contactConstraint().contactConstr->removeDofContact(slidingContactId_);
        ctl.contactConstraint().contactConstr->addDofContact(slidingContactId_, dof.asDiagonal());
        ctl.contactConstraint().contactConstr->updateDofContacts();
        auto t = copSlidingFootTask_->targetPose();
        t.translation().x() += move_.x();
        t.translation().y() += move_.y();
        com_offset_ = com_offset_sliding_;
        forceDistChanged_ = true;
        copSlidingFootTask_->targetPose(t);
        comTask_->stiffness(2.0);
        comTask_->weight(1000.0);
        // LOG_INFO("SlidingFoot::" << slidingSurface_ << " enable adjust sliding force")
        LOG_INFO("SlidingFoot::" << slidingSurface_ << " sliding now")
      }
      break;
    case Phase::ADJUST_SLIDING_FORCE:
      tick_++;
      controlSlidingForce();
      fZ = copSlidingFootTask_->measuredWrench().force().z();
      if((tick_ > tickAdjust_)) // && fZ < slidingForceTarget_) ||
                                //(tick_ > 3*tickAdjust_) )
      {
        tick_ = 0;
        phase_ = Phase::SLIDE_FOOT;
        Eigen::Vector6d dof;
        dof << 1., 1., 1., 0., 0., 1.;
        slidingContactId_ = getContactId(ctl, slidingSurface_);
        if(!kinematic_)
        {
          dof(5) = 0.;
        }
        ctl.contactConstraint().contactConstr->removeDofContact(slidingContactId_);
        ctl.contactConstraint().contactConstr->addDofContact(slidingContactId_, dof.asDiagonal());
        ctl.contactConstraint().contactConstr->updateDofContacts();
        auto t = copSlidingFootTask_->targetPose();
        t.translation().x() += move_.x();
        t.translation().y() += move_.y();
        com_offset_ = com_offset_sliding_;
        forceDistChanged_ = true;
        copSlidingFootTask_->targetPose(t);
        comTask_->stiffness(2.0);
        comTask_->weight(1000.0);
        LOG_INFO("SlidingFoot::" << slidingSurface_
                                 << " move foot (force: " << copSlidingFootTask_->measuredWrench().force().z()
                                 << "N, target: " << slidingForceTarget_ << "N)")
      }
      break;
    case Phase::SLIDE_FOOT:
      tick_++;
      controlSlidingForce();
      if((tick_ > 500 && copSlidingFootTask_->speed().segment(3, 2).norm() < 0.001) || tick_ > 2000)
      {
        ctl.solver().removeTask(copSlidingFootTask_);
        // ctl.solver().removeTask(copSupportFootTask_);
        com_offset_ = Eigen::Vector3d::Zero();
        resetAndRestoreBalance(ctl);
        if(tick_ > 2000)
        {
          LOG_WARNING("SlidingFoot::" << slidingSurface_ << " timeout")
        }
        LOG_ERROR("eval: " << copSlidingFootTask_->eval().segment(3, 2).norm() << ", tick: " << tick_)
        LOG_ERROR("speed: " << copSlidingFootTask_->speed().segment(3, 2).norm() << ", tick: " << tick_)
        LOG_INFO("SlidingFoot::" << slidingSurface_ << " restore balance")
        tick_ = 0;
        phase_ = Phase::BALANCE;
      }
      break;
    case Phase::BALANCE:
      tick_++;
      if(tick_ > 400)
      {
        phase_ = Phase::REGULATE_FOOT_ORIENTATION;
        LOG_INFO("Start to regulate sliding foot orientation")
        tick_ = 0;
        ctl.solver().addTask(copSlidingFootTask_);
        copSlidingFootTask_->admittance({{5e-3, 5e-3, 0}, {0, 0, 0}});
        Eigen::Vector6d dof;
        dof << 0., 0., 1., 1., 1., 1.;
        slidingContactId_ = getContactId(ctl, slidingSurface_);
        ctl.contactConstraint().contactConstr->removeDofContact(slidingContactId_);
        ctl.contactConstraint().contactConstr->addDofContact(slidingContactId_, dof.asDiagonal());
        ctl.contactConstraint().contactConstr->updateDofContacts();
      }
      break;
    case Phase::REGULATE_FOOT_ORIENTATION:
      tick_++;
      if(tick_ > 1000)
      {
        ctl.solver().removeTask(copSlidingFootTask_);
        if(next_ == supportSurface_)
        {
          output("OK");
        }
        else
        {
          output("SAME");
        }
        return true;
      }
    default:
      break;
  };
  return false;
}

void SlidingFootContactState::teardown(Controller & ctl)
{
  ctl.logger().removeLogEntry("Sliding_" + slidingSurface_ + "_com_targetIn");
  ctl.logger().removeLogEntry("Sliding_" + slidingSurface_ + "_com_sensor");
  if(wait_for_slide_trigger_)
  {
    auto gui = ctl.gui();
    if(gui)
    {
      std::cout << "Remove elements?" << std::endl;
      gui->removeElement({"#FSM#"}, "SLIDE!");
      gui->removeElement({"#FSM#"}, "Report offset");
      gui->removeElement({"#FSM#"}, "Sliding target");
      gui->removeElement({"#FSM#"}, "Next foot");
      gui->removeElement({"#FSM#"}, "Free foot Z");
      std::cout << "OK" << std::endl;
    }
  }
  ctl.solver().removeTask(comTask_);
  // ctl.solver().removeTask(chestOriTask_);
  if(rhRelEf_)
  {
    ctl.solver().removeTask(rhRelEf_);
  }
  ctl.solver().removeTask(lhRelEf_);
  if(copHandTask_)
  {
    ctl.solver().removeTask(copHandTask_);
  }
  ctl.contactConstraint().contactConstr->resetDofContacts();
  ctl.contactConstraint().contactConstr->updateDofContacts();
}

tasks::qp::ContactId SlidingFootContactState::getContactId(Controller & ctl, const std::string & s)
{
  for(const auto & c : ctl.solver().contacts())
  {
    if((c.r1Index() == 0 && c.r1Surface()->name() == s) || (c.r2Index() == 0 && c.r2Surface()->name() == s))
    {
      return c.contactId(ctl.robots());
    }
  }
  LOG_ERROR_AND_THROW(std::runtime_error, "Failed to find contact id for " << s)
}

void SlidingFootContactState::setHandDofContact(Controller & ctl)
{
  if(!kinematic_ && handSurface_.size() && handForceTarget_ > 0)
  {
    auto cId = getContactId(ctl, handSurface_);
    Eigen::Vector6d dof;
    dof << 1., 1., 1., 1., 0., 1.;
    auto res = ctl.contactConstraint().contactConstr->addDofContact(cId, dof.asDiagonal());
    if(!res)
    {
      LOG_ERROR("Failed to add dof contact on hand")
    }
    ctl.contactConstraint().contactConstr->updateDofContacts();
  }
}

void SlidingFootContactState::controlCoM(Controller &)
{
  if(forceDistChanged_)
  {
    com_target0.x() = copSupportFootTask_->targetPose().translation().x() * supportForceTarget_ / mg_
                      + copSlidingFootTask_->targetPose().translation().x() * slidingForceTarget_ / mg_;
    com_target0.y() = copSupportFootTask_->targetPose().translation().y() * supportForceTarget_ / mg_
                      + copSlidingFootTask_->targetPose().translation().y() * slidingForceTarget_ / mg_;
    if(copHandTask_)
    {
      com_target0.x() += copHandTask_->targetPose().translation().x() * handForceTarget_ / mg_;
      com_target0.y() += copHandTask_->targetPose().translation().y() * handForceTarget_ / mg_;
    }
    com_target0.x() += com_offset_.x();
    com_target0.y() += com_offset_.y();
    if(com_init_z_ < 0)
    {
      com_init_z_ = comTask_->com().z();
    }
    com_target0.z() = com_init_z_ + com_offset_.z();
    LOG_ERROR("Applied CoM offset: " << com_offset_.transpose())
    initial_com = comTask_->com();
    comTask_->com(com_target0);
    forceDistChanged_ = false;
  }
  // if(kinematic_) { return; }
  // com_sensor = Eigen::Vector3d::Zero();
  // auto copSupport = copSupportFootTask_->worldMeasuredCoP();
  // auto copSliding = copSlidingFootTask_->worldMeasuredCoP();
  // auto wrenchSupport = copSupportFootTask_->measuredWrench();
  // auto wrenchSliding = copSlidingFootTask_->measuredWrench();
  // com_sensor.x() = copSupport.translation().x() * wrenchSupport.force().z() / mg_ +
  //                 copSliding.translation().x() * wrenchSliding.force().z() / mg_;
  // com_sensor.y() = copSupport.translation().y() * wrenchSupport.force().z() / mg_ +
  //                 copSliding.translation().y() * wrenchSliding.force().z() / mg_;
  // if(copHandTask_)
  //{
  //  auto copH = copHandTask_->worldMeasuredCoP();
  //  auto wrenchH = copHandTask_->measuredWrench();
  //  com_sensor.x() += copH.translation().x() * wrenchH.force().z() / mg_ -
  //                    copH.translation().z() * wrenchH.force().x() / mg_;
  //  com_sensor.y() += copH.translation().y() * wrenchH.force().z() / mg_ -
  //                    copH.translation().z() * wrenchH.force().y() / mg_;
  //}
  // double alpha = 1e-3;
  ////FIXME Should be current com target?
  // Eigen::Vector3d delta_com = alpha*(com_target0 - com_sensor);
  // delta_com.z() = 0.0;
  // comTask_->move_com(delta_com);
}

void SlidingFootContactState::controlSlidingForce()
{
  if(kinematic_)
  {
    return;
  }
  double forceDiff = copSlidingFootTask_->measuredWrench().force().z() - slidingForceTarget_;
  auto nPose = copSlidingFootTask_->targetPose();
  nPose.translation().z() += 1e-6 * forceDiff;
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
  ctl.contactConstraint().contactConstr->updateDofContacts();
  ctl.solver().setContacts(ctl.solver().contacts());
  ctl.getPostureTask(ctl.robot().name())->reset();
  comTask_->reset();
  if(copHandTask_)
  {
    copHandTask_->reset();
    copHandTask_->admittance({{0., 0., 0.}, {0., 0., 1e-4}});
    setHandDofContact(ctl);
  }
  slidingForceTarget_ = (mg_ - handForceTarget_) / 2;
  supportForceTarget_ = (mg_ - handForceTarget_) / 2;
  forceDistChanged_ = true;
}

} // namespace fsm

} // namespace mc_control

EXPORT_SINGLE_STATE("SlidingFootContact", mc_control::fsm::SlidingFootContactState)
