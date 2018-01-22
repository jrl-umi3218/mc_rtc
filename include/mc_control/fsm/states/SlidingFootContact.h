#pragma once

#include <mc_control/fsm/State.h>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/OrientationTask.h>

namespace mc_control
{

namespace fsm
{

/** Implements a state that is able to slide a foot contact */

struct SlidingFootContactState : State
{
  virtual ~SlidingFootContactState() = default;

  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller&) override;

  bool run(Controller&) override;

  void teardown(Controller&) override;
protected:
  /** Configuration */
  bool kinematic_ = false;
  std::string slidingSurface_;
  std::string supportSurface_;
  std::string handSurface_;
  double slidingForceTarget_ = 50.0;
  double supportForceTarget_ = 0.0;
  double handForceTarget_ = 20.0;
  Eigen::Vector2d move_;
  /** Implementation */
  std::shared_ptr<mc_tasks::CoMTask> comTask_;
  std::shared_ptr<mc_tasks::CoPTask> copHandTask_;
  std::shared_ptr<mc_tasks::CoPTask> copSlidingFootTask_;
  std::shared_ptr<mc_tasks::CoPTask> copSupportFootTask_; /** Only used for CoP computation */
  std::shared_ptr<mc_tasks::OrientationTask> chestOriTask_;
  double mg_ = 0.0;
  tasks::qp::ContactId slidingContactId_;
  /** Internal phases */
  enum class Phase
  {
    REACH_SUPPORT,
    ADJUST_SLIDING_FORCE,
    SLIDE_FOOT,
    BALANCE
  };
  Phase phase_ = Phase::REACH_SUPPORT;
  unsigned int tick_ = 0;
  bool forceDistChanged_ = true;
  Eigen::Vector3d com_target0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d com_sensor = Eigen::Vector3d::Zero();
protected:
  tasks::qp::ContactId getContactId(Controller & ctl, const std::string & s);
  void setHandDofContact(Controller & ctl);
  void controlCoM(Controller & ctl);
  void controlSlidingForce();
  void resetAndRestoreBalance(Controller & ctl);
};

} // namespace fsm

} // namespace mc_control
