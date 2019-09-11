/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/CoPTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/RelativeEndEffectorTask.h>

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

namespace mc_control
{

namespace fsm
{

/** Implements a state that is able to slide a foot contact */

struct MC_CONTROL_FSM_STATE_DLLAPI SlidingFootContactState : State
{
  virtual ~SlidingFootContactState() = default;

  void configure(const mc_rtc::Configuration & config) override;

  void start(Controller &) override;

  bool run(Controller &) override;

  void teardown(Controller &) override;

protected:
  /** Configuration */
  bool kinematic_ = false;
  std::string slidingSurface_;
  std::string supportSurface_;
  std::string handSurface_;
  double slidingForceTarget_ = 50.0;
  double supportForceTarget_ = 0.0;
  double handForceTarget_ = 20.0;
  unsigned int tickSupport_ = 300;
  unsigned int tickAdjust_ = 300;
  Eigen::Vector2d move_;
  double com_init_z_ = -1.0;
  Eigen::Vector3d com_offset_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d com_offset_sliding_ = Eigen::Vector3d::Zero();
  double move_com_z_ = 0;
  double rot_angle_ = -2.5 * M_PI / 180;
  bool wait_for_slide_trigger_ = false;
  bool slide_triggered_ = true;
  std::string next_ = "";
  /** Implementation */
  std::shared_ptr<mc_tasks::CoMTask> comTask_;
  std::shared_ptr<mc_tasks::force::CoPTask> copHandTask_;
  std::shared_ptr<mc_tasks::force::CoPTask> copSlidingFootTask_;
  std::shared_ptr<mc_tasks::force::CoPTask> copSupportFootTask_; /** Only used for CoP computation */
  std::shared_ptr<mc_tasks::OrientationTask> chestOriTask_;
  std::shared_ptr<mc_tasks::RelativeEndEffectorTask> lhRelEf_;
  std::shared_ptr<mc_tasks::RelativeEndEffectorTask> rhRelEf_;
  double mg_ = 0.0;
  tasks::qp::ContactId slidingContactId_;
  /** Internal phases */
  enum class Phase
  {
    REACH_SUPPORT,
    ADJUST_SLIDING_FORCE,
    SLIDE_FOOT,
    BALANCE,
    REGULATE_FOOT_ORIENTATION
  };
  Phase phase_ = Phase::REACH_SUPPORT;
  unsigned int tick_ = 0;
  bool forceDistChanged_ = true;
  Eigen::Vector3d initial_com = Eigen::Vector3d::Zero();
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
