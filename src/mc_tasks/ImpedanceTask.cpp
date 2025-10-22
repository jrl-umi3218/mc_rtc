/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/ImpedanceTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rbdyn/configuration_io.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/Transform.h>

namespace mc_tasks
{

namespace force
{

ImpedanceTask::ImpedanceTask(const std::string & surfaceName,
                             const mc_rbdyn::Robots & robots,
                             unsigned int robotIndex,
                             double stiffness,
                             double weight,
                             bool showTarget,
                             bool showPose,
                             bool showCompliance)
: ImpedanceTask(robots.robot(robotIndex).frame(surfaceName), stiffness, weight, showTarget, showPose, showCompliance)
{
}

ImpedanceTask::ImpedanceTask(const mc_rbdyn::RobotFrame & frame,
                             double stiffness,
                             double weight,
                             bool showTarget,
                             bool showPose,
                             bool showCompliance)
: TransformTask(frame, stiffness, weight), lowPass_(0.005, cutoffPeriod_)
{
  const auto & robot = frame.robot();
  type_ = "impedance";
  name_ = "impedance_" + robot.name() + "_" + frame.name();

  if(!frame.hasForceSensor())
  {
    mc_rtc::log::error_and_throw("[{}] Frame {} does not have a force sensor attached", name_, frame.name());
  }
  showTarget_ = showTarget;
  showPose_ = showPose;
  showCompliance_ = showCompliance;
}

void ImpedanceTask::update(mc_solver::QPSolver & solver)
{
  // 1. Filter the measured wrench
  measuredWrench_ = frame_->wrench();
  lowPass_.update(measuredWrench_);
  filteredMeasuredWrench_ = lowPass_.eval();

  // 2. Compute the compliance acceleration
  sva::PTransformd T_0_s(surfacePose().rotation());
  // deltaCompAccelW_ is represented in the world frame
  //   \Delta \ddot{p}_{cd} = - \frac{D}{M} \Delta \dot{p}_{cd} - \frac{K}{M} \Delta p_{cd})
  //   + \frac{K_f}{M} (f_m - f_d) where \Delta p_{cd} = p_c - p_d
  // See the Constructor description for the definition of symbols
  deltaCompAccelW_ = T_0_s.invMul( // T_0_s.invMul transforms the MotionVecd value from surface to world frame
      sva::MotionVecd(
          // Compute in the surface frame because the impedance parameters and wrench gain are represented in the
          // surface frame
          gains().M().vector().cwiseInverse().cwiseProduct(
              // T_0_s transforms the MotionVecd value from world to surface frame
              -gains().D().vector().cwiseProduct((T_0_s * deltaCompVelW_).vector())
              - gains().K().vector().cwiseProduct((T_0_s * sva::transformVelocity(deltaCompPoseW_)).vector())
              + gains().wrench().vector().cwiseProduct((filteredMeasuredWrench_ - targetWrench_).vector()))));

  if(deltaCompAccelW_.linear().norm() > deltaCompAccelLinLimit_)
  {
    mc_rtc::log::warning("linear deltaCompAccel limited from {} to {}", deltaCompAccelW_.linear().norm(),
                         deltaCompAccelLinLimit_);
    deltaCompAccelW_.linear().normalize();
    deltaCompAccelW_.linear() *= deltaCompAccelLinLimit_;
  }
  if(deltaCompAccelW_.angular().norm() > deltaCompAccelAngLimit_)
  {
    mc_rtc::log::warning("angular deltaCompAccel limited from {} to {}", deltaCompAccelW_.angular().norm(),
                         deltaCompAccelAngLimit_);
    deltaCompAccelW_.angular().normalize();
    deltaCompAccelW_.angular() *= deltaCompAccelAngLimit_;
  }

  // 3. Compute the compliance pose and velocity by time integral
  double dt = solver.dt();
  // 3.1 Integrate velocity to pose
  sva::PTransformd T_0_deltaC(deltaCompPoseW_.rotation());
  // Represent the compliance velocity and acceleration in the deltaCompliance frame and scale by dt
  sva::MotionVecd mvDeltaCompVelIntegralC = T_0_deltaC * (dt * (deltaCompVelW_ + 0.5 * dt * deltaCompAccelW_));
  // Convert the angular velocity to the rotation matrix through AngleAxis representation
  Eigen::AngleAxisd aaDeltaCompVelIntegralC(Eigen::Quaterniond::Identity());
  if(mvDeltaCompVelIntegralC.angular().norm() > 1e-6)
  {
    aaDeltaCompVelIntegralC =
        Eigen::AngleAxisd(mvDeltaCompVelIntegralC.angular().norm(), mvDeltaCompVelIntegralC.angular().normalized());
  }
  sva::PTransformd deltaCompVelIntegral(
      // Rotation matrix is transposed because sva::PTransformd uses the left-handed coordinates
      aaDeltaCompVelIntegralC.toRotationMatrix().transpose(), mvDeltaCompVelIntegralC.linear());
  // Since deltaCompVelIntegral is multiplied by deltaCompPoseW_, it must be represented in the deltaCompliance frame
  deltaCompPoseW_ = deltaCompVelIntegral * deltaCompPoseW_;
  // 3.2 Integrate acceleration to velocity
  deltaCompVelW_ += dt * deltaCompAccelW_;

  if(deltaCompVelW_.linear().norm() > deltaCompVelLinLimit_)
  {
    mc_rtc::log::warning("linear deltaCompVel limited from {} to {}", deltaCompVelW_.linear().norm(),
                         deltaCompVelLinLimit_);
    deltaCompVelW_.linear().normalize();
    deltaCompVelW_.linear() *= deltaCompVelLinLimit_;
  }
  if(deltaCompVelW_.angular().norm() > deltaCompVelAngLimit_)
  {
    mc_rtc::log::warning("angular deltaCompVel limited from {} to {}", deltaCompVelW_.angular().norm(),
                         deltaCompVelLinLimit_);
    deltaCompVelW_.angular().normalize();
    deltaCompVelW_.angular() *= deltaCompVelAngLimit_;
  }

  if(deltaCompPoseW_.translation().norm() > deltaCompPoseLinLimit_)
  {
    mc_rtc::log::warning("linear deltaCompPose limited from {} to {}", deltaCompPoseW_.translation().norm(),
                         deltaCompPoseLinLimit_);
    deltaCompPoseW_.translation().normalize();
    deltaCompPoseW_.translation() *= deltaCompPoseLinLimit_;
  }
  Eigen::AngleAxisd aaDeltaCompRot(deltaCompPoseW_.rotation());
  if(aaDeltaCompRot.angle() > deltaCompPoseAngLimit_)
  {
    mc_rtc::log::warning("angular deltaCompPose limited from {} to {}", aaDeltaCompRot.angle(), deltaCompPoseAngLimit_);
    aaDeltaCompRot.angle() = deltaCompPoseAngLimit_;
    deltaCompPoseW_.rotation() = aaDeltaCompRot.toRotationMatrix();
  }

  // 4. Update deltaCompPoseW_ in hold mode (See the hold method documentation for more information)
  if(hold_)
  {
    // Transform to target pose frame (see compliancePose implementation)
    sva::PTransformd T_0_d(targetPoseW_.rotation());
    // The previous compliancePose() is stored in SurfaceTransformTask::target()
    deltaCompPoseW_ = T_0_d.inv() * TransformTask::target() * targetPoseW_.inv() * T_0_d;
  }

  // 5. Set compliance values to the targets of SurfaceTransformTask
  refAccel(T_0_s * (targetAccelW_ + deltaCompAccelW_)); // represented in the surface frame
  TransformTask::refVelB(T_0_s * (targetVelW_ + deltaCompVelW_)); // represented in the surface frame
  TransformTask::target(compliancePose()); // represented in the world frame
}

void ImpedanceTask::reset()
{
  // Set the target pose of SurfaceTransformTask to the current pose
  // Reset the target velocity and acceleration of SurfaceTransformTask to zero
  TransformTask::reset();

  // Set the target and compliance poses to the SurfaceTransformTask target (i.e., the current pose)
  targetPoseW_ = TransformTask::target();
  deltaCompPoseW_ = sva::PTransformd::Identity();

  // Reset the target and compliance velocity and acceleration to zero
  targetVelW_ = sva::MotionVecd::Zero();
  targetAccelW_ = sva::MotionVecd::Zero();
  deltaCompVelW_ = sva::MotionVecd::Zero();
  deltaCompAccelW_ = sva::MotionVecd::Zero();

  // Reset the target wrench to zero
  targetWrench_ = sva::ForceVecd::Zero();
  measuredWrench_ = sva::ForceVecd::Zero();
  filteredMeasuredWrench_ = sva::ForceVecd::Zero();
  lowPass_.reset(sva::ForceVecd::Zero());

  // Reset hold
  hold_ = false;
}

void ImpedanceTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("gains")) { gains_ = config("gains"); }
  if(config.has("wrench")) { targetWrench(config("wrench")); }
  if(config.has("cutoffPeriod")) { cutoffPeriod(config("cutoffPeriod")); }
  if(auto deltaCompPoseLinLimit = config.find("deltaCompPoseLinLimit"))
  {
    deltaCompPoseLinLimit_ = std::max<double>(*deltaCompPoseLinLimit, 0.0);
  }
  if(auto deltaCompPoseAngLimit = config.find("deltaCompPoseAngLimit"))
  {
    deltaCompPoseAngLimit_ = std::max<double>(*deltaCompPoseAngLimit, 0.0);
  }
  if(auto deltaCompVelLinLimit = config.find("deltaCompVelLinLimit"))
  {
    deltaCompVelLinLimit_ = std::max<double>(*deltaCompVelLinLimit, 0.0);
  }
  if(auto deltaCompVelAngLimit = config.find("deltaCompVelAngLimit"))
  {
    deltaCompVelAngLimit_ = std::max<double>(*deltaCompVelAngLimit, 0.0);
  }
  if(auto deltaCompAccelLinLimit = config.find("deltaCompAccelLinLimit"))
  {
    deltaCompAccelLinLimit_ = std::max<double>(*deltaCompAccelLinLimit, 0.0);
  }
  if(auto deltaCompAccelAngLimit = config.find("deltaCompAccelAngLimit"))
  {
    deltaCompAccelAngLimit_ = std::max<double>(*deltaCompAccelAngLimit, 0.0);
  }
  TransformTask::load(solver, config);
  // The TransformTask::load function above only sets
  // the TrajectoryTaskGeneric's target, but not the compliance target, so we
  // need to set it manually here.
  targetPose(TransformTask::target());
}

void ImpedanceTask::addToSolver(mc_solver::QPSolver & solver)
{
  lowPass_.dt(solver.dt());
  cutoffPeriod(cutoffPeriod_);
  TransformTask::addToSolver(solver);
}

void ImpedanceTask::addToLogger(mc_rtc::Logger & logger)
{
  TransformTask::addToLogger(logger);

  // impedance parameters
  logger.addLogEntry(name_ + "_gains_M", this, [this]() -> const sva::ImpedanceVecd & { return gains().M().vec(); });
  logger.addLogEntry(name_ + "_gains_D", this, [this]() -> const sva::ImpedanceVecd & { return gains().D().vec(); });
  logger.addLogEntry(name_ + "_gains_K", this, [this]() -> const sva::ImpedanceVecd & { return gains().K().vec(); });
  logger.addLogEntry(name_ + "_gains_wrench", this,
                     [this]() -> const sva::ImpedanceVecd & { return gains().wrench().vec(); });

  // compliance values
  logger.addLogEntry(name_ + "_compliancePose", this, [this]() { return compliancePose(); });
  MC_RTC_LOG_HELPER(name_ + "_deltaCompliancePose", deltaCompPoseW_);
  MC_RTC_LOG_HELPER(name_ + "_deltaComplianceVel", deltaCompVelW_);
  MC_RTC_LOG_HELPER(name_ + "_deltaComplianceAccel", deltaCompAccelW_);

  // target values
  MC_RTC_LOG_HELPER(name_ + "_targetPose", targetPoseW_);
  MC_RTC_LOG_HELPER(name_ + "_targetVel", targetVelW_);
  MC_RTC_LOG_HELPER(name_ + "_targetAccel", targetAccelW_);

  // wrench
  MC_RTC_LOG_HELPER(name_ + "_targetWrench", targetWrench_);
  MC_RTC_LOG_HELPER(name_ + "_measuredWrench", measuredWrench_);
  MC_RTC_LOG_HELPER(name_ + "_filteredMeasuredWrench", filteredMeasuredWrench_);
  logger.addLogEntry(name_ + "_cutoffPeriod", this, [this]() { return cutoffPeriod(); });

  MC_RTC_LOG_HELPER(name_ + "_hold", hold_);
}

void ImpedanceTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  // Don't add TransformTask because the target of TransformTask should not be set by user
  TrajectoryTaskGeneric::addToGUI(gui);

  auto showTarget = [this, &gui]()
  {
    if(showTarget_)
    {
      gui.addElement({"Tasks", name_},
                     mc_rtc::gui::Transform(
                         "targetPose", [this]() -> const sva::PTransformd & { return this->targetPose(); },
                         [this](const sva::PTransformd & pos) { this->targetPose(pos); }));
    }
    else
    {
      gui.removeElement({"Tasks", name_}, "targetPose");
    }
  };

  auto showCompliance = [this, &gui]()
  {
    if(showCompliance_)
    {
      gui.addElement({"Tasks", name_},
                     mc_rtc::gui::Transform("compliancePose", [this]() { return this->compliancePose(); }));
    }
    else
    {
      gui.removeElement({"Tasks", name_}, "compliancePose");
    }
  };

  auto showPose = [this, &gui]()
  {
    if(showPose_)
    {
      gui.addElement({"Tasks", name_}, mc_rtc::gui::Transform("pose", [this]() { return this->surfacePose(); }));
    }
    else
    {
      gui.removeElement({"Tasks", name_}, "pose");
    }
  };

  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Checkbox(
                     "Show target frame", [this]() { return showTarget_; },
                     [this, showTarget]()
                     {
                       showTarget_ = !showTarget_;
                       showTarget();
                     }),
                 mc_rtc::gui::Checkbox(
                     "Show pose frame", [this]() { return showPose_; },
                     [this, showPose]()
                     {
                       showPose_ = !showPose_;
                       showPose();
                     }),
                 mc_rtc::gui::Checkbox(
                     "Show compliance frame", [this]() { return showCompliance_; },
                     [this, showCompliance]()
                     {
                       showCompliance_ = !showCompliance_;
                       showCompliance();
                     }));

  showPose();
  showTarget();
  showCompliance();
  gui.addElement({"Tasks", name_},
                 // wrench
                 mc_rtc::gui::ArrayInput(
                     "targetWrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                     [this]() { return this->targetWrench().vector(); },
                     [this](const Eigen::Vector6d & a) { this->targetWrench(a); }),
                 mc_rtc::gui::ArrayLabel("measuredWrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->measuredWrench_.vector(); }),
                 mc_rtc::gui::ArrayLabel("filteredMeasuredWrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->filteredMeasuredWrench_.vector(); }),
                 mc_rtc::gui::NumberInput(
                     "cutoffPeriod", [this]() { return this->cutoffPeriod(); },
                     [this](double a) { return this->cutoffPeriod(a); }),
                 mc_rtc::gui::Checkbox("hold", [this]() { return hold_; }, [this]() { hold_ = !hold_; }));
  gui.addElement(
      {"Tasks", name_, "Impedance gains"},
      mc_rtc::gui::ArrayInput(
          "mass", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() -> const sva::ImpedanceVecd &
          { return gains().mass().vec(); }, [this](const Eigen::Vector6d & a) { gains().mass().vec(a); }),
      mc_rtc::gui::ArrayInput(
          "damper", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() -> const sva::ImpedanceVecd &
          { return gains().damper().vec(); }, [this](const Eigen::Vector6d & a) { gains().damper().vec(a); }),
      mc_rtc::gui::ArrayInput(
          "spring", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() -> const sva::ImpedanceVecd &
          { return gains().spring().vec(); }, [this](const Eigen::Vector6d & a) { gains().spring().vec(a); }),
      mc_rtc::gui::ArrayInput(
          "wrench", {"cx", "cy", "cz", "fx", "fy", "fz"}, [this]() -> const sva::ImpedanceVecd &
          { return gains().wrench().vec(); }, [this](const Eigen::Vector6d & a) { gains().wrench().vec(a); }));
}

} // namespace force

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "impedance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      using Allocator = Eigen::aligned_allocator<mc_tasks::force::ImpedanceTask>;
      const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "impedance");
      const auto & robot = solver.robots().robot(robotIndex);
      const auto & frame = [&]() -> const mc_rbdyn::RobotFrame &
      {
        if(config.has("surface"))
        {
          mc_rtc::log::deprecated("ImpedanceTask", "surface", "frame");
          return robot.frame(config("surface"));
        }
        return robot.frame(config("frame"));
      }();
      auto t = std::allocate_shared<mc_tasks::force::ImpedanceTask>(Allocator{}, frame);
      t->reset();
      t->load(solver, config);
      return t;
    });
} // namespace
