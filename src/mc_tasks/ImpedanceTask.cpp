/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/configuration_io.h>
#include <mc_tasks/ImpedanceTask.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/gui/ArrayLabel.h>
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
                             double impM,
                             double impD,
                             double impK,
                             double oriScale)
: SurfaceTransformTask(surfaceName, robots, robotIndex, stiffness, weight), lowPass_(0.005, 0.05)
{
  const auto & robot = robots.robot(robotIndex);
  type_ = "impedance";
  name_ = "impedance_" + robots.robot(rIndex).name() + "_" + surfaceName;

  if(!robot.surfaceHasIndirectForceSensor(surfaceName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Surface {} does not have a force sensor attached", name_,
                                                     surfaceName);
  }

  // Set impedance parameters
  impedance(sva::ForceVecd(Eigen::Vector3d::Constant(oriScale * impM), Eigen::Vector3d::Constant(impM)),
            sva::ForceVecd(Eigen::Vector3d::Constant(oriScale * impD), Eigen::Vector3d::Constant(impD)),
            sva::ForceVecd(Eigen::Vector3d::Constant(oriScale * impK), Eigen::Vector3d::Constant(impK)));
}

void ImpedanceTask::update(mc_solver::QPSolver & solver)
{
  // 1. Filter the measured wrench
  if(lowPass_.dt() != solver.dt())
  {
    lowPass_.dt(solver.dt());
  }
  measuredWrench_ = robots.robot(rIndex).surfaceWrench(surfaceName);
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
          impM_.vector().cwiseInverse().cwiseProduct(
              // T_0_s transforms the MotionVecd value from world to surface frame
              -impD_.vector().cwiseProduct((T_0_s * deltaCompVelW_).vector())
              - impK_.vector().cwiseProduct((T_0_s * sva::transformVelocity(deltaCompPoseW_)).vector())
              + wrenchGain_.vector().cwiseProduct((filteredMeasuredWrench_ - targetWrench_).vector()))));

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

  // 4. Set compliance values to the targets of SurfaceTransformTask
  refAccel(T_0_s * (desiredAccelW_ + deltaCompAccelW_)); // represented in the surface frame
  refVelB(T_0_s * (desiredVelW_ + deltaCompVelW_)); // represented in the surface frame
  target(compliancePose()); // represented in the world frame
}

void ImpedanceTask::reset()
{
  // Set the target pose of SurfaceTransformTask to the current pose
  // Reset the target velocity and acceleration of SurfaceTransformTask to zero
  SurfaceTransformTask::reset();

  // Set the desired and compliance poses to the SurfaceTransformTask target (i.e., the current pose)
  desiredPoseW_ = target();
  deltaCompPoseW_ = sva::PTransformd::Identity();

  // Reset the desired and compliance velocity and acceleration to zero
  desiredVelW_ = sva::MotionVecd::Zero();
  desiredAccelW_ = sva::MotionVecd::Zero();
  deltaCompVelW_ = sva::MotionVecd::Zero();
  deltaCompAccelW_ = sva::MotionVecd::Zero();

  // Reset the target wrench to zero
  targetWrench_ = sva::ForceVecd::Zero();
  measuredWrench_ = sva::ForceVecd::Zero();
  filteredMeasuredWrench_ = sva::ForceVecd::Zero();
  lowPass_.reset(sva::ForceVecd::Zero());
}

void ImpedanceTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("impedanceM"))
  {
    impedanceM(config("impedanceM"));
  }
  if(config.has("impedanceD"))
  {
    impedanceD(config("impedanceD"));
  }
  if(config.has("impedanceK"))
  {
    impedanceK(config("impedanceK"));
  }
  if(config.has("wrenchGain"))
  {
    wrenchGain(config("wrenchGain"));
  }

  if(config.has("target"))
  {
    desiredPose(config("target"));
  }

  if(config.has("wrench"))
  {
    targetWrench(config("wrench"));
  }

  if(config.has("cutoffPeriod"))
  {
    cutoffPeriod(config("cutoffPeriod"));
  }

  SurfaceTransformTask::load(solver, config);
}

void ImpedanceTask::addToLogger(mc_rtc::Logger & logger)
{
  SurfaceTransformTask::addToLogger(logger);

  // impedance parameters
  logger.addLogEntry(name_ + "_impedanceM", [this]() -> const sva::ForceVecd & { return impM_; });
  logger.addLogEntry(name_ + "_impedanceD", [this]() -> const sva::ForceVecd & { return impD_; });
  logger.addLogEntry(name_ + "_impedanceK", [this]() -> const sva::ForceVecd & { return impK_; });
  logger.addLogEntry(name_ + "_wrenchGain", [this]() -> const sva::MotionVecd & { return wrenchGain_; });

  // compliance values
  logger.addLogEntry(name_ + "_deltaCompliancePose", [this]() -> const sva::PTransformd & { return deltaCompPoseW_; });
  logger.addLogEntry(name_ + "_deltaComplianceVel", [this]() -> const sva::MotionVecd & { return deltaCompVelW_; });
  logger.addLogEntry(name_ + "_deltaComplianceAccel", [this]() -> const sva::MotionVecd & { return deltaCompAccelW_; });

  // desired values
  logger.addLogEntry(name_ + "_desiredPose", [this]() -> const sva::PTransformd & { return desiredPoseW_; });
  logger.addLogEntry(name_ + "_desiredVel", [this]() -> const sva::MotionVecd & { return desiredVelW_; });
  logger.addLogEntry(name_ + "_desiredAccel", [this]() -> const sva::MotionVecd & { return desiredAccelW_; });

  // wrench
  logger.addLogEntry(name_ + "_targetWrench", [this]() -> const sva::ForceVecd & { return targetWrench_; });
  logger.addLogEntry(name_ + "_measuredWrench", [this]() -> const sva::ForceVecd & { return measuredWrench_; });
  logger.addLogEntry(name_ + "_filteredMeasuredWrench",
                     [this]() -> const sva::ForceVecd & { return filteredMeasuredWrench_; });
  logger.addLogEntry(name_ + "_cutoffPeriod", [this]() { return cutoffPeriod(); });
}

void ImpedanceTask::removeFromLogger(mc_rtc::Logger & logger)
{
  SurfaceTransformTask::removeFromLogger(logger);

  // impedance parameters
  logger.removeLogEntry(name_ + "_impedanceM");
  logger.removeLogEntry(name_ + "_impedanceD");
  logger.removeLogEntry(name_ + "_impedanceK");
  logger.removeLogEntry(name_ + "_wrenchGain");

  // compliance values
  logger.removeLogEntry(name_ + "_deltaCompliancePose");
  logger.removeLogEntry(name_ + "_deltaComplianceVel");
  logger.removeLogEntry(name_ + "_deltaComplianceAccel");

  // desired values
  logger.removeLogEntry(name_ + "_desiredPose");
  logger.removeLogEntry(name_ + "_desiredVel");
  logger.removeLogEntry(name_ + "_desiredAccel");

  // wrench
  logger.removeLogEntry(name_ + "_targetWrench");
  logger.removeLogEntry(name_ + "_measuredWrench");
  logger.removeLogEntry(name_ + "_filteredMeasuredWrench");
  logger.removeLogEntry(name_ + "_cutoffPeriod");
}

void ImpedanceTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  // Don't add SurfaceTransformTask because the target of SurfaceTransformTask should not be set by user
  TrajectoryTaskGeneric<tasks::qp::SurfaceTransformTask>::addToGUI(gui);

  gui.addElement({"Tasks", name_},
                 // pose
                 mc_rtc::gui::Transform("desiredPose",
                                        [this]() -> const sva::PTransformd & { return this->desiredPose(); },
                                        [this](const sva::PTransformd & pos) { this->desiredPose(pos); }),
                 mc_rtc::gui::Transform("compliancePose", [this]() { return this->compliancePose(); }),
                 mc_rtc::gui::Transform("pose", [this]() { return this->surfacePose(); }),
                 // impedance parameters
                 mc_rtc::gui::ArrayInput("impedanceM", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->impedanceM().vector(); },
                                         [this](const Eigen::Vector6d & a) { this->impedanceM(a); }),
                 mc_rtc::gui::ArrayInput("impedanceD", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->impedanceD().vector(); },
                                         [this](const Eigen::Vector6d & a) { this->impedanceD(a); }),
                 mc_rtc::gui::ArrayInput("impedanceK", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->impedanceK().vector(); },
                                         [this](const Eigen::Vector6d & a) { this->impedanceK(a); }),
                 mc_rtc::gui::ArrayInput("wrenchGain", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->wrenchGain().vector(); },
                                         [this](const Eigen::Vector6d & a) { this->wrenchGain(a); }),
                 // wrench
                 mc_rtc::gui::ArrayInput("targetWrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->targetWrench().vector(); },
                                         [this](const Eigen::Vector6d & a) { this->targetWrench(a); }),
                 mc_rtc::gui::ArrayLabel("measuredWrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->measuredWrench_.vector(); }),
                 mc_rtc::gui::ArrayLabel("filteredMeasuredWrench", {"cx", "cy", "cz", "fx", "fy", "fz"},
                                         [this]() { return this->filteredMeasuredWrench_.vector(); }),
                 mc_rtc::gui::NumberInput("cutoffPeriod", [this]() { return this->cutoffPeriod(); },
                                          [this](double a) { return this->cutoffPeriod(a); }));
}

} // namespace force

} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "impedance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      const auto robotIndex = robotIndexFromConfig(config, solver.robots(), "impedance");
      auto t = std::make_shared<mc_tasks::force::ImpedanceTask>(config("surface"), solver.robots(), robotIndex);
      t->reset();
      t->load(solver, config);
      return t;
    });
}
