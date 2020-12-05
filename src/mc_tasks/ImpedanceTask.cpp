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
                             double impK)
: SurfaceTransformTask(surfaceName, robots, robotIndex, stiffness, weight)
{
  const auto & robot = robots.robot(robotIndex);
  if(!robot.surfaceHasIndirectForceSensor(surfaceName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_tasks::ImpedanceTask] Surface {} does not have a force sensor attached", surfaceName);
  }

  type_ = "impedance";
  name_ = "impedance_" + robots.robot(rIndex).name() + "_" + surfaceName;

  // Set impedance parameters
  impedance(sva::ForceVecd(Eigen::Vector3d::Constant(10.0 * impM), Eigen::Vector3d::Constant(impM)),
            sva::ForceVecd(Eigen::Vector3d::Constant(10.0 * impD), Eigen::Vector3d::Constant(impD)),
            sva::ForceVecd(Eigen::Vector3d::Constant(10.0 * impK), Eigen::Vector3d::Constant(impK)));
}

void ImpedanceTask::update(mc_solver::QPSolver & solver)
{
  // 1. Compute the compliance acceleration
  sva::PTransformd T_0_s(surfacePose().rotation());
  // compAccelW_ is represented in the world frame
  //   \ddot{p}_c = \ddot{p}_d + \frac{D}{M} (\dot{p}_d - \dot{p}_c) + \frac{K}{M} (p_d - p_c) + \frac{K_f}{M} (f_m -
  //   f_d) See the Constructor description for the definition of symbols
  compAccelW_ =
      desiredAccelW_
      + T_0_s.invMul( // T_0_s.invMul transforms the MotionVecd value from the surface frame to the world frame
            sva::MotionVecd(
                // Compute in the surface frame because the impedance parameters and wrench gain are represented in the
                // surface frame
                impM_.vector().cwiseInverse().cwiseProduct(
                    // T_0_s transforms the MotionVecd value from the world from to the surface frame
                    impD_.vector().cwiseProduct((T_0_s * (desiredVelW_ - compVelW_)).vector())
                    + impK_.vector().cwiseProduct((T_0_s * transformError(compPoseW_, desiredPoseW_)).vector())
                    + wrenchGain_.vector().cwiseProduct((measuredWrench() - targetWrench_).vector()))));

  // 2. Compute the compliance pose and velocity by time integral
  double dt = solver.dt();
  // 2.1 Integrate velocity to pose
  sva::PTransformd T_0_c(compPoseW_.rotation());
  // Represent the compliance velocity and acceleration in the compliance frame and scale by dt
  sva::MotionVecd compDeltaC = T_0_c * (dt * compVelW_ + 0.5 * dt * dt * compAccelW_);
  // Convert the angular velocity to the rotation matrix through AngleAxis representation
  Eigen::AngleAxisd aaCompDeltaC(Eigen::Quaterniond::Identity());
  if(compDeltaC.angular().norm() > 1e-6)
  {
    aaCompDeltaC = Eigen::AngleAxisd(compDeltaC.angular().norm(), compDeltaC.angular().normalized());
  }
  sva::PTransformd compVelIntegral(
      // Rotation matrix is transposed because sva::PTransformd uses the left-handed coordinates
      aaCompDeltaC.toRotationMatrix().transpose(), compDeltaC.linear());
  // Since compVelIntegral is multiplied by compPoseW_, it must be represented in the compliance frame
  compPoseW_ = compVelIntegral * compPoseW_;
  // 2.2 Integrate acceleration to velocity
  compVelW_ += dt * compAccelW_;

  // 3. Set compliance values to the targets of SurfaceTransformTask
  refAccel(T_0_s * compAccelW_); // represented in the surface frame
  refVelB(T_0_s * compVelW_); // represented in the surface frame
  target(compPoseW_); // represented in the world frame
}

void ImpedanceTask::reset()
{
  // Set the target pose of SurfaceTransformTask to the current pose
  // Reset the target velocity and acceleration of SurfaceTransformTask to zero
  SurfaceTransformTask::reset();

  // Set the desired and compliance poses to the SurfaceTransformTask target (i.e., the current pose)
  desiredPoseW_ = target();
  compPoseW_ = desiredPoseW_;

  // Reset the desired and compliance velocity and acceleration to zero
  desiredVelW_ = sva::MotionVecd::Zero();
  desiredAccelW_ = sva::MotionVecd::Zero();
  compVelW_ = sva::MotionVecd::Zero();
  compAccelW_ = sva::MotionVecd::Zero();

  // Reset the target wrench to zero
  targetWrench_ = sva::ForceVecd::Zero();
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
  logger.addLogEntry(name_ + "_compliancePose", [this]() -> const sva::PTransformd & { return compPoseW_; });
  logger.addLogEntry(name_ + "_complianceVel", [this]() -> const sva::MotionVecd & { return compVelW_; });
  logger.addLogEntry(name_ + "_complianceAccel", [this]() -> const sva::MotionVecd & { return compAccelW_; });

  // desired values
  logger.addLogEntry(name_ + "_desiredPose", [this]() -> const sva::PTransformd & { return desiredPoseW_; });
  logger.addLogEntry(name_ + "_desiredVel", [this]() -> const sva::MotionVecd & { return desiredVelW_; });
  logger.addLogEntry(name_ + "_desiredAccel", [this]() -> const sva::MotionVecd & { return desiredAccelW_; });

  // wrench
  logger.addLogEntry(name_ + "_targetWrench", [this]() -> const sva::ForceVecd & { return targetWrench_; });
  logger.addLogEntry(name_ + "_measuredWrench",
                     [this]() -> sva::ForceVecd { return measuredWrench(); }); // should not be reference
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
  logger.removeLogEntry(name_ + "_compliancePose");
  logger.removeLogEntry(name_ + "_complianceVel");
  logger.removeLogEntry(name_ + "_complianceAccel");

  // desired values
  logger.removeLogEntry(name_ + "_desiredPose");
  logger.removeLogEntry(name_ + "_desiredVel");
  logger.removeLogEntry(name_ + "_desiredAccel");

  // wrench
  logger.removeLogEntry(name_ + "_targetWrench");
  logger.removeLogEntry(name_ + "_measuredWrench");
}

void ImpedanceTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  // Don't add SurfaceTransformTask because the target of SurfaceTransformTask should not be set by user
  TrajectoryTaskGeneric<tasks::qp::SurfaceTransformTask>::addToGUI(gui);

  gui.addElement({"Tasks", name_},
                 // pose
                 mc_rtc::gui::Transform("desiredPose", [this]() { return this->desiredPose(); },
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
                                         [this]() { return this->measuredWrench().vector(); }));
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
      t->load(solver, config);
      return t;
    });
}
