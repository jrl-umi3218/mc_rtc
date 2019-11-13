/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/SplineTrajectoryTask.h>

#include <mc_rtc/gui/Rotation.h>
#include <mc_rtc/gui/Transform.h>

namespace mc_tasks
{
template<typename Derived>
SplineTrajectoryTask<Derived>::SplineTrajectoryTask(const mc_rbdyn::Robots & robots,
                                                    unsigned int robotIndex,
                                                    const std::string & surfaceName,
                                                    double duration,
                                                    double stiffness,
                                                    double weight,
                                                    const Eigen::Matrix3d & target,
                                                    const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
: TrajectoryTaskGeneric<tasks::qp::TransformTask>(robots, robotIndex, stiffness, weight), rIndex_(robotIndex),
  surfaceName_(surfaceName), duration_(duration),
  oriSpline_(duration,
             robots.robot(robotIndex).surface(surfaceName).X_0_s(robots.robot(robotIndex)).rotation(),
             target,
             oriWp)
{
  const auto & robot = robots.robot(robotIndex);
  const auto & surface = robot.surface(surfaceName);
  type_ = "trajectory";
  name_ = "trajectory_" + robot.name() + "_" + surface.name();

  finalize(robots.mbs(), static_cast<int>(robotIndex), surface.bodyName(), surface.X_0_s(robot), surface.X_b_s());
}

template<typename Derived>
std::function<bool(const mc_tasks::MetaTask &, std::string &)> SplineTrajectoryTask<Derived>::buildCompletionCriteria(
    double dt,
    const mc_rtc::Configuration & config) const
{

  if(config.has("timeElapsed"))
  {
    bool useDuration = config("timeElapsed");
    if(useDuration)
    {
      return [](const mc_tasks::MetaTask & t, std::string & out) {
        const auto & self = static_cast<const SplineTrajectoryBase &>(t);
        out += "duration";
        return self.timeElapsed();
      };
    }
  }
  return TrajectoryBase::buildCompletionCriteria(dt, config);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::update()
{
  auto & spline = static_cast<Derived &>(*this).spline();
  spline.samplingPoints(samples_);
  spline.update();

  // Interpolate position
  auto res = spline.splev(currTime_, 2);
  Eigen::Vector3d & pos = res[0];
  Eigen::Vector3d & vel = res[1];
  Eigen::Vector3d & acc = res[2];

  // Interpolate orientation
  Eigen::Matrix3d ori_target = oriSpline_.eval(currTime_);
  sva::PTransformd target(ori_target, pos);

  // Set the trajectory tracking task targets from the trajectory.
  Eigen::VectorXd refVel(6);
  Eigen::VectorXd refAcc(6);
  refVel.head<3>() = Eigen::Vector3d::Zero();
  refVel.tail<3>() = vel;
  refAcc.head<3>() = Eigen::Vector3d::Zero();
  refAcc.tail<3>() = acc;
  this->refVel(refVel);
  this->refAccel(refAcc);
  this->refPose(target);

  currTime_ = std::min(currTime_ + timeStep_, duration_);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::oriWaypoints(const std::vector<std::pair<double, Eigen::Matrix3d>> & oriWp)
{
  oriSpline_.waypoints(oriWp);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::dimWeight(const Eigen::VectorXd & dimW)
{
  if(dimW.size() != 6)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "SplineTrajectoryTask dimWeight should be a Vector6d!");
  }
  TrajectoryBase::dimWeight(dimW);
}

template<typename Derived>
Eigen::VectorXd SplineTrajectoryTask<Derived>::dimWeight() const
{
  return TrajectoryBase::dimWeight();
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::target(const sva::PTransformd & target)
{
  auto & derived = static_cast<Derived &>(*this);
  derived.target(target.translation());
  oriSpline_.target(target.rotation());
}

template<typename Derived>
const sva::PTransformd SplineTrajectoryTask<Derived>::target() const
{
  const auto & derived = static_cast<const Derived &>(*this);
  return sva::PTransformd(oriSpline_.target(), derived.target());
}

template<typename Derived>
Eigen::VectorXd SplineTrajectoryTask<Derived>::eval() const
{
  const auto & robot = robots.robot(rIndex_);
  sva::PTransformd X_0_s = robot.surface(surfaceName_).X_0_s(robot);
  return sva::transformError(X_0_s, target()).vector();
}

template<typename Derived>
Eigen::VectorXd SplineTrajectoryTask<Derived>::evalTracking() const
{
  return TrajectoryBase::eval();
}

template<typename Derived>
bool SplineTrajectoryTask<Derived>::timeElapsed() const
{
  return currTime_ >= duration_;
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::displaySamples(unsigned s)
{
  samples_ = s;
}

template<typename Derived>
unsigned SplineTrajectoryTask<Derived>::displaySamples() const
{
  return samples_;
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::refPose(const sva::PTransformd & target)
{
  errorT->target(target);
}

template<typename Derived>
const sva::PTransformd & SplineTrajectoryTask<Derived>::refPose() const
{
  return errorT->target();
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::addToSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver_)
  {
    timeStep_ = solver.dt();
  }
  TrajectoryBase::addToSolver(solver);
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_surfacePose", [this]() {
    const auto & robot = this->robots.robot(rIndex_);
    return robot.surfacePose(surfaceName_);
  });
  logger.addLogEntry(name_ + "_targetPose", [this]() { return this->target(); });
  logger.addLogEntry(name_ + "_refPose", [this]() { return this->refPose(); });
  logger.addLogEntry(name_ + "_speed", [this]() -> const Eigen::VectorXd { return this->speed(); });
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::removeFromLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::removeFromLogger(logger);
  logger.removeLogEntry(name_ + "_surfacePose");
  logger.removeLogEntry(name_ + "_targetPose");
  logger.removeLogEntry(name_ + "_refPose");
  logger.removeLogEntry(name_ + "_speed");
}

template<typename Derived>
void SplineTrajectoryTask<Derived>::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTask::addToGUI(gui);

  auto & spline = static_cast<Derived &>(*this).spline();
  gui.addElement({"Tasks", name_}, mc_rtc::gui::Transform("Surface pose", [this]() {
                   const auto & robot = this->robots.robot(rIndex_);
                   return robot.surface(surfaceName_).X_0_s(robot);
                 }));

  gui.addElement({"Tasks", name_}, mc_rtc::gui::Rotation("Target Rotation", [this]() { return this->target(); },
                                                         [this](const Eigen::Quaterniond & ori) {
                                                           sva::PTransformd X_0_t(ori, this->target().translation());
                                                           this->target(X_0_t);
                                                         }));

  // Target rotation is handled independently
  for(unsigned i = 0; i < oriSpline_.waypoints().size(); ++i)
  {
    gui.addElement({"Tasks", name_, "Orientation Waypoint"},
                   mc_rtc::gui::Rotation("Waypoint " + std::to_string(i),
                                         [this, i, &spline]() {
                                           // Get position of orientation waypoint along the spline
                                           const auto & wp = this->oriSpline_.waypoint(i);
                                           return sva::PTransformd(wp.second, spline.splev(wp.first, 0)[0]);
                                         },
                                         [this, i](const Eigen::Quaterniond & ori) {
                                           this->oriSpline_.waypoint(i, ori.toRotationMatrix());
                                         }));
  }
}

} // namespace mc_tasks
