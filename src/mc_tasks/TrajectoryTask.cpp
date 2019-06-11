/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/Surface.h>
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/TrajectoryTask.h>
#include <mc_trajectory/spline_utils.h>

namespace mc_tasks
{
TrajectoryTask::TrajectoryTask(const mc_rbdyn::Robots & robots,
                               unsigned int robotIndex,
                               const std::string & surfaceName,
                               double duration,
                               double stiffness,
                               double posW,
                               double oriW)
: TrajectoryTaskGeneric<tasks::qp::TransformTask>(robots, robotIndex, stiffness, posW)
{
  this->rIndex = robotIndex;
  this->surfaceName = surfaceName;
  this->duration = duration;

  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  const auto & surface = robot.surface(surfaceName);
  type_ = "trajectory";
  name_ = "trajectory_" + robot.name() + "_" + surface.name();
  X_0_start = surface.X_0_s(robot);

  finalize(robots.mbs(), static_cast<int>(robotIndex), surface.bodyName(), X_0_start, surface.X_b_s());
  posWeight(posW);
  oriWeight(oriW);
}

void TrajectoryTask::posWeight(const double posWeight)
{
  auto dimWeight = trajectoryT_->dimWeight();
  dimWeight.tail(3).setConstant(posWeight);
  trajectoryT_->dimWeight(dimWeight);
}

double TrajectoryTask::posWeight() const
{
  return trajectoryT_->dimWeight()(3);
}

void TrajectoryTask::oriWeight(const double oriWeight)
{
  auto dimWeight = trajectoryT_->dimWeight();
  dimWeight.head(3).setConstant(oriWeight);
  trajectoryT_->dimWeight(dimWeight);
}

double TrajectoryTask::oriWeight() const
{
  return trajectoryT_->dimWeight()(0);
}

void TrajectoryTask::target(const sva::PTransformd & target)
{
  X_0_t = target;
}

const sva::PTransformd & TrajectoryTask::target() const
{
  return X_0_t;
}

Eigen::VectorXd TrajectoryTask::eval() const
{
  const auto & robot = robots.robot(rIndex);
  sva::PTransformd X_0_s = robot.surface(surfaceName).X_0_s(robot);
  return sva::transformError(X_0_s, X_0_t).vector();
}

Eigen::VectorXd TrajectoryTask::evalTracking() const
{
  return TrajectoryBase::eval();
}

bool TrajectoryTask::timeElapsed()
{
  return t >= duration;
}

void TrajectoryTask::displaySamples(unsigned s)
{
  samples_ = s;
}

unsigned TrajectoryTask::displaySamples() const
{
  return samples_;
}

void TrajectoryTask::refPose(const sva::PTransformd & target)
{
  errorT->target(target);
}

const sva::PTransformd & TrajectoryTask::refPose() const
{
  return errorT->target();
}

void TrajectoryTask::update()
{
  t = std::min(t + timeStep, duration);
}

void TrajectoryTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver)
  {
    timeStep = solver.dt();
  }
  TrajectoryBase::addToSolver(solver);
}

void TrajectoryTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_surface_pose", [this]() {
    const auto & robot = robots.robot(rIndex);
    return robot.surface(surfaceName).X_0_s(robot);
  });
  logger.addLogEntry(name_ + "_target_pose", [this]() { return this->X_0_t; });
  logger.addLogEntry(name_ + "_speed", [this]() { return sva::MotionVecd(this->speed()); });
}

void TrajectoryTask::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_target_pose");
  logger.removeLogEntry(name_ + "_speed");
}

void TrajectoryTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement({"Tasks", name_, "Gains"},
                 mc_rtc::gui::NumberInput("stiffness", [this]() { return this->stiffness(); },
                                          [this](const double & s) { this->setGains(s, this->damping()); }),
                 mc_rtc::gui::NumberInput("damping", [this]() { return this->damping(); },
                                          [this](const double & d) { this->setGains(this->stiffness(), d); }),
                 mc_rtc::gui::NumberInput("stiffness & damping", [this]() { return this->stiffness(); },
                                          [this](const double & g) { this->stiffness(g); }),
                 mc_rtc::gui::NumberInput("posWeight", [this]() { return this->posWeight(); },
                                          [this](const double & d) { this->posWeight(d); }),
                 mc_rtc::gui::NumberInput("oriWeight", [this]() { return this->oriWeight(); },
                                          [this](const double & g) { this->oriWeight(g); }));
}

} // namespace mc_tasks
