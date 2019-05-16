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
: robots(robots)
{
  this->rIndex = robotIndex;
  this->surfaceName = surfaceName;
  this->X_0_t = X_0_t;
  this->duration = duration;
  stiffness_ = stiffness;
  damping_ = 2 * sqrt(stiffness_);

  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  const auto & surface = robot.surface(surfaceName);
  type_ = "trajectory";
  name_ = "trajectory_" + robot.name() + "_" + surface.name();
  X_0_start = surface.X_0_s(robot);

  transTask.reset(new tasks::qp::TransformTask(robots.mbs(), static_cast<int>(robotIndex), surface.bodyName(),
                                               X_0_start, surface.X_b_s()));
  transTrajTask.reset(new tasks::qp::TrajectoryTask(robots.mbs(), static_cast<int>(robotIndex), transTask.get(),
                                                    stiffness, 2 * sqrt(stiffness), 1.0));
  posWeight(posW);
  oriWeight(oriW);
}

void TrajectoryTask::stiffness(double s)
{
  stiffness_ = s;
  setGains(s, 2 * std::sqrt(s));
}

void TrajectoryTask::damping(double d)
{
  damping_ = d;
  setGains(stiffness(), d);
}

void TrajectoryTask::setGains(double s, double d)
{
  transTrajTask->setGains(s, d);
}

double TrajectoryTask::stiffness() const
{
  return stiffness_;
}

double TrajectoryTask::damping() const
{
  return damping_;
}

void TrajectoryTask::posWeight(const double posWeight)
{
  auto dimWeight = transTrajTask->dimWeight();
  dimWeight.tail(3).setConstant(posWeight);
  transTrajTask->dimWeight(dimWeight);
}

double TrajectoryTask::posWeight() const
{
  return transTrajTask->dimWeight()(3);
}

void TrajectoryTask::oriWeight(const double oriWeight)
{
  auto dimWeight = transTrajTask->dimWeight();
  dimWeight.head(3).setConstant(oriWeight);
  transTrajTask->dimWeight(dimWeight);
}

double TrajectoryTask::oriWeight() const
{
  return transTrajTask->dimWeight()(0);
}

void TrajectoryTask::dimWeight(const Eigen::VectorXd & dimW)
{
  assert(dimW.size() == 6);
  transTrajTask->dimWeight(dimW);
}

Eigen::VectorXd TrajectoryTask::dimWeight() const
{
  return transTrajTask->dimWeight();
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
  return transTask->eval();
}

Eigen::VectorXd TrajectoryTask::speed() const
{
  return transTask->speed();
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

void TrajectoryTask::refVel(const Eigen::VectorXd & vel)
{
  transTrajTask->refVel(vel);
}

const Eigen::VectorXd & TrajectoryTask::refVel() const
{
  return transTrajTask->refVel();
}

void TrajectoryTask::refAcc(const Eigen::VectorXd & acc)
{
  transTrajTask->refAccel(acc);
}

const Eigen::VectorXd & TrajectoryTask::refAcc() const
{
  return transTrajTask->refAccel();
}
void TrajectoryTask::refPose(const sva::PTransformd & target)
{
  transTask->target(target);
}
const sva::PTransformd & TrajectoryTask::refPose() const
{
  return transTask->target();
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
    solver.addTask(transTrajTask.get());
    inSolver = true;
  }
}

void TrajectoryTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(inSolver)
  {
    solver.removeTask(transTrajTask.get());
    inSolver = false;
  }
}

void TrajectoryTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                        const std::vector<std::string> & activeJoints,
                                        const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }
  const auto stiff = stiffness();
  const auto damp = damping();
  const auto & dimW = dimWeight();
  selectorT = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::ActiveJoints(
      robots.mbs(), static_cast<int>(rIndex), transTask.get(), activeJoints, activeDofs));
  transTrajTask = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, selectorT.get(), stiff, damp, 1.0);
  transTrajTask->dimWeight(dimW);
  if(putBack)
  {
    addToSolver(solver);
  }
}

void TrajectoryTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                          const std::vector<std::string> & unactiveJoints,
                                          const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }
  const auto stiff = stiffness();
  const auto damp = damping();
  const auto & dimW = dimWeight();
  selectorT = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::UnactiveJoints(
      robots.mbs(), static_cast<int>(rIndex), transTask.get(), unactiveJoints, unactiveDofs));
  transTrajTask = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, selectorT.get(), stiff, damp, 1.0);
  transTrajTask->dimWeight(dimW);
  if(putBack)
  {
    addToSolver(solver);
  }
}

void TrajectoryTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }
  const auto stiff = stiffness();
  const auto damp = damping();
  const auto & dimW = dimWeight();
  selectorT = nullptr;
  transTrajTask = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, transTask.get(), stiff, damp, 1.0);
  transTrajTask->dimWeight(dimW);
  if(putBack)
  {
    addToSolver(solver);
  }
}

void TrajectoryTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_surface_pose", [this]() {
    const auto & robot = robots.robot(rIndex);
    return robot.surface(surfaceName).X_0_s(robot);
  });
  logger.addLogEntry(name_ + "_target_trajectory_pose", [this]() { return this->transTask->target(); });
  logger.addLogEntry(name_ + "_target_pose", [this]() { return this->X_0_t; });
  logger.addLogEntry(name_ + "_target_vel", [this]() { return sva::MotionVecd(this->transTrajTask->refVel()); });
  logger.addLogEntry(name_ + "_target_acc", [this]() { return sva::MotionVecd(this->transTrajTask->refAccel()); });
  logger.addLogEntry(name_ + "_speed", [this]() { return sva::MotionVecd(this->speed()); });
  logger.addLogEntry(name_ + "_normalAcc", [this]() { return sva::MotionVecd(this->transTask->normalAcc()); });
}

void TrajectoryTask::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_surface_pose");
  logger.removeLogEntry(name_ + "_target_trajectory_pose");
  logger.removeLogEntry(name_ + "_target_pose");
  logger.removeLogEntry(name_ + "_target_vel");
  logger.removeLogEntry(name_ + "_target_acc");
  logger.removeLogEntry(name_ + "_speed");
  logger.removeLogEntry(name_ + "_normalAcc");
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
