/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/EndEffectorTask.h>

#include <mc_rtc/gui/Transform.h>

namespace mc_tasks
{

EndEffectorTask::EndEffectorTask(const std::string & bodyName,
                                 const mc_rbdyn::Robots & robots,
                                 unsigned int robotIndex,
                                 double stiffness,
                                 double weight)
: EndEffectorTask(bodyName, Eigen::Vector3d::Zero(), robots, robotIndex, stiffness, weight)
{
}

EndEffectorTask::EndEffectorTask(const std::string & bodyName,
                                 const Eigen::Vector3d & bodyPoint,
                                 const mc_rbdyn::Robots & robots,
                                 unsigned int robotIndex,
                                 double stiffness,
                                 double weight)
: robots(robots), robotIndex(robotIndex), bodyName(bodyName)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  bodyIndex = robot.bodyIndexByName(bodyName);
  sva::PTransformd bpw = robot.mbc().bodyPosW[bodyIndex];

  curTransform = sva::PTransformd{bodyPoint} * bpw;

  positionTask.reset(new mc_tasks::PositionTask(bodyName, bodyPoint, robots, robotIndex, stiffness, weight));
  orientationTask.reset(new mc_tasks::OrientationTask(bodyName, robots, robotIndex, stiffness, weight));

  type_ = "body6d";
  name_ = "body6d_" + robot.name() + "_" + bodyName;
  positionTask->name(name_ + "_position");
  orientationTask->name(name_ + "_orientation");
}

void EndEffectorTask::reset()
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  curTransform = sva::PTransformd{positionTask->bodyPoint()} * robot.mbc().bodyPosW[bodyIndex];
  positionTask->position(curTransform.translation());
  orientationTask->orientation(curTransform.rotation());
}

void EndEffectorTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  MetaTask::removeFromSolver(*positionTask, solver);
  MetaTask::removeFromSolver(*orientationTask, solver);
}

void EndEffectorTask::addToSolver(mc_solver::QPSolver & solver)
{
  MetaTask::addToSolver(*positionTask, solver);
  MetaTask::addToSolver(*orientationTask, solver);
}

void EndEffectorTask::update(mc_solver::QPSolver & solver)
{
  MetaTask::update(*positionTask, solver);
  MetaTask::update(*orientationTask, solver);
}

void EndEffectorTask::add_ef_pose(const sva::PTransformd & dtr)
{
  auto new_rot = curTransform.rotation() * dtr.rotation();
  Eigen::Vector3d new_t = curTransform.translation() + dtr.translation();
  curTransform = sva::PTransformd(new_rot, new_t);
  positionTask->position(curTransform.translation());
  orientationTask->orientation(curTransform.rotation());
}

void EndEffectorTask::set_ef_pose(const sva::PTransformd & tf)
{
  curTransform = tf;
  positionTask->position(tf.translation());
  orientationTask->orientation(tf.rotation());
}

sva::PTransformd EndEffectorTask::get_ef_pose()
{
  return sva::PTransformd(orientationTask->orientation(), positionTask->position());
}

void EndEffectorTask::dimWeight(const Eigen::VectorXd & dimW)
{
  assert(dimW.size() == 6);
  orientationTask->dimWeight(dimW.head(3));
  positionTask->dimWeight(dimW.tail(3));
}

Eigen::VectorXd EndEffectorTask::dimWeight() const
{
  Eigen::VectorXd ret(6);
  ret << orientationTask->dimWeight(), positionTask->dimWeight();
  return ret;
}

void EndEffectorTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                         const std::vector<std::string> & activeJointsName,
                                         const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs)
{
  ensureHasJoints(robots.robot(robotIndex), activeJointsName, "[EndEffectorTask::selectActiveJoints]");
  positionTask->selectActiveJoints(solver, activeJointsName, activeDofs);
  orientationTask->selectActiveJoints(solver, activeJointsName, activeDofs);
}

void EndEffectorTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                           const std::vector<std::string> & unactiveJointsName,
                                           const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs)
{
  ensureHasJoints(robots.robot(robotIndex), unactiveJointsName, "[EndEffectorTask::selectUnactiveJoints]");
  positionTask->selectUnactiveJoints(solver, unactiveJointsName, unactiveDofs);
  orientationTask->selectUnactiveJoints(solver, unactiveJointsName, unactiveDofs);
}

void EndEffectorTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  positionTask->resetJointsSelector(solver);
  orientationTask->resetJointsSelector(solver);
}

Eigen::VectorXd EndEffectorTask::eval() const
{
  Eigen::Vector6d err;
  err << orientationTask->eval(), positionTask->eval();
  return err;
}

Eigen::VectorXd EndEffectorTask::speed() const
{
  Eigen::Vector6d spd;
  spd << orientationTask->speed(), positionTask->speed();
  return spd;
}

void EndEffectorTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  MetaTask::load(solver, config);
  if(config.has("stiffness"))
  {
    auto s = config("stiffness");
    if(s.size())
    {
      Eigen::VectorXd stiff = s;
      positionTask->stiffness(stiff);
      orientationTask->stiffness(stiff);
    }
    else
    {
      double stiff = s;
      positionTask->stiffness(stiff);
      orientationTask->stiffness(stiff);
    }
  }
  if(config.has("damping"))
  {
    auto d = config("damping");
    if(d.size())
    {
      positionTask->setGains(positionTask->dimStiffness(), d);
      orientationTask->setGains(orientationTask->dimStiffness(), d);
    }
    else
    {
      positionTask->setGains(positionTask->stiffness(), d);
      orientationTask->setGains(orientationTask->stiffness(), d);
    }
  }
  if(config.has("weight"))
  {
    double w = config("weight");
    positionTask->weight(w);
    orientationTask->weight(w);
  }
}

void EndEffectorTask::addToLogger(mc_rtc::Logger & logger)
{
  positionTask->addToLogger(logger);
  orientationTask->addToLogger(logger);
  logger.addLogEntry(name_ + "_target", [this]() -> const sva::PTransformd & { return curTransform; });
  logger.addLogEntry(
      name_, [this]() -> const sva::PTransformd & { return robots.robot(robotIndex).mbc().bodyPosW[bodyIndex]; });
}

void EndEffectorTask::removeFromLogger(mc_rtc::Logger & logger)
{
  positionTask->removeFromLogger(logger);
  orientationTask->removeFromLogger(logger);
  logger.removeLogEntry(name_ + "_target");
  logger.removeLogEntry(name_);
}

void EndEffectorTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement(
      {"Tasks", name_},
      mc_rtc::gui::Transform("pos_target", [this]() { return this->get_ef_pose(); },
                             [this](const sva::PTransformd & pos) { this->set_ef_pose(pos); }),
      mc_rtc::gui::Transform("pos", [this]() { return robots.robot(robotIndex).mbc().bodyPosW[bodyIndex]; }));
  gui.addElement({"Tasks", name_, "Gains", "Position"},
                 mc_rtc::gui::NumberInput(
                     "stiffness", [this]() { return this->positionTask->stiffness(); },
                     [this](const double & s) { this->positionTask->setGains(s, this->positionTask->damping()); }),
                 mc_rtc::gui::NumberInput(
                     "damping", [this]() { return this->positionTask->damping(); },
                     [this](const double & d) { this->positionTask->setGains(this->positionTask->stiffness(), d); }),
                 mc_rtc::gui::NumberInput("stiffness & damping", [this]() { return this->positionTask->stiffness(); },
                                          [this](const double & g) { this->positionTask->stiffness(g); }),
                 mc_rtc::gui::NumberInput("weight", [this]() { return this->positionTask->weight(); },
                                          [this](const double & w) { this->positionTask->weight(w); }));
  gui.addElement({"Tasks", name_, "Gains", "Orientation"},
                 mc_rtc::gui::NumberInput("stiffness", [this]() { return this->orientationTask->stiffness(); },
                                          [this](const double & s) {
                                            this->orientationTask->setGains(s, this->orientationTask->damping());
                                          }),
                 mc_rtc::gui::NumberInput("damping", [this]() { return this->orientationTask->damping(); },
                                          [this](const double & d) {
                                            this->orientationTask->setGains(this->orientationTask->stiffness(), d);
                                          }),
                 mc_rtc::gui::NumberInput("stiffness & damping",
                                          [this]() { return this->orientationTask->stiffness(); },
                                          [this](const double & g) { this->orientationTask->stiffness(g); }),
                 mc_rtc::gui::NumberInput("weight", [this]() { return this->orientationTask->weight(); },
                                          [this](const double & w) { this->orientationTask->weight(w); }));
}

} // namespace mc_tasks
