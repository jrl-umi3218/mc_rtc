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
: EndEffectorTask(robots.robot(robotIndex).frame(bodyName), stiffness, weight)
{
}

EndEffectorTask::EndEffectorTask(const mc_rbdyn::RobotFrame & frame, double stiffness, double weight)
{
  curTransform = frame.position();

  positionTask = std::make_shared<mc_tasks::PositionTask>(frame, stiffness, weight);
  orientationTask = std::make_shared<mc_tasks::OrientationTask>(frame, stiffness, weight);

  type_ = "body6d";
  name_ = "body6d_" + frame.robot().name() + "_" + frame.name();
  name(name_);
}

void EndEffectorTask::reset()
{
  curTransform = frame().position();
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
  ensureHasJoints(frame().robot(), activeJointsName, "[" + name() + "::selectActiveJoints]");
  positionTask->selectActiveJoints(solver, activeJointsName, activeDofs);
  orientationTask->selectActiveJoints(solver, activeJointsName, activeDofs);
}

void EndEffectorTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                           const std::vector<std::string> & unactiveJointsName,
                                           const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs)
{
  ensureHasJoints(frame().robot(), unactiveJointsName, "[" + name() + "::selectUnActiveJoints]");
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
  MC_RTC_LOG_HELPER(name_ + "_target", curTransform);
  logger.addLogEntry(name_, this, [this]() { return frame().position(); });
}

void EndEffectorTask::removeFromLogger(mc_rtc::Logger & logger)
{
  MetaTask::removeFromLogger(logger);
  positionTask->removeFromLogger(logger);
  orientationTask->removeFromLogger(logger);
}

void EndEffectorTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Transform(
                     "pos_target", [this]() { return this->get_ef_pose(); },
                     [this](const sva::PTransformd & pos) { this->set_ef_pose(pos); }),
                 mc_rtc::gui::Transform("pos", [this]() { return frame().position(); }));
  gui.addElement({"Tasks", name_, "Gains", "Position"},
                 mc_rtc::gui::NumberInput(
                     "stiffness", [this]() { return this->positionTask->stiffness(); },
                     [this](const double & s) { this->positionTask->setGains(s, this->positionTask->damping()); }),
                 mc_rtc::gui::NumberInput(
                     "damping", [this]() { return this->positionTask->damping(); },
                     [this](const double & d) { this->positionTask->setGains(this->positionTask->stiffness(), d); }),
                 mc_rtc::gui::NumberInput(
                     "stiffness & damping", [this]() { return this->positionTask->stiffness(); },
                     [this](const double & g) { this->positionTask->stiffness(g); }),
                 mc_rtc::gui::NumberInput(
                     "weight", [this]() { return this->positionTask->weight(); },
                     [this](const double & w) { this->positionTask->weight(w); }));
  gui.addElement(
      {"Tasks", name_, "Gains", "Orientation"},
      mc_rtc::gui::NumberInput(
          "stiffness", [this]() { return this->orientationTask->stiffness(); },
          [this](const double & s) { this->orientationTask->setGains(s, this->orientationTask->damping()); }),
      mc_rtc::gui::NumberInput(
          "damping", [this]() { return this->orientationTask->damping(); },
          [this](const double & d) { this->orientationTask->setGains(this->orientationTask->stiffness(), d); }),
      mc_rtc::gui::NumberInput(
          "stiffness & damping", [this]() { return this->orientationTask->stiffness(); },
          [this](const double & g) { this->orientationTask->stiffness(g); }),
      mc_rtc::gui::NumberInput(
          "weight", [this]() { return this->orientationTask->weight(); },
          [this](const double & w) { this->orientationTask->weight(w); }));
}

void EndEffectorTask::name(const std::string & name)
{
  MetaTask::name(name);
  positionTask->name(name + "_position");
  orientationTask->name(name + "_orientation");
}

} // namespace mc_tasks
