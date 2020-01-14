/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/NumberInput.h>

#include <cmath>

namespace mc_tasks
{

template<typename T>
TrajectoryTaskGeneric<T>::TrajectoryTaskGeneric(const mc_rbdyn::Robots & robots,
                                                unsigned int robotIndex,
                                                double stiffness,
                                                double w)
: robots(robots), rIndex(robotIndex), stiffness_(Eigen::VectorXd::Constant(1, stiffness)),
  damping_(Eigen::VectorXd::Constant(1, 2 * std::sqrt(stiffness))), weight_(w)
{
}

template<typename T>
TrajectoryTaskGeneric<T>::~TrajectoryTaskGeneric()
{
}

template<typename T>
template<typename... Args>
void TrajectoryTaskGeneric<T>::finalize(Args &&... args)
{
  errorT = std::make_shared<T>(args...);
  trajectoryT_ = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, errorT.get(), stiffness_(0),
                                                             damping_(0), weight_);
  stiffness_ = trajectoryT_->stiffness();
  damping_ = trajectoryT_->damping();
  if(refVel_.size() != trajectoryT_->refVel().size())
  {
    refVel_ = trajectoryT_->refVel();
  }
  if(refAccel_.size() != trajectoryT_->refAccel().size())
  {
    refAccel_ = trajectoryT_->refAccel();
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(inSolver_)
  {
    solver.removeTask(trajectoryT_.get());
    inSolver_ = false;
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::addToSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver_)
  {
    solver.addTask(trajectoryT_.get());
    inSolver_ = true;
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::reset()
{
  const Eigen::VectorXd & v = trajectoryT_->refVel();
  refVel(Eigen::VectorXd::Zero(v.size()));
  refAccel(Eigen::VectorXd::Zero(v.size()));
}

template<typename T>
void TrajectoryTaskGeneric<T>::update(mc_solver::QPSolver &)
{
}

template<typename T>
void TrajectoryTaskGeneric<T>::refVel(const Eigen::VectorXd & vel)
{
  trajectoryT_->refVel(vel);
  refVel_ = vel;
}

template<typename T>
const Eigen::VectorXd & TrajectoryTaskGeneric<T>::refVel() const
{
  return trajectoryT_->refVel();
}

template<typename T>
void TrajectoryTaskGeneric<T>::refAccel(const Eigen::VectorXd & accel)
{
  trajectoryT_->refAccel(accel);
  refAccel_ = accel;
}

template<typename T>
const Eigen::VectorXd & TrajectoryTaskGeneric<T>::refAccel() const
{
  return trajectoryT_->refAccel();
}

template<typename T>
void TrajectoryTaskGeneric<T>::stiffness(double s)
{
  setGains(s, 2 * std::sqrt(s));
}

template<typename T>
void TrajectoryTaskGeneric<T>::stiffness(const Eigen::VectorXd & stiffness)
{
  setGains(stiffness, 2 * stiffness.cwiseSqrt());
}

template<typename T>
void TrajectoryTaskGeneric<T>::damping(double d)
{
  damping_.setConstant(d);
  trajectoryT_->setGains(stiffness_, damping_);
}

template<typename T>
void TrajectoryTaskGeneric<T>::damping(const Eigen::VectorXd & damping)
{
  setGains(trajectoryT_->stiffness(), damping);
}

template<typename T>
void TrajectoryTaskGeneric<T>::setGains(double s, double d)
{
  stiffness_.setConstant(s);
  damping_.setConstant(d);
  trajectoryT_->setGains(stiffness_, damping_);
}

template<typename T>
void TrajectoryTaskGeneric<T>::setGains(const Eigen::VectorXd & stiffness, const Eigen::VectorXd & damping)
{
  stiffness_ = stiffness;
  damping_ = damping;
  trajectoryT_->setGains(stiffness, damping);
}

template<typename T>
double TrajectoryTaskGeneric<T>::stiffness() const
{
  return stiffness_(0);
}

template<typename T>
double TrajectoryTaskGeneric<T>::damping() const
{
  return damping_(0);
}

template<typename T>
const Eigen::VectorXd & TrajectoryTaskGeneric<T>::dimStiffness() const
{
  return stiffness_;
}

template<typename T>
const Eigen::VectorXd & TrajectoryTaskGeneric<T>::dimDamping() const
{
  return damping_;
}

template<typename T>
void TrajectoryTaskGeneric<T>::weight(double w)
{
  weight_ = w;
  trajectoryT_->weight(w);
}

template<typename T>
double TrajectoryTaskGeneric<T>::weight() const
{
  return weight_;
}

template<typename T>
void TrajectoryTaskGeneric<T>::dimWeight(const Eigen::VectorXd & w)
{
  trajectoryT_->dimWeight(w);
}

template<typename T>
Eigen::VectorXd TrajectoryTaskGeneric<T>::dimWeight() const
{
  return trajectoryT_->dimWeight();
}

template<typename T>
void TrajectoryTaskGeneric<T>::selectActiveJoints(
    const std::vector<std::string> & activeJointsName,
    const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs)
{
  if(inSolver_)
  {
    LOG_WARNING("selectActiveJoints(names) ignored: use selectActiveJoints(solver, names) for a task already added to "
                "the solver");
  }
  selectorT_ = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::ActiveJoints(
      robots.mbs(), static_cast<int>(rIndex), errorT.get(), activeJointsName, activeDofs));
  trajectoryT_ = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, selectorT_.get(), 1, 2, weight_);
  trajectoryT_->setGains(stiffness_, damping_);
}

template<typename T>
void TrajectoryTaskGeneric<T>::selectActiveJoints(
    mc_solver::QPSolver & solver,
    const std::vector<std::string> & activeJointsName,
    const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs)
{
  if(inSolver_)
  {
    removeFromSolver(solver);
    selectActiveJoints(activeJointsName, activeDofs);
    addToSolver(solver);
  }
  else
  {
    selectActiveJoints(activeJointsName, activeDofs);
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::selectUnactiveJoints(
    const std::vector<std::string> & unactiveJointsName,
    const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs)
{
  if(inSolver_)
  {
    LOG_WARNING("selectUnactiveJoints(names) ignored: use selectUnactiveJoints(solver, names) for a task already added "
                "to the solver");
  }
  selectorT_ = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::UnactiveJoints(
      robots.mbs(), static_cast<int>(rIndex), errorT.get(), unactiveJointsName, unactiveDofs));
  trajectoryT_ = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, selectorT_.get(), 1, 2, weight_);
  trajectoryT_->setGains(stiffness_, damping_);
}

template<typename T>
void TrajectoryTaskGeneric<T>::selectUnactiveJoints(
    mc_solver::QPSolver & solver,
    const std::vector<std::string> & unactiveJointsName,
    const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs)
{
  if(inSolver_)
  {
    removeFromSolver(solver);
    selectUnactiveJoints(unactiveJointsName, unactiveDofs);
    addToSolver(solver);
  }
  else
  {
    selectUnactiveJoints(unactiveJointsName, unactiveDofs);
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::resetJointsSelector()
{
  if(inSolver_)
  {
    LOG_WARNING(
        "resetJointsSelector() ignored: use resetJointsSelector(solver) for a task already added to the solver");
  }
  selectorT_ = nullptr;
  trajectoryT_ = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, errorT.get(), 1, 2, weight_);
  trajectoryT_->setGains(stiffness_, damping_);
}

template<typename T>
void TrajectoryTaskGeneric<T>::resetJointsSelector(mc_solver::QPSolver & solver)
{
  if(inSolver_)
  {
    removeFromSolver(solver);
    resetJointsSelector();
    addToSolver(solver);
  }
  else
  {
    resetJointsSelector();
  }
}

template<typename T>
Eigen::VectorXd TrajectoryTaskGeneric<T>::eval() const
{
  if(selectorT_)
  {
    return selectorT_->eval().cwiseProduct(trajectoryT_->dimWeight());
  }
  return errorT->eval().cwiseProduct(trajectoryT_->dimWeight());
}

template<typename T>
Eigen::VectorXd TrajectoryTaskGeneric<T>::speed() const
{
  if(selectorT_)
  {
    return selectorT_->speed().cwiseProduct(trajectoryT_->dimWeight());
  }
  return errorT->speed().cwiseProduct(trajectoryT_->dimWeight());
}

template<typename T>
const Eigen::VectorXd & TrajectoryTaskGeneric<T>::normalAcc() const
{
  if(selectorT_)
  {
    return selectorT_->normalAcc();
  }
  return errorT->normalAcc();
}

template<typename T>
const Eigen::MatrixXd & TrajectoryTaskGeneric<T>::jac() const
{
  if(selectorT_)
  {
    return selectorT_->jac();
  }
  return errorT->jac();
}

template<typename T>
void TrajectoryTaskGeneric<T>::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  MetaTask::load(solver, config);
  if(config.has("stiffness"))
  {
    auto s = config("stiffness");
    if(s.size())
    {
      Eigen::VectorXd stiff = s;
      stiffness(stiff);
    }
    else
    {
      stiffness(static_cast<double>(s));
    }
  }
  if(config.has("damping"))
  {
    auto d = config("damping");
    if(d.size())
    {
      setGains(dimStiffness(), d);
    }
    else
    {
      setGains(stiffness(), d);
    }
  }
  if(config.has("weight"))
  {
    weight(config("weight"));
  }
  if(config.has("refVel"))
  {
    refVel(config("refVel"));
  }
  if(config.has("refAccel"))
  {
    refAccel(config("refAccel"));
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::ArrayInput("refVel", [this]() { return this->refVel(); },
                                         [this](const Eigen::VectorXd & v) { this->refVel(v); }),
                 mc_rtc::gui::ArrayInput("refAccel", [this]() { return this->refAccel(); },
                                         [this](const Eigen::VectorXd & v) { this->refAccel(v); }));
  gui.addElement({"Tasks", name_, "Gains"},
                 mc_rtc::gui::NumberInput("stiffness", [this]() { return this->stiffness(); },
                                          [this](const double & s) { this->setGains(s, this->damping()); }),
                 mc_rtc::gui::NumberInput("damping", [this]() { return this->damping(); },
                                          [this](const double & d) { this->setGains(this->stiffness(), d); }),
                 mc_rtc::gui::NumberInput("stiffness & damping", [this]() { return this->stiffness(); },
                                          [this](const double & g) { this->stiffness(g); }),
                 mc_rtc::gui::NumberInput("weight", [this]() { return this->weight(); },
                                          [this](const double & w) { this->weight(w); }));
  gui.addElement(
      {"Tasks", name_, "Gains", "Dimensional"},
      mc_rtc::gui::ArrayInput("stiffness", [this]() { return this->dimStiffness(); },
                              [this](const Eigen::VectorXd & v) { this->setGains(v, this->dimDamping()); }),
      mc_rtc::gui::ArrayInput("damping", [this]() { return this->dimDamping(); },
                              [this](const Eigen::VectorXd & v) { this->setGains(this->dimStiffness(), v); }),
      mc_rtc::gui::ArrayInput("stiffness & damping", [this]() { return this->dimStiffness(); },
                              [this](const Eigen::VectorXd & v) { this->stiffness(v); }));
}

template<typename T>
void TrajectoryTaskGeneric<T>::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_damping", [this]() { return damping_(0); });
  logger.addLogEntry(name_ + "_stiffness", [this]() { return stiffness_(0); });
  logger.addLogEntry(name_ + "_dimDamping", [this]() -> const Eigen::VectorXd & { return damping_; });
  logger.addLogEntry(name_ + "_dimStiffness", [this]() -> const Eigen::VectorXd & { return stiffness_; });
  logger.addLogEntry(name_ + "_refVel", [this]() { return refVel_; });
  logger.addLogEntry(name_ + "_refAccel", [this]() { return refAccel_; });
}

template<typename T>
void TrajectoryTaskGeneric<T>::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_damping");
  logger.removeLogEntry(name_ + "_stiffness");
  logger.removeLogEntry(name_ + "_dimDamping");
  logger.removeLogEntry(name_ + "_dimStiffness");
  logger.removeLogEntry(name_ + "_refVel");
  logger.removeLogEntry(name_ + "_refAccel");
}

template<typename T>
std::function<bool(const mc_tasks::MetaTask & task, std::string &)> TrajectoryTaskGeneric<T>::buildCompletionCriteria(
    double dt,
    const mc_rtc::Configuration & config) const
{
  return MetaTask::buildCompletionCriteria(dt, config);
}

} // namespace mc_tasks
