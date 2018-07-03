#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <cmath>

namespace mc_tasks
{

template<typename T>
TrajectoryTaskGeneric<T>::TrajectoryTaskGeneric(const mc_rbdyn::Robots & robots,
                                             unsigned int robotIndex,
                                             double stiffness,
                                             double w)
: robots(robots), rIndex(robotIndex),
  stiff(stiffness), damp(2*std::sqrt(stiff)), wt(w)
{
}

template<typename T>
TrajectoryTaskGeneric<T>::~TrajectoryTaskGeneric()
{
}

template<typename T>
template<typename ... Args>
void TrajectoryTaskGeneric<T>::finalize(Args && ... args)
{
  errorT = std::make_shared<T>(args...);
  trajectoryT = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, errorT.get(), stiff, damp, wt);
}

template<typename T>
void TrajectoryTaskGeneric<T>::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(inSolver)
  {
    solver.removeTask(trajectoryT.get());
    inSolver = false;
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::addToSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver)
  {
    solver.addTask(trajectoryT.get());
    inSolver = true;
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::reset()
{
  const Eigen::VectorXd & v = trajectoryT->refVel();
  refVel(Eigen::VectorXd::Zero(v.size()));
  refAccel(Eigen::VectorXd::Zero(v.size()));
}

template<typename T>
void TrajectoryTaskGeneric<T>::update()
{
}

template<typename T>
void TrajectoryTaskGeneric<T>::refVel(const Eigen::VectorXd & vel)
{
  trajectoryT->refVel(vel);
  refVel_ = vel;
}

template<typename T>
void TrajectoryTaskGeneric<T>::refAccel(const Eigen::VectorXd & accel)
{
  trajectoryT->refAccel(accel);
  refAccel_ = accel;
}

template<typename T>
void TrajectoryTaskGeneric<T>::stiffness(double s)
{
  setGains(s, 2*std::sqrt(s));
}

template<typename T>
void TrajectoryTaskGeneric<T>::damping(double d)
{
  setGains(stiff, d);
}

template<typename T>
void TrajectoryTaskGeneric<T>::setGains(double s, double d)
{
  stiff = s;
  damp = d;
  trajectoryT->setGains(s, d);
}

template<typename T>
double TrajectoryTaskGeneric<T>::stiffness() const
{
  return stiff;
}

template<typename T>
double TrajectoryTaskGeneric<T>::damping() const
{
  return damp;
}

template<typename T>
void TrajectoryTaskGeneric<T>::weight(double w)
{
  wt = w;
  trajectoryT->weight(w);
}

template<typename T>
double TrajectoryTaskGeneric<T>::weight() const
{
  return wt;
}

template<typename T>
void TrajectoryTaskGeneric<T>::dimWeight(const Eigen::VectorXd & w)
{
  trajectoryT->dimWeight(w);
}

template<typename T>
Eigen::VectorXd TrajectoryTaskGeneric<T>::dimWeight() const
{
  return trajectoryT->dimWeight();
}

template<typename T>
void TrajectoryTaskGeneric<T>::selectActiveJoints(const std::vector<std::string> & activeJointsName)
{
  if(inSolver)
  {
    LOG_WARNING("selectActiveJoints(names) ignored: use selectActiveJoints(solver, names) for a task already added to the solver");
  }
  selectorT = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::ActiveJoints(robots.mbs(), rIndex, errorT.get(), activeJointsName));
  trajectoryT = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, selectorT.get(), stiff, damp, wt);
}

template<typename T>
void TrajectoryTaskGeneric<T>::selectActiveJoints(mc_solver::QPSolver & solver,
                                                  const std::vector<std::string> & activeJointsName)
{
  if(inSolver)
  {
    removeFromSolver(solver);
    selectActiveJoints(activeJointsName);
    addToSolver(solver);
  }
  else
  {
    selectActiveJoints(activeJointsName);
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::selectUnactiveJoints(const std::vector<std::string> & unactiveJointsName)
{
  if(inSolver)
  {
    LOG_WARNING("selectUnactiveJoints(names) ignored: use selectUnactiveJoints(solver, names) for a task already added to the solver");
  }
  selectorT = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::UnactiveJoints(robots.mbs(), rIndex, errorT.get(), unactiveJointsName));
  trajectoryT = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, selectorT.get(), stiff, damp, wt);
}

template<typename T>
void TrajectoryTaskGeneric<T>::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                                  const std::vector<std::string> & unactiveJointsName)
{
  if(inSolver)
  {
    removeFromSolver(solver);
    selectUnactiveJoints(unactiveJointsName);
    addToSolver(solver);
  }
  else
  {
    selectUnactiveJoints(unactiveJointsName);
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::resetJointsSelector()
{
  if(inSolver)
  {
    LOG_WARNING("resetJointsSelector() ignored: use resetJointsSelector(solver) for a task already added to the solver");
  }
  selectorT = nullptr;
  trajectoryT = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, errorT.get(), stiff, damp, wt);
}

template<typename T>
void TrajectoryTaskGeneric<T>::resetJointsSelector(mc_solver::QPSolver & solver)
{
  if(inSolver)
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
  if(selectorT)
  {
    return selectorT->eval();
  }
  return errorT->eval();
}

template<typename T>
Eigen::VectorXd TrajectoryTaskGeneric<T>::speed() const
{
  if(selectorT)
  {
    return selectorT->speed();
  }
  return errorT->speed();
}

template<typename T>
void TrajectoryTaskGeneric<T>::load(mc_solver::QPSolver & solver,
                                    const mc_rtc::Configuration & config)
{
  MetaTask::load(solver, config);
  if(config.has("stiffness"))
  {
    stiffness(config("stiffness"));
  }
  if(config.has("damping"))
  {
    setGains(stiffness(), config("damping"));
  }
  if(config.has("weight"))
  {
    weight(config("weight"));
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement(
    {"Tasks", name_, "Gains"},
    mc_rtc::gui::NumberInput("stiffness",
                             [this]() { return this->stiffness(); },
                             [this](const double & s) { this->setGains(s, this->damping()); }),
    mc_rtc::gui::NumberInput("damping",
                             [this]() { return this->damping(); },
                             [this](const double & d) { this->setGains(this->stiffness(), d); }),
    mc_rtc::gui::NumberInput("stiffness & damping",
                             [this]() { return this->stiffness(); },
                             [this](const double & g) { this->stiffness(g); }),
    mc_rtc::gui::NumberInput("weight",
                             [this]() { return this->weight(); },
                             [this](const double & w) { this->weight(w); })
  );
}

}
