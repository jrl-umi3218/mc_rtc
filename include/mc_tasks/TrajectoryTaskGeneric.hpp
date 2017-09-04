#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <cmath>

namespace mc_tasks
{

template<typename T>
TrajectoryTaskGeneric<T>::TrajectoryTaskGeneric(const mc_rbdyn::Robots & robots,
                                             unsigned int robotIndex,
                                             double stifness,
                                             double w)
: robots(robots), rIndex(robotIndex),
  stiff(stifness), damp(2*std::sqrt(stiff)), wt(w)
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
void TrajectoryTaskGeneric<T>::update()
{
}

template<typename T>
void TrajectoryTaskGeneric<T>::refVel(const Eigen::VectorXd & vel)
{
  trajectoryT->refVel(vel);
}

template<typename T>
void TrajectoryTaskGeneric<T>::refAccel(const Eigen::VectorXd & accel)
{
  trajectoryT->refAccel(accel);
}

template<typename T>
void TrajectoryTaskGeneric<T>::stiffness(double s)
{
  setGains(s, 2*std::sqrt(s));
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
void TrajectoryTaskGeneric<T>::selectActiveJoints(mc_solver::QPSolver & solver,
                                                  const std::vector<std::string> & activeJointsName)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }
  selectorT = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::ActiveJoints(robots.mbs(), rIndex, errorT.get(), activeJointsName));
  trajectoryT = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, selectorT.get(), stiff, damp, wt);
  if(putBack)
  {
    addToSolver(solver);
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                                  const std::vector<std::string> & unactiveJointsName)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }
  selectorT = std::make_shared<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::UnactiveJoints(robots.mbs(), rIndex, errorT.get(), unactiveJointsName));
  trajectoryT = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, selectorT.get(), stiff, damp, wt);
  if(putBack)
  {
    addToSolver(solver);
  }
}

template<typename T>
void TrajectoryTaskGeneric<T>::resetJointsSelector(mc_solver::QPSolver & solver)
{
  bool putBack = inSolver;
  if(putBack)
  {
    removeFromSolver(solver);
  }
  selectorT = nullptr;
  trajectoryT = std::make_shared<tasks::qp::TrajectoryTask>(robots.mbs(), rIndex, errorT.get(), stiff, damp, wt);
  if(putBack)
  {
    addToSolver(solver);
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

}
