/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/QPSolver.h>

#include <mc_solver/DynamicsConstraint.h>

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Surface.h>
#include <mc_tasks/MetaTask.h>

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Force.h>
#include <mc_rtc/gui/Form.h>

#include <mc_rtc/logging.h>

namespace mc_solver
{

static thread_local QPSolver::Backend CONTEXT_BACKEND = QPSolver::Backend::Unset;

QPSolver::Backend QPSolver::context_backend()
{
  return CONTEXT_BACKEND;
}

void QPSolver::context_backend(Backend backend)
{
  CONTEXT_BACKEND = backend;
}

QPSolver::QPSolver(mc_rbdyn::RobotsPtr robots, double timeStep, Backend backend)
: backend_(backend), robots_p(robots), timeStep(timeStep)
{
  context_backend(backend_);
  if(timeStep <= 0)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("timeStep has to be > 0! timeStep = {}", timeStep);
  }
  realRobots_p = mc_rbdyn::Robots::make();
  for(const auto & robot : *robots)
  {
    realRobots_p->robotCopy(robot, robot.name());
  }
}

QPSolver::QPSolver(double timeStep, Backend backend) : QPSolver{mc_rbdyn::Robots::make(), timeStep, backend} {}

void QPSolver::addConstraintSet(ConstraintSet & cs)
{
  if(cs.backend() != backend_)
  {
    mc_rtc::log::error_and_throw(
        "[QPSolver::addConstraintSet] Constraint backend ({}) is different from this solver backend ({})", cs.backend(),
        backend_);
  }
  cs.addToSolver(*this);
  if(dynamic_cast<DynamicsConstraint *>(&cs) != nullptr)
  {
    addDynamicsConstraint(static_cast<DynamicsConstraint *>(&cs));
  }
}

void QPSolver::removeConstraintSet(ConstraintSet & cs)
{
  if(cs.backend() != backend_)
  {
    mc_rtc::log::error_and_throw(
        "[QPSolver::removeConstraintSet] Constraint backend ({}) is different from this solver backend ({})",
        cs.backend(), backend_);
  }
  cs.removeFromSolver(*this);
  removeDynamicsConstraint(&cs);
}

void QPSolver::addTask(mc_tasks::MetaTask * task)
{
  if(std::find(metaTasks_.begin(), metaTasks_.end(), task) == metaTasks_.end())
  {
    if(task->backend() != backend_)
    {
      mc_rtc::log::error_and_throw("[QPSolver::addTask] Task backend ({}) is different from this solver backend ({})",
                                   task->backend(), backend_);
    }
    metaTasks_.push_back(task);
    task->addToSolver(*this);
    task->resetIterInSolver();
    if(logger_)
    {
      task->addToLogger(*logger_);
    }
    if(gui_)
    {
      addTaskToGUI(task);
    }
    mc_rtc::log::info("Added task {}", task->name());
  }
}

void QPSolver::removeTask(mc_tasks::MetaTask * task)
{
  auto it = std::find(metaTasks_.begin(), metaTasks_.end(), task);
  if(it != metaTasks_.end())
  {
    if(task->backend() != backend_)
    {
      mc_rtc::log::error_and_throw(
          "[QPSolver::removeTask] Task backend ({}) is different from this solver backend ({})", task->backend(),
          backend_);
    }
    task->removeFromSolver(*this);
    task->resetIterInSolver();
    if(logger_)
    {
      task->removeFromLogger(*logger_);
    }
    if(gui_)
    {
      task->removeFromGUI(*gui_);
    }
    mc_rtc::log::info("Removed task {}", task->name());
    metaTasks_.erase(it);
    shPtrTasksStorage.erase(std::remove_if(shPtrTasksStorage.begin(), shPtrTasksStorage.end(),
                                           [task](const std::shared_ptr<void> & p) { return task == p.get(); }),
                            shPtrTasksStorage.end());
  }
}

const std::vector<mc_rbdyn::Contact> & QPSolver::contacts() const
{
  return contacts_;
}

const std::vector<mc_tasks::MetaTask *> & QPSolver::tasks() const
{
  return metaTasks_;
}

bool QPSolver::run(FeedbackType fType)
{
  return run_impl(fType);
}

const mc_rbdyn::Robot & QPSolver::robot() const
{
  return robots_p->robot();
}
mc_rbdyn::Robot & QPSolver::robot()
{
  return robots_p->robot();
}

mc_rbdyn::Robot & QPSolver::robot(unsigned int idx)
{
  return robots_p->robot(idx);
}
const mc_rbdyn::Robot & QPSolver::robot(unsigned int idx) const
{
  return robots_p->robot(idx);
}

const mc_rbdyn::Robot & QPSolver::env() const
{
  return robots_p->env();
}
mc_rbdyn::Robot & QPSolver::env()
{
  return robots_p->env();
}

const mc_rbdyn::Robots & QPSolver::robots() const
{
  assert(robots_p);
  return *robots_p;
}
mc_rbdyn::Robots & QPSolver::robots()
{
  assert(robots_p);
  return *robots_p;
}

const mc_rbdyn::Robots & QPSolver::realRobots() const
{
  assert(realRobots_p);
  return *realRobots_p;
}
mc_rbdyn::Robots & QPSolver::realRobots()
{
  assert(realRobots_p);
  return *realRobots_p;
}

double QPSolver::dt() const
{
  return timeStep;
}

void QPSolver::logger(std::shared_ptr<mc_rtc::Logger> logger)
{
  if(logger_)
  {
    for(auto t : metaTasks_)
    {
      t->removeFromLogger(*logger_);
    }
  }
  logger_ = logger;
  if(logger_)
  {
    for(auto t : metaTasks_)
    {
      t->addToLogger(*logger_);
    }
  }
}

std::shared_ptr<mc_rtc::Logger> QPSolver::logger() const
{
  return logger_;
}

void QPSolver::gui(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
{
  if(gui_)
  {
    for(auto t : metaTasks_)
    {
      t->removeFromGUI(*gui_);
    }
  }
  gui_ = gui;
  if(gui_)
  {
    for(auto t : metaTasks_)
    {
      addTaskToGUI(t);
    }
  }
}

/** Access to the gui instance */
std::shared_ptr<mc_rtc::gui::StateBuilder> QPSolver::gui() const
{
  return gui_;
}

void QPSolver::addTaskToGUI(mc_tasks::MetaTask * t)
{
  assert(gui_);
  t->addToGUI(*gui_);
  gui_->addElement({"Tasks", t->name()},
                   mc_rtc::gui::Button("Remove from solver", [this, t]() { this->removeTask(t); }));
}

} // namespace mc_solver
