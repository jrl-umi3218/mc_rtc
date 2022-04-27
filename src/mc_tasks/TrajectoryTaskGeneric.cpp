/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_solver/TasksQPSolver.h>

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/NumberInput.h>

#include <cmath>

namespace mc_tasks
{

static inline tasks::qp::TrajectoryTask * tasks_trajectory(mc_rtc::void_ptr & ptr)
{
  return static_cast<tasks::qp::TrajectoryTask *>(ptr.get());
}

static inline const tasks::qp::TrajectoryTask * tasks_trajectory(const mc_rtc::void_ptr & ptr)
{
  return static_cast<const tasks::qp::TrajectoryTask *>(ptr.get());
}

static inline tasks::qp::HighLevelTask * tasks_error(mc_rtc::void_ptr & ptr)
{
  return static_cast<tasks::qp::HighLevelTask *>(ptr.get());
}

/* Note: this break const-correrctness because eval/speed are not const */
static inline const tasks::qp::HighLevelTask * tasks_error(const mc_rtc::void_ptr & ptr)
{
  return static_cast<tasks::qp::HighLevelTask *>(ptr.get());
}

static inline tasks::qp::JointsSelector * tasks_selector(mc_rtc::void_ptr & ptr)
{
  return static_cast<tasks::qp::JointsSelector *>(ptr.get());
}

/* Note: this break const-correrctness because eval/speed are not const */
static inline tasks::qp::JointsSelector * tasks_selector(const mc_rtc::void_ptr & ptr)
{
  return static_cast<tasks::qp::JointsSelector *>(ptr.get());
}

static inline void set_gains(MetaTask::Backend backend,
                             mc_rtc::void_ptr & traj,
                             const Eigen::VectorXd & s,
                             const Eigen::VectorXd & d)
{
  switch(backend)
  {
    case MetaTask::Backend::Tasks:
      tasks_trajectory(traj)->setGains(s, d);
      break;
    default:
      break;
  }
}

TrajectoryTaskGeneric::TrajectoryTaskGeneric(const mc_rbdyn::Robots & robots,
                                             unsigned int robotIndex,
                                             double stiffness,
                                             double w)
: robots(robots), rIndex(robotIndex), stiffness_(Eigen::VectorXd::Constant(1, stiffness)),
  damping_(Eigen::VectorXd::Constant(1, 2 * std::sqrt(stiffness))), weight_(w)
{
}

TrajectoryTaskGeneric::TrajectoryTaskGeneric(const mc_rbdyn::RobotFrame & frame, double stiffness, double weight)
: TrajectoryTaskGeneric(frame.robot().robots(), frame.robot().robotIndex(), stiffness, weight)
{
}

void TrajectoryTaskGeneric::removeFromSolver(mc_solver::QPSolver & solver)
{
  if(inSolver_)
  {
    inSolver_ = false;
    switch(backend_)
    {
      case Backend::Tasks:
        tasks_solver(solver).removeTask(tasks_trajectory(trajectoryT_));
        break;
      default:
        break;
    }
  }
}

void TrajectoryTaskGeneric::addToSolver(mc_solver::QPSolver & solver)
{
  if(!inSolver_)
  {
    inSolver_ = true;
    switch(backend_)
    {
      case Backend::Tasks:
        tasks_solver(solver).addTask(tasks_trajectory(trajectoryT_));
        break;
      default:
        break;
    }
  }
}

void TrajectoryTaskGeneric::reset()
{
  int dim = [this]() {
    switch(backend_)
    {
      case Backend::Tasks:
        return tasks_error(errorT)->dim();
      default:
        return 0;
    }
  }();
  refVel(Eigen::VectorXd::Zero(dim));
  refAccel(Eigen::VectorXd::Zero(dim));
}

void TrajectoryTaskGeneric::update(mc_solver::QPSolver &) {}

void TrajectoryTaskGeneric::refVel(const Eigen::VectorXd & vel)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_trajectory(trajectoryT_)->refVel(vel);
      break;
    default:
      break;
  }
  refVel_ = vel;
}

const Eigen::VectorXd & TrajectoryTaskGeneric::refVel() const
{
  return refVel_;
}

void TrajectoryTaskGeneric::refAccel(const Eigen::VectorXd & accel)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_trajectory(trajectoryT_)->refAccel(accel);
      break;
    default:
      break;
  }
  refAccel_ = accel;
}

const Eigen::VectorXd & TrajectoryTaskGeneric::refAccel() const
{
  return refAccel_;
}

void TrajectoryTaskGeneric::stiffness(double s)
{
  setGains(s, 2 * std::sqrt(s));
}

void TrajectoryTaskGeneric::stiffness(const Eigen::VectorXd & stiffness)
{
  setGains(stiffness, 2 * stiffness.cwiseSqrt());
}

void TrajectoryTaskGeneric::damping(double d)
{
  damping_.setConstant(d);
  set_gains(backend_, trajectoryT_, stiffness_, damping_);
}

void TrajectoryTaskGeneric::damping(const Eigen::VectorXd & damping)
{
  damping_ = damping;
  set_gains(backend_, trajectoryT_, stiffness_, damping_);
}

void TrajectoryTaskGeneric::setGains(double s, double d)
{
  stiffness_.setConstant(s);
  damping_.setConstant(d);
  set_gains(backend_, trajectoryT_, stiffness_, damping_);
}

void TrajectoryTaskGeneric::setGains(const Eigen::VectorXd & stiffness, const Eigen::VectorXd & damping)
{
  stiffness_ = stiffness;
  damping_ = damping;
  set_gains(backend_, trajectoryT_, stiffness_, damping_);
}

double TrajectoryTaskGeneric::stiffness() const
{
  return stiffness_(0);
}

double TrajectoryTaskGeneric::damping() const
{
  return damping_(0);
}

const Eigen::VectorXd & TrajectoryTaskGeneric::dimStiffness() const
{
  return stiffness_;
}

const Eigen::VectorXd & TrajectoryTaskGeneric::dimDamping() const
{
  return damping_;
}

void TrajectoryTaskGeneric::weight(double w)
{
  weight_ = w;
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_trajectory(trajectoryT_)->weight(w);
      break;
    default:
      break;
  }
}

double TrajectoryTaskGeneric::weight() const
{
  return weight_;
}

void TrajectoryTaskGeneric::dimWeight(const Eigen::VectorXd & w)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_trajectory(trajectoryT_)->dimWeight(w);
      break;
    default:
      break;
  }
}

Eigen::VectorXd TrajectoryTaskGeneric::dimWeight() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_trajectory(trajectoryT_)->dimWeight();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void TrajectoryTaskGeneric::selectActiveJoints(const std::vector<std::string> & activeJointsName,
                                               const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs,
                                               bool checkJoints)
{
  if(inSolver_)
  {
    mc_rtc::log::warning("selectActiveJoints(names) ignored: use selectActiveJoints(solver, names) for a task already "
                         "added to the solver");
    return;
  }
  if(checkJoints)
  {
    ensureHasJoints(robots.robot(rIndex), activeJointsName, "[" + name() + "::selectActiveJoints]");
  }
  switch(backend_)
  {
    case Backend::Tasks:
    {
      selectorT_ = mc_rtc::make_void_ptr<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::ActiveJoints(
          robots.mbs(), static_cast<int>(rIndex), tasks_error(errorT), activeJointsName, activeDofs));
      trajectoryT_ = mc_rtc::make_void_ptr<tasks::qp::TrajectoryTask>(
          robots.mbs(), static_cast<int>(rIndex), tasks_selector(selectorT_), 1, 2, dimWeight(), weight_);
      set_gains(backend_, trajectoryT_, stiffness_, damping_);
    }
    break;
    default:
      break;
  }
}

void TrajectoryTaskGeneric::selectActiveJoints(mc_solver::QPSolver & solver,
                                               const std::vector<std::string> & activeJointsName,
                                               const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs)
{
  ensureHasJoints(robots.robot(rIndex), activeJointsName, "[" + name() + "::selectActiveJoints]");
  if(inSolver_)
  {
    removeFromSolver(solver);
    selectActiveJoints(activeJointsName, activeDofs, false);
    addToSolver(solver);
  }
  else
  {
    selectActiveJoints(activeJointsName, activeDofs, false);
  }
}

void TrajectoryTaskGeneric::selectUnactiveJoints(
    const std::vector<std::string> & unactiveJointsName,
    const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs,
    bool checkJoints)
{
  if(inSolver_)
  {
    mc_rtc::log::warning(
        "{}::selectUnactiveJoints(names) ignored: use selectUnactiveJoints(solver, names) for a task already added "
        "to the solver",
        name());
    return;
  }
  if(checkJoints)
  {
    ensureHasJoints(robots.robot(rIndex), unactiveJointsName, "[" + name() + "::selectUnActiveJoints]");
  }
  switch(backend_)
  {
    case Backend::Tasks:
    {
      selectorT_ = mc_rtc::make_void_ptr<tasks::qp::JointsSelector>(tasks::qp::JointsSelector::UnactiveJoints(
          robots.mbs(), static_cast<int>(rIndex), tasks_error(errorT), unactiveJointsName, unactiveDofs));
      trajectoryT_ = mc_rtc::make_void_ptr<tasks::qp::TrajectoryTask>(
          robots.mbs(), static_cast<int>(rIndex), tasks_selector(selectorT_), 1, 2, dimWeight(), weight_);
      set_gains(backend_, trajectoryT_, stiffness_, damping_);
    }
    break;
    default:
      break;
  }
}

void TrajectoryTaskGeneric::selectUnactiveJoints(
    mc_solver::QPSolver & solver,
    const std::vector<std::string> & unactiveJointsName,
    const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs)
{
  ensureHasJoints(robots.robot(rIndex), unactiveJointsName, "[" + name() + "::selectUnActiveJoints]");
  if(inSolver_)
  {
    removeFromSolver(solver);
    selectUnactiveJoints(unactiveJointsName, unactiveDofs, false);
    addToSolver(solver);
  }
  else
  {
    selectUnactiveJoints(unactiveJointsName, unactiveDofs, false);
  }
}

void TrajectoryTaskGeneric::resetJointsSelector()
{
  if(inSolver_)
  {
    mc_rtc::log::warning(
        "{}::resetJointsSelector() ignored: use resetJointsSelector(solver) for a task already added to the solver",
        name());
    return;
  }
  selectorT_ = {nullptr, nullptr};
  switch(backend_)
  {
    case Backend::Tasks:
    {
      trajectoryT_ = mc_rtc::make_void_ptr<tasks::qp::TrajectoryTask>(robots.mbs(), static_cast<int>(rIndex),
                                                                      tasks_error(errorT), 1, 2, dimWeight(), weight_);
      set_gains(backend_, trajectoryT_, stiffness_, damping_);
    }
    break;
    default:
      break;
  }
}

void TrajectoryTaskGeneric::resetJointsSelector(mc_solver::QPSolver & solver)
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

Eigen::VectorXd TrajectoryTaskGeneric::eval() const
{
  switch(backend_)
  {
    case Backend::Tasks:
    {
      const auto & dimWeight = tasks_trajectory(trajectoryT_)->dimWeight();
      if(selectorT_)
      {
        return tasks_selector(selectorT_)->eval().cwiseProduct(dimWeight);
      }
      return tasks_error(errorT)->eval().cwiseProduct(dimWeight);
    }
    break;
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

Eigen::VectorXd TrajectoryTaskGeneric::speed() const
{
  switch(backend_)
  {
    case Backend::Tasks:
    {
      const auto & dimWeight = tasks_trajectory(trajectoryT_)->dimWeight();
      if(selectorT_)
      {
        return tasks_selector(selectorT_)->speed().cwiseProduct(dimWeight);
      }
      return tasks_error(errorT)->speed().cwiseProduct(dimWeight);
    }
    break;
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

const Eigen::VectorXd & TrajectoryTaskGeneric::normalAcc() const
{
  switch(backend_)
  {
    case Backend::Tasks:
    {
      if(selectorT_)
      {
        return tasks_selector(selectorT_)->normalAcc();
      }
      return tasks_error(errorT)->normalAcc();
    }
    break;
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

const Eigen::MatrixXd & TrajectoryTaskGeneric::jac() const
{
  switch(backend_)
  {
    case Backend::Tasks:
    {
      if(selectorT_)
      {
        return tasks_selector(selectorT_)->jac();
      }
      return tasks_error(errorT)->jac();
    }
    break;
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void TrajectoryTaskGeneric::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
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

void TrajectoryTaskGeneric::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  MetaTask::addToGUI(gui);
  gui.addElement(
      {"Tasks", name_},
      mc_rtc::gui::ArrayInput(
          "refVel", [this]() { return this->refVel(); }, [this](const Eigen::VectorXd & v) { this->refVel(v); }),
      mc_rtc::gui::ArrayInput(
          "refAccel", [this]() { return this->refAccel(); }, [this](const Eigen::VectorXd & v) { this->refAccel(v); }));
  gui.addElement({"Tasks", name_, "Gains"},
                 mc_rtc::gui::NumberInput(
                     "stiffness", [this]() { return this->stiffness(); },
                     [this](const double & s) { this->setGains(s, this->damping()); }),
                 mc_rtc::gui::NumberInput(
                     "damping", [this]() { return this->damping(); },
                     [this](const double & d) { this->setGains(this->stiffness(), d); }),
                 mc_rtc::gui::NumberInput(
                     "stiffness & damping", [this]() { return this->stiffness(); },
                     [this](const double & g) { this->stiffness(g); }),
                 mc_rtc::gui::NumberInput(
                     "weight", [this]() { return this->weight(); }, [this](const double & w) { this->weight(w); }));
  gui.addElement({"Tasks", name_, "Gains", "Dimensional"},
                 mc_rtc::gui::ArrayInput(
                     "stiffness", [this]() { return this->dimStiffness(); },
                     [this](const Eigen::VectorXd & v) { this->setGains(v, this->dimDamping()); }),
                 mc_rtc::gui::ArrayInput(
                     "damping", [this]() { return this->dimDamping(); },
                     [this](const Eigen::VectorXd & v) { this->setGains(this->dimStiffness(), v); }),
                 mc_rtc::gui::ArrayInput(
                     "stiffness & damping", [this]() { return this->dimStiffness(); },
                     [this](const Eigen::VectorXd & v) { this->stiffness(v); }));
}

void TrajectoryTaskGeneric::addToLogger(mc_rtc::Logger & logger)
{
  // clang-format off
  logger.addLogEntries(this,
                       name_ + "_damping", [this]() { return damping_(0); },
                       name_ + "_stiffness", [this]() { return stiffness_(0); });
  // clang-format on
  MC_RTC_LOG_HELPER(name_ + "_weight", weight_);
  MC_RTC_LOG_GETTER(name_ + "_dimWeight", dimWeight);
  MC_RTC_LOG_HELPER(name_ + "_dimDamping", damping_);
  MC_RTC_LOG_HELPER(name_ + "_dimStiffness", stiffness_);
  MC_RTC_LOG_HELPER(name_ + "_refVel", refVel_);
  MC_RTC_LOG_HELPER(name_ + "_refAccel", refAccel_);
}

std::function<bool(const mc_tasks::MetaTask & task, std::string &)> TrajectoryTaskGeneric::buildCompletionCriteria(
    double dt,
    const mc_rtc::Configuration & config) const
{
  return MetaTask::buildCompletionCriteria(dt, config);
}

} // namespace mc_tasks
