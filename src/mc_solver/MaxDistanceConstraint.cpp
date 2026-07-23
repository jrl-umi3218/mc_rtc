/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/MaxDistanceConstraint.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TVMQPSolver.h>
#include <mc_solver/TasksQPSolver.h>

#include <mc_tvm/MaxDistanceFunction.h>

#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/configuration_io.h>

#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Label.h>

#include <Tasks/QPConstr.h>

#include <tvm/task_dynamics/VelocityDamper.h>

#include "utils/jointsToSelector.h"

namespace mc_solver
{

namespace details
{

struct TVMMaxDistanceConstraint
{
  struct MaxDistData
  {
    MaxDistData(int id, const mc_rbdyn::MaxDist & maxD) : id(id), maxDistance(maxD) {}
    int id;
    mc_rbdyn::MaxDist maxDistance;
    mc_tvm::MaxDistanceFunctionPtr function;
    tvm::TaskWithRequirementsPtr task;
  };
  /** All max distances handled by this constraint */
  std::vector<MaxDistData> data_;
  /** Solver this has been added to */
  mc_solver::TVMQPSolver * solver;

  auto getData(const mc_rbdyn::MaxDist & maxD)
  {
    return std::find_if(data_.begin(), data_.end(), [&](const auto & d) { return d.maxDistance == maxD; });
  }

  auto getData(int id)
  {
    return std::find_if(data_.begin(), data_.end(), [&](const auto & d) { return d.id == id; });
  }

  template<bool Delete>
  std::vector<MaxDistData>::iterator removeOrDeleteMaxDist(TVMQPSolver & solver, std::vector<MaxDistData>::iterator it)
  {
    if(it == data_.end()) { return data_.end(); }
    if(it->task)
    {
      solver.problem().remove(*it->task);
      if constexpr(!Delete) { it->task.reset(); }
    }
    if constexpr(Delete) { return data_.erase(it); }
    else
    {
      return it;
    }
  }

  void deleteMaxDist(TVMQPSolver & solver, const mc_rbdyn::MaxDist & maxD)
  {
    removeOrDeleteMaxDist<true>(solver, getData(maxD));
  }

  void deleteMaxDist(TVMQPSolver & solver, int id) { removeOrDeleteMaxDist<true>(solver, getData(id)); }

  void removeMaxDists(mc_solver::TVMQPSolver & solver)
  {
    for(auto it = data_.begin(); it != data_.end(); ++it) { removeOrDeleteMaxDist<false>(solver, it); }
  }

  void clear()
  {
    if(!solver)
    {
      data_.clear();
      return;
    }
    auto it = data_.begin();
    while(it != data_.end()) { it = removeOrDeleteMaxDist<true>(*solver, it); }
  }

  MaxDistData & createMaxDist(TVMQPSolver & solver,
                              const mc_rbdyn::Robot & r1,
                              const mc_rbdyn::Robot & r2,
                              const mc_rbdyn::MaxDist & maxD,
                              int id,
                              const Eigen::VectorXd & r1Selector,
                              const Eigen::VectorXd & r2Selector)
  {
    data_.push_back({id, maxD});
    auto & data = data_.back();
    auto & c1 = r1.tvmConvex(maxD.body1);
    auto & c2 = r2.tvmConvex(maxD.body2);
    data.function = std::make_shared<mc_tvm::MaxDistanceFunction>(c1, c2, r1Selector, r2Selector, solver.dt());
    return data;
  }

  void addMaxDist(TVMQPSolver & solver, MaxDistData & data)
  {
    const auto & maxD = data.maxDistance;
    data.task = solver.problem().add(
        data.function >= 0.,
        tvm::task_dynamics::VelocityDamper(
            solver.dt(), {maxD.iDist, maxD.sDist, maxD.damping, mc_solver::MaxDistanceConstraint::defaultDampingOffset},
            tvm::constant::big_number),
        {tvm::requirements::PriorityLevel(0)});
  }
};

} // namespace details

/** Helper to cast the constraint */
static inline mc_rtc::void_ptr_caster<tasks::qp::MaxDistanceConstr> tasks_constraint{};
static inline mc_rtc::void_ptr_caster<details::TVMMaxDistanceConstraint> tvm_constraint{};

/** Helper for wildcard
 *
 * Returns false if body is not a wildcard
 *
 * Throws if body is a wildcard but there's no match in robot
 */
template<typename Callback>
bool handle_wildcard(const mc_rbdyn::Robot & robot, const std::string & body, Callback cb)
{
  if(body.back() != '*') { return false; }
  std::string search = body.substr(0, body.size() - 1);
  bool match = false;
  for(const auto & convex : robot.convexes())
  {
    const auto & cName = convex.first;
    if(cName.size() < search.size()) { continue; }
    if(cName.substr(0, search.size()) == search)
    {
      match = true;
      cb(cName);
    }
  }
  if(!match) { mc_rtc::log::error_and_throw("No match found for max distance wildcard {} in {}", body, robot.name()); }
  return true;
}

static mc_rtc::void_ptr make_constraint(QPSolver::Backend backend, const mc_rbdyn::Robots & robots, double timeStep)
{
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
      return mc_rtc::make_void_ptr<tasks::qp::MaxDistanceConstr>(robots.mbs(), timeStep);
    case QPSolver::Backend::TVM:
      return mc_rtc::make_void_ptr<details::TVMMaxDistanceConstraint>();
    default:
      mc_rtc::log::error_and_throw("[MaxDistanceConstr] Not implemented for solver backend: {}", backend);
  }
}

MaxDistanceConstraint::MaxDistanceConstraint(const mc_rbdyn::Robots & robots,
                                             unsigned int r1Index,
                                             unsigned int r2Index,
                                             double timeStep)
: constraint_(make_constraint(backend_, robots, timeStep)), r1Index(r1Index), r2Index(r2Index), maxDistId(0),
  maxDistIdDict()
{
}

bool MaxDistanceConstraint::removeMaxDist(QPSolver & solver, const std::string & b1Name, const std::string & b2Name)
{
  const auto & robots = solver.robots();
  const mc_rbdyn::Robot & r1 = robots.robot(r1Index);
  const mc_rbdyn::Robot & r2 = robots.robot(r2Index);
  auto on_b1_wildcard = [&](const std::string & nb1) { removeMaxDist(solver, nb1, b2Name); };
  auto on_b2_wildcard = [&](const std::string & nb2) { removeMaxDist(solver, b1Name, nb2); };
  if(handle_wildcard(r1, b1Name, on_b1_wildcard) || handle_wildcard(r2, b2Name, on_b2_wildcard)) { return true; }
  auto p = __popMaxDistId(b1Name, b2Name);
  if(!p.second.isNone())
  {
    if(monitored_.count(p.first)) { toggleMaxDistMonitor(p.first, &p.second); }
    category_.push_back("Monitors");
    std::string name = "Monitor " + p.second.body1 + "/" + p.second.body2;
    gui_->removeElement(category_, name);
    category_.pop_back();
    maxDists.erase(std::find(maxDists.begin(), maxDists.end(), p.second));
    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto maxDConstr = tasks_constraint(constraint_);
        auto & qpsolver = tasks_solver(solver);
        bool ret = maxDConstr->rmMaxDist(p.first);
        if(ret)
        {
          maxDConstr->updateNrVars({}, qpsolver.data());
          qpsolver.updateConstrSize();
        }
        return ret;
      }
      case QPSolver::Backend::TVM:
        tvm_constraint(constraint_)->deleteMaxDist(tvm_solver(solver), p.second);
        break;
      default:
        break;
    }
  }
  return false;
}

void MaxDistanceConstraint::removeMaxDists(QPSolver & solver, const std::vector<mc_rbdyn::MaxDist> & maxDs)
{
  for(const auto & m : maxDs) { removeMaxDist(solver, m.body1, m.body2); }
}

bool MaxDistanceConstraint::removeMaxDistByBody(QPSolver & solver,
                                                const std::string & b1Name,
                                                const std::string & b2Name)
{
  const auto & r1 = solver.robots().robot(r1Index);
  const auto & r2 = solver.robots().robot(r2Index);
  std::vector<mc_rbdyn::MaxDist> toRm;
  for(const auto & maxD : maxDists)
  {
    if(r1.convex(maxD.body1).first == b1Name && r2.convex(maxD.body2).first == b2Name)
    {
      auto out = __popMaxDistId(maxD.body1, maxD.body2);
      toRm.push_back(out.second);
      switch(backend_)
      {
        case QPSolver::Backend::Tasks:
        {
          auto maxDistConstr = tasks_constraint(constraint_);
          maxDistConstr->rmMaxDist(out.first);
          break;
        }
        case QPSolver::Backend::TVM:
          tvm_constraint(constraint_)->deleteMaxDist(tvm_solver(solver), out.first);
          break;
        default:
          break;
      }
      if(monitored_.count(out.first)) { toggleMaxDistMonitor(out.first, &out.second); }
      category_.push_back("Monitors");
      std::string name = "Monitor " + out.second.body1 + "/" + out.second.body2;
      gui_->removeElement(category_, name);
      category_.pop_back();
    }
  }
  for(const auto & it : toRm) { maxDists.erase(std::find(maxDists.begin(), maxDists.end(), it)); }
  if(toRm.size())
  {
    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto maxDistConstr = tasks_constraint(constraint_);
        auto & qpsolver = tasks_solver(solver);
        maxDistConstr->updateNrVars({}, qpsolver.data());
        qpsolver.updateConstrSize();
        break;
      }
      case QPSolver::Backend::TVM:
        break;
      default:
        break;
    }
  }
  return toRm.size() > 0;
}

void MaxDistanceConstraint::__addMaxDist(mc_solver::QPSolver & solver, const mc_rbdyn::MaxDist & m)
{
  const auto & robots = solver.robots();
  const mc_rbdyn::Robot & r1 = robots.robot(r1Index);
  const mc_rbdyn::Robot & r2 = robots.robot(r2Index);
  if(m.body1.size() == 0 || m.body2.size() == 0)
  {
    mc_rtc::log::error("Attempted to add a max distance without a specific body");
    return;
  }
  auto on_b1_wildcard = [&](const std::string & nb1)
  {
    auto nM = m;
    nM.body1 = nb1;
    __addMaxDist(solver, nM);
  };
  auto on_b2_wildcard = [&](const std::string & nb2)
  {
    auto nM = m;
    nM.body2 = nb2;
    __addMaxDist(solver, nM);
  };
  if(handle_wildcard(r1, m.body1, on_b1_wildcard) || handle_wildcard(r2, m.body2, on_b2_wildcard)) { return; }
  int maxDId = __createMaxDId(m);
  if(maxDId < 0) { return; }
  maxDists.push_back(m);

  auto computeJointsSelector =
      [&robots](const std::optional<std::vector<std::string>> & joints, bool inactive, auto rIndex)
  {
    if(joints)
    {
      // check that all joints exist
      for(const auto & j : *joints)
      {
        if(!robots.robot(rIndex).hasJoint(j))
        {
          mc_rtc::log::error_and_throw("[MaxDistanceConstraint] No joint named \"{}\" in robot \"{}\"", j,
                                       robots.robot(rIndex).name());
        }
      }
      if(inactive) { return jointsToSelector<false>(robots.robot(rIndex), *joints); }
      else
      {
        return jointsToSelector<true>(robots.robot(rIndex), *joints);
      }
    }
    else
    {
      return Eigen::VectorXd::Zero(0).eval();
    }
  };

  auto r1Selector = computeJointsSelector(m.r1Joints, m.r1JointsInactive, r1Index);
  auto r2Selector = r1Index == r2Index ? Eigen::VectorXd::Zero(0).eval()
                                       : computeJointsSelector(m.r2Joints, m.r2JointsInactive, r2Index);

  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto maxDistConstr = tasks_constraint(constraint_);
      const auto & body1 = r1.convex(m.body1);
      const auto & body2 = r2.convex(m.body2);
      const sva::PTransformd & X_b1_c = r1.collisionTransform(m.body1);
      const sva::PTransformd & X_b2_c = r2.collisionTransform(m.body2);
      if(r1.mb().nrDof() == 0)
      {
        maxDistConstr->addMaxDist(robots.mbs(), maxDistId, static_cast<int>(r2Index), body2.first, body2.second.get(),
                                  X_b2_c, static_cast<int>(r1Index), body1.first, body1.second.get(), X_b1_c, m.iDist,
                                  m.sDist, m.damping, defaultDampingOffset, r2Selector, r1Selector);
      }
      else
      {
        maxDistConstr->addMaxDist(robots.mbs(), maxDistId, static_cast<int>(r1Index), body1.first, body1.second.get(),
                                  X_b1_c, static_cast<int>(r2Index), body2.first, body2.second.get(), X_b2_c, m.iDist,
                                  m.sDist, m.damping, defaultDampingOffset, r1Selector, r2Selector);
      }
      break;
    }
    case QPSolver::Backend::TVM:
    {
      auto & data =
          tvm_constraint(constraint_)->createMaxDist(tvm_solver(solver), r1, r2, m, maxDistId, r1Selector, r2Selector);
      if(inSolver_) { tvm_constraint(constraint_)->addMaxDist(tvm_solver(solver), data); }
      break;
    }
    default:
      break;
  }
  addMonitorButton(maxDistId, m);
}

void MaxDistanceConstraint::addMonitorButton(int maxDistId, const mc_rbdyn::MaxDist & m)
{
  if(gui_ && inSolver_)
  {
    auto & gui = *gui_;
    std::string name = m.body1 + "/" + m.body2;
    category_.push_back("Monitors");
    gui.addElement(category_, mc_rtc::gui::Checkbox(
                                  "Monitor " + name, [maxDistId, this]() { return monitored_.count(maxDistId) != 0; },
                                  [maxDistId, this]() { toggleMaxDistMonitor(maxDistId); }));
    category_.pop_back();
  }
}

void MaxDistanceConstraint::toggleMaxDistMonitor(int maxDistId, const mc_rbdyn::MaxDist * m_p)
{
  auto findMaxDistById = [this, maxDistId, &m_p]()
  {
    if(m_p) { return; }
    for(const auto & m : maxDistIdDict)
    {
      if(m.second.first == maxDistId)
      {
        m_p = &m.second.second;
        return;
      }
    }
    mc_rtc::log::error_and_throw(
        "[toggleMaxDistMonitor] Attempted to toggleMaxDistMonitor on non-existent max distance");
  };
  findMaxDistById();
  const auto & m = *m_p;
  auto & gui = *gui_;
  std::string label = m.body1 + "::" + m.body2;
  if(monitored_.count(maxDistId))
  {
    // Remove the monitor
    gui.removeElement(category_, label);
    category_.push_back("Arrows");
    gui.removeElement(category_, label);
    category_.pop_back();
    monitored_.erase(maxDistId);
  }
  else
  {
    auto addMonitor = [&](auto && distance_callback, auto && p1_callback, auto && p2_callback)
    {
      gui.addElement(category_, mc_rtc::gui::Label(label, [distance_callback]()
                                                   { return fmt::format("{:0.2f} cm", 100.0 * distance_callback()); }));
      category_.push_back("Arrows");
      gui.addElement(category_, mc_rtc::gui::Arrow(label, p1_callback, p2_callback));
      category_.pop_back();
    };
    // Add the monitor
    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto maxDistConstr = tasks_constraint(constraint_);
        addMonitor([maxDistConstr, maxDistId]() { return maxDistConstr->getMaxDistData(maxDistId).distance; },
                   [maxDistConstr, maxDistId]() -> const Eigen::Vector3d &
                   { return maxDistConstr->getMaxDistData(maxDistId).p1; },
                   [maxDistConstr, maxDistId]() -> const Eigen::Vector3d &
                   { return maxDistConstr->getMaxDistData(maxDistId).p2; });
        break;
      }
      case QPSolver::Backend::TVM:
      {
        auto maxDistConstr = tvm_constraint(constraint_);
        auto fn = maxDistConstr->getData(maxDistId)->function;
        addMonitor([fn]() { return fn->distance(); }, [fn]() -> const Eigen::Vector3d & { return fn->p1(); },
                   [fn]() -> const Eigen::Vector3d & { return fn->p2(); });
        break;
      }
      default:
        break;
    }
    monitored_.insert(maxDistId);
  }
}

void MaxDistanceConstraint::addMaxDist(QPSolver & solver, const mc_rbdyn::MaxDist & m)
{
  addMaxDists(solver, {m});
}

void MaxDistanceConstraint::addMaxDists(QPSolver & solver, const std::vector<mc_rbdyn::MaxDist> & mDs)
{
  for(const auto & m : mDs) { __addMaxDist(solver, m); }
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & maxDistConstr = *tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      maxDistConstr.updateNrVars({}, qpsolver.data());
      qpsolver.updateConstrSize();
      break;
    }
    case QPSolver::Backend::TVM:
      break;
    default:
      break;
  }
}

void MaxDistanceConstraint::addToSolverImpl(QPSolver & solver)
{
  gui_ = solver.gui();
  const mc_rbdyn::Robot & r1 = solver.robots().robot(r1Index);
  const mc_rbdyn::Robot & r2 = solver.robots().robot(r2Index);
  category_ = {"MaxDists", r1.name() + "/" + r2.name()};
  gui_->addElement(category_, mc_rtc::gui::Checkbox("Automatic monitor", autoMonitor_));
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & maxdistConstr = *tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      maxdistConstr.addToSolver(solver.robots().mbs(), qpsolver.solver());
      break;
    }
    case QPSolver::Backend::TVM:
    {
      auto cstr = tvm_constraint(constraint_);
      for(auto & c : cstr->data_) { tvm_constraint(constraint_)->addMaxDist(tvm_solver(solver), c); }
      tvm_constraint(constraint_)->solver = &tvm_solver(solver);
      break;
    }
    default:
      break;
  }
  for(const auto & maxDs : maxDistIdDict) { addMonitorButton(maxDs.second.first, maxDs.second.second); }
}

void MaxDistanceConstraint::update(QPSolver &)
{
  mc_rtc::log::warning("DEBUG MaxDistanceConstraint update");
  if(!autoMonitor_) { return; }
  auto getDistance = [this](int maxDistId)
  {
    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto maxDistConstr = tasks_constraint(constraint_);
        return maxDistConstr->getMaxDistData(maxDistId).distance;
      }
      case QPSolver::Backend::TVM:
      {
        auto maxDistConstr = tvm_constraint(constraint_);
        auto & fn = maxDistConstr->getData(maxDistId)->function;
        return fn->distance();
      }
      default:
        mc_rtc::log::error_and_throw("Not implemented for this backend");
    }
  };
  for(const auto & [name, info] : maxDistIdDict)
  {
    const auto & [maxDistId, maxDD] = info;
    auto distance = getDistance(maxDistId);
    if(distance > maxDD.iDist && !monitored_.count(maxDistId)) { toggleMaxDistMonitor(maxDistId, &maxDD); }
    if(distance < maxDD.iDist && monitored_.count(maxDistId)) { toggleMaxDistMonitor(maxDistId, &maxDD); }
  }
}

void MaxDistanceConstraint::removeFromSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & maxDistConstr = *tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      maxDistConstr.removeFromSolver(qpsolver.solver());
      break;
    }
    case QPSolver::Backend::TVM:
    {
      tvm_constraint(constraint_)->removeMaxDists(tvm_solver(solver));
      tvm_constraint(constraint_)->solver = nullptr;
      break;
    }
    default:
      break;
  }
  gui_->removeCategory(category_);
}

void MaxDistanceConstraint::reset()
{
  maxDists.clear();
  maxDistIdDict.clear();
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
      tasks_constraint(constraint_)->reset();
      break;
    case QPSolver::Backend::TVM:
      tvm_constraint(constraint_)->clear();
      break;
    default:
      break;
  }
  if(gui_) { gui_->removeCategory(category_); }
}

std::string MaxDistanceConstraint::__keyByNames(const std::string & name1, const std::string & name2)
{
  return name1 + name2;
}

int MaxDistanceConstraint::__createMaxDId(const mc_rbdyn::MaxDist & m)
{
  std::string key = __keyByNames(m.body1, m.body2);
  auto it = maxDistIdDict.find(key);
  if(it != maxDistIdDict.end()) { return -1; }
  int maxDistId = this->maxDistId;
  maxDistIdDict[key] = std::pair<int, mc_rbdyn::MaxDist>(maxDistId, m);
  this->maxDistId += 1;
  return maxDistId;
}

std::pair<int, mc_rbdyn::MaxDist> MaxDistanceConstraint::__popMaxDistId(const std::string & name1,
                                                                        const std::string & name2)
{
  std::string key = __keyByNames(name1, name2);
  if(maxDistIdDict.count(key))
  {
    std::pair<int, mc_rbdyn::MaxDist> p = maxDistIdDict[key];
    maxDistIdDict.erase(key);
    return p;
  }
  return std::pair<unsigned int, mc_rbdyn::MaxDist>(0, mc_rbdyn::MaxDist());
}

bool MaxDistanceConstraint::hasMaxDist(const std::string & c1, const std::string & c2) const noexcept
{
  auto it =
      std::find_if(maxDists.begin(), maxDists.end(), [&](const auto & c) { return c.body1 == c1 && c.body2 == c2; });
  return it != maxDists.end();
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintSetLoader::register_load_function(
    "maxDistance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      auto ret = std::make_shared<mc_solver::MaxDistanceConstraint>(
          solver.robots(), robotIndexFromConfig(config, solver.robots(), "maxDistance", false, "r1Index", "r1", ""),
          robotIndexFromConfig(config, solver.robots(), "maxDistance", false, "r2Index", "r2", ""), solver.dt());
      ret->automaticMonitor(config("automaticMonitor", true));
      if(ret->r1Index == ret->r2Index)
      {
        if(config("useCommon", false))
        {
          ret->addMaxDists(solver, solver.robots().robotModule(ret->r1Index).commonMaxDistances());
        }
      }
      std::vector<mc_rbdyn::MaxDist> maxDists = config("maxDistance", std::vector<mc_rbdyn::MaxDist>{});
      ret->addMaxDists(solver, maxDists);
      return ret;
    });
} // namespace
