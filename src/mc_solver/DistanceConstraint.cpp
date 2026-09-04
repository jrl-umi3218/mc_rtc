/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/DistanceConstraint.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TVMQPSolver.h>
#include <mc_solver/TasksQPSolver.h>

#include <mc_tvm/DistanceFunction.h>

#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/configuration_io.h>

#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/Label.h>

#include <Tasks/QPConstr.h>

#include "mc_rbdyn/DistanceLimit.h"
#include <tvm/task_dynamics/VelocityDamper.h>

#include "utils/jointsToSelector.h"

namespace mc_solver
{

namespace details
{

struct TVMDistanceConstraint
{
  struct DistanceLimitData
  {
    DistanceLimitData(int id, const mc_rbdyn::DistanceLimit & dl) : id(id), distLim(dl) {}
    int id;
    mc_rbdyn::DistanceLimit distLim;
    mc_tvm::DistanceFunctionPtr function;
    tvm::TaskWithRequirementsPtr task;
  };
  /** All distances handled by this constraint */
  std::vector<DistanceLimitData> data_;
  /** Solver this has been added to */
  mc_solver::TVMQPSolver * solver;

  auto getData(const mc_rbdyn::DistanceLimit & dl)
  {
    return std::find_if(data_.begin(), data_.end(), [&](const auto & d) { return d.distLim == dl; });
  }

  auto getData(int id)
  {
    return std::find_if(data_.begin(), data_.end(), [&](const auto & d) { return d.id == id; });
  }

  template<bool Delete>
  std::vector<DistanceLimitData>::iterator removeOrDeleteDistanceLimit(TVMQPSolver & solver,
                                                                       std::vector<DistanceLimitData>::iterator it)
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

  void deleteDistanceLimit(TVMQPSolver & solver, const mc_rbdyn::DistanceLimit & dl)
  {
    removeOrDeleteDistanceLimit<true>(solver, getData(dl));
  }

  void deleteDistanceLimit(TVMQPSolver & solver, int id) { removeOrDeleteDistanceLimit<true>(solver, getData(id)); }

  void deleteDistanceLimits(mc_solver::TVMQPSolver & solver)
  {
    for(auto it = data_.begin(); it != data_.end(); ++it) { removeOrDeleteDistanceLimit<false>(solver, it); }
  }

  void clear()
  {
    if(!solver)
    {
      data_.clear();
      return;
    }
    auto it = data_.begin();
    while(it != data_.end()) { it = removeOrDeleteDistanceLimit<true>(*solver, it); }
  }

  DistanceLimitData & createDistanceLimit(TVMQPSolver & solver,
                                          const mc_rbdyn::Robot & r1,
                                          const mc_rbdyn::Robot & r2,
                                          const mc_rbdyn::DistanceLimit & dl,
                                          int id,
                                          const Eigen::VectorXd & r1Selector,
                                          const Eigen::VectorXd & r2Selector)
  {
    data_.push_back({id, dl});
    auto & data = data_.back();
    auto & c1 = r1.tvmConvex(dl.body1);
    auto & c2 = r2.tvmConvex(dl.body2);
    data.function = std::make_shared<mc_tvm::DistanceFunction>(c1, c2, r1Selector, r2Selector, solver.dt());
    return data;
  }

  void addDistanceLimit(TVMQPSolver & solver, DistanceLimitData & data)
  {
    const auto & dl = data.distLim;

    if(dl.iDist > dl.sDist)
    {
      // Lower distance bound: distance > sDist
      data.task = solver.problem().add(
          data.function >= 0.,
          tvm::task_dynamics::VelocityDamper(
              solver.dt(), {dl.iDist, dl.sDist, dl.damping, mc_solver::DistanceConstraint::defaultDampingOffset},
              tvm::constant::big_number),
          {tvm::requirements::PriorityLevel(0)});
    }
    else
    {
      // Upper distance bound: distance < sDist
      data.task = solver.problem().add(
          data.function <= 0.,
          tvm::task_dynamics::VelocityDamper(
              solver.dt(), {-dl.iDist, -dl.sDist, dl.damping, mc_solver::DistanceConstraint::defaultDampingOffset},
              tvm::constant::big_number),
          {tvm::requirements::PriorityLevel(0)});
    }
  }
};

} // namespace details

/** Helper to cast the constraint */
static inline mc_rtc::void_ptr_caster<tasks::qp::DistanceConstr> tasks_constraint{};
static inline mc_rtc::void_ptr_caster<details::TVMDistanceConstraint> tvm_constraint{};

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
  if(!match)
  {
    mc_rtc::log::error_and_throw("No match found for distance limit wildcard {} in {}", body, robot.name());
  }
  return true;
}

static mc_rtc::void_ptr make_constraint(QPSolver::Backend backend, const mc_rbdyn::Robots & robots, double timeStep)
{
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
      return mc_rtc::make_void_ptr<tasks::qp::DistanceConstr>(robots.mbs(), timeStep);
    case QPSolver::Backend::TVM:
      return mc_rtc::make_void_ptr<details::TVMDistanceConstraint>();
    default:
      mc_rtc::log::error_and_throw("[DistanceConstr] Not implemented for solver backend: {}", backend);
  }
}

DistanceConstraint::DistanceConstraint(const mc_rbdyn::Robots & robots,
                                       unsigned int r1Index,
                                       unsigned int r2Index,
                                       double timeStep)
: constraint_(make_constraint(backend_, robots, timeStep)), r1Index(r1Index), r2Index(r2Index), dlId(0), dlIdDict()
{
}

bool DistanceConstraint::removeDistanceLimit(QPSolver & solver, const mc_rbdyn::DistanceLimit & dl)
{
  const auto & robots = solver.robots();
  const mc_rbdyn::Robot & r1 = robots.robot(r1Index);
  const mc_rbdyn::Robot & r2 = robots.robot(r2Index);

  auto on_b1_wildcard = [&](const std::string & nb1)
  {
    auto nDl = dl;
    nDl.body1 = nb1;
    removeDistanceLimit(solver, nDl);
  };

  auto on_b2_wildcard = [&](const std::string & nb2)
  {
    auto nDl = dl;
    nDl.body2 = nb2;
    removeDistanceLimit(solver, nDl);
  };

  if(handle_wildcard(r1, dl.body1, on_b1_wildcard) || handle_wildcard(r2, dl.body2, on_b2_wildcard)) { return true; }

  auto p = __popDistanceLimitId(dl);

  if(!p.second.isNone())
  {
    if(monitored_.count(p.first)) { toggleDistanceLimitMonitor(p.first, &p.second); }

    category_.push_back("Monitors");
    std::string name = "Monitor " + p.second.body1 + "/" + p.second.body2;
    gui_->removeElement(category_, name);
    category_.pop_back();

    dls.erase(std::find(dls.begin(), dls.end(), p.second));

    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto distConstr = tasks_constraint(constraint_);
        auto & qpsolver = tasks_solver(solver);
        bool ret = distConstr->rmDistanceLimit(p.first);

        if(ret)
        {
          distConstr->updateNrVars({}, qpsolver.data());
          qpsolver.updateConstrSize();
        }

        return ret;
      }

      case QPSolver::Backend::TVM:
        tvm_constraint(constraint_)->deleteDistanceLimit(tvm_solver(solver), p.second);
        break;

      default:
        break;
    }
  }

  return false;
}

void DistanceConstraint::removeDistanceLimits(QPSolver & solver, const std::vector<mc_rbdyn::DistanceLimit> & dls)
{
  for(const auto & dl : dls) { removeDistanceLimit(solver, dl); }
}

bool DistanceConstraint::removeDistanceLimitByBody(QPSolver & solver,
                                                   const std::string & b1Name,
                                                   const std::string & b2Name)
{
  const auto & r1 = solver.robots().robot(r1Index);
  const auto & r2 = solver.robots().robot(r2Index);
  std::vector<mc_rbdyn::DistanceLimit> toRm;
  for(const auto & dl : dls)
  {
    if(r1.convex(dl.body1).first == b1Name && r2.convex(dl.body2).first == b2Name)
    {
      auto out = __popDistanceLimitId(dl);
      toRm.push_back(out.second);
      switch(backend_)
      {
        case QPSolver::Backend::Tasks:
        {
          auto distConstr = tasks_constraint(constraint_);
          distConstr->rmDistanceLimit(out.first);
          break;
        }
        case QPSolver::Backend::TVM:
          tvm_constraint(constraint_)->deleteDistanceLimit(tvm_solver(solver), out.first);
          break;
        default:
          break;
      }
      if(monitored_.count(out.first)) { toggleDistanceLimitMonitor(out.first, &out.second); }
      category_.push_back("Monitors");
      std::string name = "Monitor " + __keyByNames(out.second);
      gui_->removeElement(category_, name);
      category_.pop_back();
    }
  }
  for(const auto & it : toRm) { dls.erase(std::find(dls.begin(), dls.end(), it)); }
  if(toRm.size())
  {
    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto distConstr = tasks_constraint(constraint_);
        auto & qpsolver = tasks_solver(solver);
        distConstr->updateNrVars({}, qpsolver.data());
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

void DistanceConstraint::__addDistanceLimit(mc_solver::QPSolver & solver, const mc_rbdyn::DistanceLimit & dl)
{
  const auto & robots = solver.robots();
  const mc_rbdyn::Robot & r1 = robots.robot(r1Index);
  const mc_rbdyn::Robot & r2 = robots.robot(r2Index);
  if(dl.body1.size() == 0 || dl.body2.size() == 0)
  {
    mc_rtc::log::error("Attempted to add a distance limit without a specific body");
    return;
  }
  auto on_b1_wildcard = [&](const std::string & nb1)
  {
    auto nDl = dl;
    nDl.body1 = nb1;
    __addDistanceLimit(solver, nDl);
  };
  auto on_b2_wildcard = [&](const std::string & nb2)
  {
    auto nDl = dl;
    nDl.body2 = nb2;
    __addDistanceLimit(solver, nDl);
  };
  if(handle_wildcard(r1, dl.body1, on_b1_wildcard) || handle_wildcard(r2, dl.body2, on_b2_wildcard)) { return; }
  int dlId = __createDistanceLimitId(dl);
  if(dlId < 0) { return; }
  dls.push_back(dl);

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
          mc_rtc::log::error_and_throw("[DistanceConstraint] No joint named \"{}\" in robot \"{}\"", j,
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

  auto r1Selector = computeJointsSelector(dl.r1Joints, dl.r1JointsInactive, r1Index);
  auto r2Selector = r1Index == r2Index ? Eigen::VectorXd::Zero(0).eval()
                                       : computeJointsSelector(dl.r2Joints, dl.r2JointsInactive, r2Index);

  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto distConstr = tasks_constraint(constraint_);
      const auto & body1 = r1.convex(dl.body1);
      const auto & body2 = r2.convex(dl.body2);
      const sva::PTransformd & X_b1_c = r1.convexTransform(dl.body1);
      const sva::PTransformd & X_b2_c = r2.convexTransform(dl.body2);
      if(r1.mb().nrDof() == 0)
      {
        distConstr->addDistanceLimit(robots.mbs(), dlId, static_cast<int>(r2Index), body2.first, body2.second.get(),
                                     X_b2_c, static_cast<int>(r1Index), body1.first, body1.second.get(), X_b1_c,
                                     dl.iDist, dl.sDist, dl.damping, defaultDampingOffset, r2Selector, r1Selector);
      }
      else
      {
        distConstr->addDistanceLimit(robots.mbs(), dlId, static_cast<int>(r1Index), body1.first, body1.second.get(),
                                     X_b1_c, static_cast<int>(r2Index), body2.first, body2.second.get(), X_b2_c,
                                     dl.iDist, dl.sDist, dl.damping, defaultDampingOffset, r1Selector, r2Selector);
      }
      break;
    }
    case QPSolver::Backend::TVM:
    {
      auto & data = tvm_constraint(constraint_)
                        ->createDistanceLimit(tvm_solver(solver), r1, r2, dl, dlId, r1Selector, r2Selector);
      if(inSolver_) { tvm_constraint(constraint_)->addDistanceLimit(tvm_solver(solver), data); }
      break;
    }
    default:
      break;
  }
  addMonitorButton(dlId, dl);
}

void DistanceConstraint::addMonitorButton(int dlId, const mc_rbdyn::DistanceLimit & dl)
{
  if(gui_ && inSolver_)
  {
    auto & gui = *gui_;
    std::string name = dl.body1 + "/" + dl.body2;
    category_.push_back("Monitors");
    gui.addElement(category_, mc_rtc::gui::Checkbox(
                                  "Monitor " + name, [dlId, this]() { return monitored_.count(dlId) != 0; },
                                  [dlId, this]() { toggleDistanceLimitMonitor(dlId); }));
    category_.pop_back();
  }
}

void DistanceConstraint::toggleDistanceLimitMonitor(int dlId, const mc_rbdyn::DistanceLimit * dl_p)
{
  auto findDistanceLimitById = [this, dlId, &dl_p]()
  {
    if(dl_p) { return; }
    for(const auto & c : dlIdDict)
    {
      if(c.second.first == dlId)
      {
        dl_p = &c.second.second;
        return;
      }
    }
    mc_rtc::log::error_and_throw(
        "[DistanceConstraint] Attempted to toggleDistanceLimitMonitor on non-existent distance limit");
  };
  findDistanceLimitById();
  const auto & dl = *dl_p;
  auto & gui = *gui_;
  std::string label = dl.body1 + "::" + dl.body2;
  if(monitored_.count(dlId))
  {
    // Remove the monitor
    gui.removeElement(category_, label);
    category_.push_back("Arrows");
    gui.removeElement(category_, label);
    category_.pop_back();
    monitored_.erase(dlId);
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
        auto distConstr = tasks_constraint(constraint_);
        addMonitor([distConstr, dlId]() { return distConstr->getDistanceData(dlId).distance; }, // needs Tasks change?
                   [distConstr, dlId]() -> const Eigen::Vector3d & { return distConstr->getDistanceData(dlId).p1; },
                   [distConstr, dlId]() -> const Eigen::Vector3d & { return distConstr->getDistanceData(dlId).p2; });
        break;
      }
      case QPSolver::Backend::TVM:
      {
        auto distConstr = tvm_constraint(constraint_);
        auto fn = distConstr->getData(dlId)->function;
        addMonitor([fn]() { return fn->distance(); }, [fn]() -> const Eigen::Vector3d & { return fn->p1(); },
                   [fn]() -> const Eigen::Vector3d & { return fn->p2(); });
        break;
      }
      default:
        break;
    }
    monitored_.insert(dlId);
  }
}

void DistanceConstraint::addDistanceLimit(QPSolver & solver, const mc_rbdyn::DistanceLimit & dl)
{
  addDistanceLimits(solver, {dl});
}

void DistanceConstraint::addDistanceLimits(QPSolver & solver, const std::vector<mc_rbdyn::DistanceLimit> & dls)
{
  for(const auto & dl : dls) { __addDistanceLimit(solver, dl); }
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & distConstr = *tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      distConstr.updateNrVars({}, qpsolver.data());
      qpsolver.updateConstrSize();
      break;
    }
    case QPSolver::Backend::TVM:
      break;
    default:
      break;
  }
}

void DistanceConstraint::addToSolverImpl(QPSolver & solver)
{
  gui_ = solver.gui();
  const mc_rbdyn::Robot & r1 = solver.robots().robot(r1Index);
  const mc_rbdyn::Robot & r2 = solver.robots().robot(r2Index);
  category_ = {"DistanceLimits", r1.name() + "/" + r2.name()};
  gui_->addElement(category_, mc_rtc::gui::Checkbox("Automatic monitor", autoMonitor_));
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & distConstr = *tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      distConstr.addToSolver(solver.robots().mbs(), qpsolver.solver());
      break;
    }
    case QPSolver::Backend::TVM:
    {
      auto cstr = tvm_constraint(constraint_);
      for(auto & c : cstr->data_) { tvm_constraint(constraint_)->addDistanceLimit(tvm_solver(solver), c); }
      tvm_constraint(constraint_)->solver = &tvm_solver(solver);
      break;
    }
    default:
      break;
  }
  for(const auto & dl : dlIdDict) { addMonitorButton(dl.second.first, dl.second.second); }
}

void DistanceConstraint::update(QPSolver &)
{
  if(!autoMonitor_) { return; }
  auto getDistance = [this](int dlId)
  {
    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto distConstr = tasks_constraint(constraint_);
        return distConstr->getDistanceData(dlId).distance;
      }
      case QPSolver::Backend::TVM:
      {
        auto distConstr = tvm_constraint(constraint_);
        auto & fn = distConstr->getData(dlId)->function;
        return fn->distance();
      }
      default:
        mc_rtc::log::error_and_throw("Not implemented for this backend");
    }
  };
  for(const auto & [name, info] : dlIdDict)
  {
    const auto & [dlId, dl] = info;
    auto distance = getDistance(dlId);
    if(distance < dl.iDist && dl.iDist > dl.sDist && !monitored_.count(dlId)) { toggleDistanceLimitMonitor(dlId, &dl); }
    if(distance > dl.iDist && dl.iDist > dl.sDist && monitored_.count(dlId)) { toggleDistanceLimitMonitor(dlId, &dl); }
    if(distance > dl.iDist && dl.iDist < dl.sDist && !monitored_.count(dlId)) { toggleDistanceLimitMonitor(dlId, &dl); }
    if(distance < dl.iDist && dl.iDist < dl.sDist && monitored_.count(dlId)) { toggleDistanceLimitMonitor(dlId, &dl); }
  }
}

void DistanceConstraint::removeFromSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & distConstr = *tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      distConstr.removeFromSolver(qpsolver.solver());
      break;
    }
    case QPSolver::Backend::TVM:
    {
      tvm_constraint(constraint_)->deleteDistanceLimits(tvm_solver(solver));
      tvm_constraint(constraint_)->solver = nullptr;
      break;
    }
    default:
      break;
  }
  gui_->removeCategory(category_);
}

void DistanceConstraint::reset()
{
  dls.clear();
  dlIdDict.clear();
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

std::string DistanceConstraint::__keyByNames(const mc_rbdyn::DistanceLimit & dl)
{
  return dl.body1 + "/" + dl.body2 + (dl.iDist > dl.sDist ? "_min" : "_max");
}

int DistanceConstraint::__createDistanceLimitId(const mc_rbdyn::DistanceLimit & dl)
{
  std::string key = __keyByNames(dl);

  auto it = dlIdDict.find(key);
  if(it != dlIdDict.end()) { return -1; }

  int dlId = this->dlId;
  dlIdDict[key] = std::pair<int, mc_rbdyn::DistanceLimit>(dlId, dl);
  this->dlId += 1;
  return dlId;
}

std::pair<int, mc_rbdyn::DistanceLimit> DistanceConstraint::__popDistanceLimitId(const mc_rbdyn::DistanceLimit & dl)
{
  std::string key = __keyByNames(dl);

  if(dlIdDict.count(key))
  {
    std::pair<int, mc_rbdyn::DistanceLimit> p = dlIdDict[key];
    dlIdDict.erase(key);
    return p;
  }

  return std::pair<unsigned int, mc_rbdyn::DistanceLimit>(0, mc_rbdyn::DistanceLimit());
}

bool DistanceConstraint::hasDistanceLimit(const std::string & c1, const std::string & c2) const noexcept
{
  auto it = std::find_if(dls.begin(), dls.end(), [&](const auto & c) { return c.body1 == c1 && c.body2 == c2; });
  return it != dls.end();
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintSetLoader::register_load_function(
    "distance",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      auto ret = std::make_shared<mc_solver::DistanceConstraint>(
          solver.robots(), robotIndexFromConfig(config, solver.robots(), "distance", false, "r1Index", "r1", ""),
          robotIndexFromConfig(config, solver.robots(), "distance", false, "r2Index", "r2", ""), solver.dt());
      ret->automaticMonitor(config("automaticMonitor", true));
      if(ret->r1Index == ret->r2Index)
      {
        if(config("useCommon", false))
        {
          ret->addDistanceLimits(solver, solver.robots().robotModule(ret->r1Index).commonSelfCollisions());
        }
        else if(config("useMinimal", false))
        {
          ret->addDistanceLimits(solver, solver.robots().robotModule(ret->r1Index).minimalSelfCollisions());
        }
      }
      std::vector<mc_rbdyn::DistanceLimit> distLims = config("distances", std::vector<mc_rbdyn::DistanceLimit>{});
      ret->addDistanceLimits(solver, distLims);
      return ret;
    });
} // namespace
