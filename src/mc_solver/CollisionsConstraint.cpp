/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_solver/CollisionsConstraint.h>

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TVMQPSolver.h>
#include <mc_solver/TasksQPSolver.h>

#include <mc_tvm/CollisionFunction.h>

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

struct TVMCollisionConstraint
{
  struct CollisionData
  {
    CollisionData(int id, const mc_rbdyn::Collision & col) : id(id), collision(col) {}
    int id;
    mc_rbdyn::Collision collision;
    mc_tvm::CollisionFunctionPtr function;
    tvm::TaskWithRequirementsPtr task;
  };
  /** All collisions handled by this constraint */
  std::vector<CollisionData> data_;
  /** Solver this has been added to */
  mc_solver::TVMQPSolver * solver;

  auto getData(const mc_rbdyn::Collision & col)
  {
    return std::find_if(data_.begin(), data_.end(), [&](const auto & d) { return d.collision == col; });
  }

  auto getData(int id)
  {
    return std::find_if(data_.begin(), data_.end(), [&](const auto & d) { return d.id == id; });
  }

  template<bool Delete>
  std::vector<CollisionData>::iterator removeOrDeleteCollision(TVMQPSolver & solver,
                                                               std::vector<CollisionData>::iterator it)
  {
    if(it == data_.end()) { return data_.end(); }
    if(it->task)
    {
      solver.problem().remove(*it->task);
      if constexpr(!Delete) { it->task.reset(); }
    }
    if constexpr(Delete) { return data_.erase(it); }
    else { return it; }
  }

  void deleteCollision(TVMQPSolver & solver, const mc_rbdyn::Collision & col)
  {
    removeOrDeleteCollision<true>(solver, getData(col));
  }

  void deleteCollision(TVMQPSolver & solver, int id) { removeOrDeleteCollision<true>(solver, getData(id)); }

  void removeCollisions(mc_solver::TVMQPSolver & solver)
  {
    for(auto it = data_.begin(); it != data_.end(); ++it) { removeOrDeleteCollision<false>(solver, it); }
  }

  void clear()
  {
    if(!solver)
    {
      data_.clear();
      return;
    }
    auto it = data_.begin();
    while(it != data_.end()) { it = removeOrDeleteCollision<true>(*solver, it); }
  }

  CollisionData & createCollision(TVMQPSolver & solver,
                                  const mc_rbdyn::Robot & r1,
                                  const mc_rbdyn::Robot & r2,
                                  const mc_rbdyn::Collision & col,
                                  int id,
                                  const Eigen::VectorXd & r1Selector,
                                  const Eigen::VectorXd & r2Selector)
  {
    data_.push_back({id, col});
    auto & data = data_.back();
    auto & c1 = r1.tvmConvex(col.body1);
    auto & c2 = r2.tvmConvex(col.body2);
    data.function = std::make_shared<mc_tvm::CollisionFunction>(c1, c2, r1Selector, r2Selector, solver.dt());
    return data;
  }

  void addCollision(TVMQPSolver & solver, CollisionData & data)
  {
    const auto & col = data.collision;
    data.task = solver.problem().add(
        data.function >= 0.,
        tvm::task_dynamics::VelocityDamper(
            solver.dt(), {col.iDist, col.sDist, col.damping, mc_solver::CollisionsConstraint::defaultDampingOffset},
            tvm::constant::big_number),
        {tvm::requirements::PriorityLevel(0)});
  }
};

} // namespace details

/** Helper to cast the constraint */
static inline mc_rtc::void_ptr_caster<tasks::qp::CollisionConstr> tasks_constraint{};
static inline mc_rtc::void_ptr_caster<details::TVMCollisionConstraint> tvm_constraint{};

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
  if(!match) { mc_rtc::log::error_and_throw("No match found for collision wildcard {} in {}", body, robot.name()); }
  return true;
}

static mc_rtc::void_ptr make_constraint(QPSolver::Backend backend, const mc_rbdyn::Robots & robots, double timeStep)
{
  switch(backend)
  {
    case QPSolver::Backend::Tasks:
      return mc_rtc::make_void_ptr<tasks::qp::CollisionConstr>(robots.mbs(), timeStep);
    case QPSolver::Backend::TVM:
      return mc_rtc::make_void_ptr<details::TVMCollisionConstraint>();
    default:
      mc_rtc::log::error_and_throw("[CollisionConstr] Not implemented for solver backend: {}", backend);
  }
}

CollisionsConstraint::CollisionsConstraint(const mc_rbdyn::Robots & robots,
                                           unsigned int r1Index,
                                           unsigned int r2Index,
                                           double timeStep)
: constraint_(make_constraint(backend_, robots, timeStep)), r1Index(r1Index), r2Index(r2Index), collId(0), collIdDict()
{
}

bool CollisionsConstraint::removeCollision(QPSolver & solver, const std::string & b1Name, const std::string & b2Name)
{
  const auto & robots = solver.robots();
  const mc_rbdyn::Robot & r1 = robots.robot(r1Index);
  const mc_rbdyn::Robot & r2 = robots.robot(r2Index);
  auto on_b1_wildcard = [&](const std::string & nb1) { removeCollision(solver, nb1, b2Name); };
  auto on_b2_wildcard = [&](const std::string & nb2) { removeCollision(solver, b1Name, nb2); };
  if(handle_wildcard(r1, b1Name, on_b1_wildcard) || handle_wildcard(r2, b2Name, on_b2_wildcard)) { return true; }
  auto p = __popCollId(b1Name, b2Name);
  if(!p.second.isNone())
  {
    if(monitored_.count(p.first)) { toggleCollisionMonitor(p.first, &p.second); }
    category_.push_back("Monitors");
    std::string name = "Monitor " + p.second.body1 + "/" + p.second.body2;
    gui_->removeElement(category_, name);
    category_.pop_back();
    cols.erase(std::find(cols.begin(), cols.end(), p.second));
    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto collConstr = tasks_constraint(constraint_);
        auto & qpsolver = tasks_solver(solver);
        bool ret = collConstr->rmCollision(p.first);
        if(ret)
        {
          collConstr->updateNrVars({}, qpsolver.data());
          qpsolver.updateConstrSize();
        }
        return ret;
      }
      case QPSolver::Backend::TVM:
        tvm_constraint(constraint_)->deleteCollision(tvm_solver(solver), p.second);
        break;
      default:
        break;
    }
  }
  return false;
}

void CollisionsConstraint::removeCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Collision> & cols)
{
  for(const auto & c : cols) { removeCollision(solver, c.body1, c.body2); }
}

bool CollisionsConstraint::removeCollisionByBody(QPSolver & solver,
                                                 const std::string & b1Name,
                                                 const std::string & b2Name)
{
  const auto & r1 = solver.robots().robot(r1Index);
  const auto & r2 = solver.robots().robot(r2Index);
  std::vector<mc_rbdyn::Collision> toRm;
  for(const auto & col : cols)
  {
    if(r1.convex(col.body1).first == b1Name && r2.convex(col.body2).first == b2Name)
    {
      auto out = __popCollId(col.body1, col.body2);
      toRm.push_back(out.second);
      switch(backend_)
      {
        case QPSolver::Backend::Tasks:
        {
          auto collConstr = tasks_constraint(constraint_);
          collConstr->rmCollision(out.first);
          break;
        }
        case QPSolver::Backend::TVM:
          tvm_constraint(constraint_)->deleteCollision(tvm_solver(solver), out.first);
          break;
        default:
          break;
      }
      if(monitored_.count(out.first)) { toggleCollisionMonitor(out.first, &out.second); }
      category_.push_back("Monitors");
      std::string name = "Monitor " + out.second.body1 + "/" + out.second.body2;
      gui_->removeElement(category_, name);
      category_.pop_back();
    }
  }
  for(const auto & it : toRm) { cols.erase(std::find(cols.begin(), cols.end(), it)); }
  if(toRm.size())
  {
    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto collConstr = tasks_constraint(constraint_);
        auto & qpsolver = tasks_solver(solver);
        collConstr->updateNrVars({}, qpsolver.data());
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

void CollisionsConstraint::__addCollision(mc_solver::QPSolver & solver, const mc_rbdyn::Collision & col)
{
  const auto & robots = solver.robots();
  const mc_rbdyn::Robot & r1 = robots.robot(r1Index);
  const mc_rbdyn::Robot & r2 = robots.robot(r2Index);
  if(col.body1.size() == 0 || col.body2.size() == 0)
  {
    mc_rtc::log::error("Attempted to add a collision without a specific body");
    return;
  }
  auto on_b1_wildcard = [&](const std::string & nb1)
  {
    auto nCol = col;
    nCol.body1 = nb1;
    __addCollision(solver, nCol);
  };
  auto on_b2_wildcard = [&](const std::string & nb2)
  {
    auto nCol = col;
    nCol.body2 = nb2;
    __addCollision(solver, nCol);
  };
  if(handle_wildcard(r1, col.body1, on_b1_wildcard) || handle_wildcard(r2, col.body2, on_b2_wildcard)) { return; }
  int collId = __createCollId(col);
  if(collId < 0) { return; }
  cols.push_back(col);

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
          mc_rtc::log::error_and_throw("[CollisionsConstraint] No joint named \"{}\" in robot \"{}\"", j,
                                       robots.robot(rIndex).name());
        }
      }
      if(inactive) { return jointsToSelector<false>(robots.robot(rIndex), *joints); }
      else { return jointsToSelector<true>(robots.robot(rIndex), *joints); }
    }
    else { return Eigen::VectorXd::Zero(0).eval(); }
  };

  auto r1Selector = computeJointsSelector(col.r1Joints, col.r1JointsInactive, r1Index);
  auto r2Selector = r1Index == r2Index ? Eigen::VectorXd::Zero(0).eval()
                                       : computeJointsSelector(col.r2Joints, col.r2JointsInactive, r2Index);

  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto collConstr = tasks_constraint(constraint_);
      const auto & body1 = r1.convex(col.body1);
      const auto & body2 = r2.convex(col.body2);
      const sva::PTransformd & X_b1_c = r1.collisionTransform(col.body1);
      const sva::PTransformd & X_b2_c = r2.collisionTransform(col.body2);
      if(r1.mb().nrDof() == 0)
      {
        collConstr->addCollision(robots.mbs(), collId, static_cast<int>(r2Index), body2.first, body2.second.get(),
                                 X_b2_c, static_cast<int>(r1Index), body1.first, body1.second.get(), X_b1_c, col.iDist,
                                 col.sDist, col.damping, defaultDampingOffset, r2Selector, r1Selector);
      }
      else
      {
        collConstr->addCollision(robots.mbs(), collId, static_cast<int>(r1Index), body1.first, body1.second.get(),
                                 X_b1_c, static_cast<int>(r2Index), body2.first, body2.second.get(), X_b2_c, col.iDist,
                                 col.sDist, col.damping, defaultDampingOffset, r1Selector, r2Selector);
      }
      break;
    }
    case QPSolver::Backend::TVM:
    {
      auto & data =
          tvm_constraint(constraint_)->createCollision(tvm_solver(solver), r1, r2, col, collId, r1Selector, r2Selector);
      if(inSolver_) { tvm_constraint(constraint_)->addCollision(tvm_solver(solver), data); }
      break;
    }
    default:
      break;
  }
  addMonitorButton(collId, col);
}

void CollisionsConstraint::addMonitorButton(int collId, const mc_rbdyn::Collision & col)
{
  if(gui_ && inSolver_)
  {
    auto & gui = *gui_;
    std::string name = col.body1 + "/" + col.body2;
    category_.push_back("Monitors");
    gui.addElement(category_, mc_rtc::gui::Checkbox(
                                  "Monitor " + name, [collId, this]() { return monitored_.count(collId) != 0; },
                                  [collId, this]() { toggleCollisionMonitor(collId); }));
    category_.pop_back();
  }
}

void CollisionsConstraint::toggleCollisionMonitor(int collId, const mc_rbdyn::Collision * col_p)
{
  auto findCollisionById = [this, collId, &col_p]()
  {
    if(col_p) { return; }
    for(const auto & c : collIdDict)
    {
      if(c.second.first == collId)
      {
        col_p = &c.second.second;
        return;
      }
    }
    mc_rtc::log::error_and_throw("[CollisionConstraint] Attempted to toggleCollisionMonitor on non-existent collision");
  };
  findCollisionById();
  const auto & col = *col_p;
  auto & gui = *gui_;
  std::string label = col.body1 + "::" + col.body2;
  if(monitored_.count(collId))
  {
    // Remove the monitor
    gui.removeElement(category_, label);
    category_.push_back("Arrows");
    gui.removeElement(category_, label);
    category_.pop_back();
    monitored_.erase(collId);
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
        auto collConstr = tasks_constraint(constraint_);
        addMonitor(
            [collConstr, collId]() { return collConstr->getCollisionData(collId).distance; },
            [collConstr, collId]() -> const Eigen::Vector3d & { return collConstr->getCollisionData(collId).p1; },
            [collConstr, collId]() -> const Eigen::Vector3d & { return collConstr->getCollisionData(collId).p2; });
        break;
      }
      case QPSolver::Backend::TVM:
      {
        auto collConstr = tvm_constraint(constraint_);
        auto fn = collConstr->getData(collId)->function;
        addMonitor([fn]() { return fn->distance(); }, [fn]() -> const Eigen::Vector3d & { return fn->p1(); },
                   [fn]() -> const Eigen::Vector3d & { return fn->p2(); });
        break;
      }
      default:
        break;
    }
    monitored_.insert(collId);
  }
}

void CollisionsConstraint::addCollision(QPSolver & solver, const mc_rbdyn::Collision & col)
{
  addCollisions(solver, {col});
}

void CollisionsConstraint::addCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Collision> & cols)
{
  for(const auto & c : cols) { __addCollision(solver, c); }
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & collConstr = *tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      collConstr.updateNrVars({}, qpsolver.data());
      qpsolver.updateConstrSize();
      break;
    }
    case QPSolver::Backend::TVM:
      break;
    default:
      break;
  }
}

void CollisionsConstraint::addToSolverImpl(QPSolver & solver)
{
  gui_ = solver.gui();
  const mc_rbdyn::Robot & r1 = solver.robots().robot(r1Index);
  const mc_rbdyn::Robot & r2 = solver.robots().robot(r2Index);
  category_ = {"Collisions", r1.name() + "/" + r2.name()};
  gui_->addElement(category_, mc_rtc::gui::Checkbox("Automatic monitor", autoMonitor_));
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & collConstr = *tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      collConstr.addToSolver(solver.robots().mbs(), qpsolver.solver());
      break;
    }
    case QPSolver::Backend::TVM:
    {
      auto cstr = tvm_constraint(constraint_);
      for(auto & c : cstr->data_) { tvm_constraint(constraint_)->addCollision(tvm_solver(solver), c); }
      tvm_constraint(constraint_)->solver = &tvm_solver(solver);
      break;
    }
    default:
      break;
  }
  for(const auto & cols : collIdDict) { addMonitorButton(cols.second.first, cols.second.second); }
}

void CollisionsConstraint::update(QPSolver &)
{
  if(!autoMonitor_) { return; }
  auto getDistance = [this](int collId)
  {
    switch(backend_)
    {
      case QPSolver::Backend::Tasks:
      {
        auto collConstr = tasks_constraint(constraint_);
        return collConstr->getCollisionData(collId).distance;
      }
      case QPSolver::Backend::TVM:
      {
        auto collConstr = tvm_constraint(constraint_);
        auto & fn = collConstr->getData(collId)->function;
        return fn->distance();
      }
      default:
        mc_rtc::log::error_and_throw("Not implemented for this backend");
    }
  };
  for(const auto & [name, info] : collIdDict)
  {
    const auto & [collId, coll] = info;
    auto distance = getDistance(collId);
    if(distance < coll.iDist && !monitored_.count(collId)) { toggleCollisionMonitor(collId, &coll); }
    if(distance > coll.iDist && monitored_.count(collId)) { toggleCollisionMonitor(collId, &coll); }
  }
}

void CollisionsConstraint::removeFromSolverImpl(QPSolver & solver)
{
  switch(backend_)
  {
    case QPSolver::Backend::Tasks:
    {
      auto & collConstr = *tasks_constraint(constraint_);
      auto & qpsolver = tasks_solver(solver);
      collConstr.removeFromSolver(qpsolver.solver());
      break;
    }
    case QPSolver::Backend::TVM:
    {
      tvm_constraint(constraint_)->removeCollisions(tvm_solver(solver));
      tvm_constraint(constraint_)->solver = nullptr;
      break;
    }
    default:
      break;
  }
  gui_->removeCategory(category_);
}

void CollisionsConstraint::reset()
{
  cols.clear();
  collIdDict.clear();
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

std::string CollisionsConstraint::__keyByNames(const std::string & name1, const std::string & name2)
{
  return name1 + name2;
}

int CollisionsConstraint::__createCollId(const mc_rbdyn::Collision & col)
{
  std::string key = __keyByNames(col.body1, col.body2);
  auto it = collIdDict.find(key);
  if(it != collIdDict.end()) { return -1; }
  int collId = this->collId;
  collIdDict[key] = std::pair<int, mc_rbdyn::Collision>(collId, col);
  this->collId += 1;
  return collId;
}

std::pair<int, mc_rbdyn::Collision> CollisionsConstraint::__popCollId(const std::string & name1,
                                                                      const std::string & name2)
{
  std::string key = __keyByNames(name1, name2);
  if(collIdDict.count(key))
  {
    std::pair<int, mc_rbdyn::Collision> p = collIdDict[key];
    collIdDict.erase(key);
    return p;
  }
  return std::pair<unsigned int, mc_rbdyn::Collision>(0, mc_rbdyn::Collision());
}

bool CollisionsConstraint::hasCollision(const std::string & c1, const std::string & c2) const noexcept
{
  auto it = std::find_if(cols.begin(), cols.end(), [&](const auto & c) { return c.body1 == c1 && c.body2 == c2; });
  return it != cols.end();
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintSetLoader::register_load_function(
    "collision",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
    {
      auto ret = std::make_shared<mc_solver::CollisionsConstraint>(
          solver.robots(), robotIndexFromConfig(config, solver.robots(), "collision", false, "r1Index", "r1", ""),
          robotIndexFromConfig(config, solver.robots(), "collision", false, "r2Index", "r2", ""), solver.dt());
      ret->automaticMonitor(config("automaticMonitor", true));
      if(ret->r1Index == ret->r2Index)
      {
        if(config("useCommon", false))
        {
          ret->addCollisions(solver, solver.robots().robotModule(ret->r1Index).commonSelfCollisions());
        }
        else if(config("useMinimal", false))
        {
          ret->addCollisions(solver, solver.robots().robotModule(ret->r1Index).minimalSelfCollisions());
        }
      }
      std::vector<mc_rbdyn::Collision> collisions = config("collisions", std::vector<mc_rbdyn::Collision>{});
      ret->addCollisions(solver, collisions);
      return ret;
    });
} // namespace
