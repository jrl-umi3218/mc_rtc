/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/configuration_io.h>
#include <mc_solver/CollisionsConstraint.h>
#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/QPSolver.h>

namespace mc_solver
{

CollisionsConstraint::CollisionsConstraint(const mc_rbdyn::Robots & robots,
                                           unsigned int r1Index,
                                           unsigned int r2Index,
                                           double timeStep)
: collConstr(new tasks::qp::CollisionConstr(robots.mbs(), timeStep)), r1Index(r1Index), r2Index(r2Index), collId(0),
  collIdDict()
{
}

bool CollisionsConstraint::removeCollision(QPSolver & solver, const std::string & b1Name, const std::string & b2Name)
{
  auto p = __popCollId(b1Name, b2Name);
  if(!p.second.isNone())
  {
    cols.erase(std::find(cols.begin(), cols.end(), p.second));
    bool ret = collConstr->rmCollision(p.first);
    if(ret)
    {
      collConstr->updateNrVars({}, solver.data());
      solver.updateConstrSize();
    }
    return ret;
  }
  return false;
}

void CollisionsConstraint::removeCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Collision> & cols)
{
  for(const auto & c : cols)
  {
    removeCollision(solver, c.body1, c.body2);
  }
}

bool CollisionsConstraint::removeCollisionByBody(QPSolver & solver,
                                                 const std::string & b1Name,
                                                 const std::string & b2Name)
{
  const mc_rbdyn::Robot & r1 = solver.robots().robot(r1Index);
  const mc_rbdyn::Robot & r2 = solver.robots().robot(r2Index);
  std::vector<mc_rbdyn::Collision> toRm;
  for(const auto & col : cols)
  {
    if(r1.convex(col.body1).first == b1Name && r2.convex(col.body2).first == b2Name)
    {
      auto out = __popCollId(col.body1, col.body2);
      toRm.push_back(out.second);
      collConstr->rmCollision(out.first);
    }
  }
  for(const auto & it : toRm)
  {
    cols.erase(std::find(cols.begin(), cols.end(), it));
  }
  if(toRm.size())
  {
    collConstr->updateNrVars({}, solver.data());
    solver.updateConstrSize();
  }
  return toRm.size() > 0;
}

void CollisionsConstraint::__addCollision(const mc_rbdyn::Robots & robots, const mc_rbdyn::Collision & col)
{
  const mc_rbdyn::Robot & r1 = robots.robot(r1Index);
  const mc_rbdyn::Robot & r2 = robots.robot(r2Index);
  const auto & body1 = r1.convex(col.body1);
  const auto & body2 = r2.convex(col.body2);
  const sva::PTransformd & X_b1_c = r1.collisionTransform(body1.first);
  const sva::PTransformd & X_b2_c = r2.collisionTransform(body2.first);
  int collId = __createCollId(col);
  cols.push_back(col);
  collConstr->addCollision(robots.mbs(), collId, static_cast<int>(r1Index), body1.first, body1.second.get(), X_b1_c,
                           static_cast<int>(r2Index), body2.first, body2.second.get(), X_b2_c, col.iDist, col.sDist,
                           col.damping, defaultDampingOffset);
}

void CollisionsConstraint::addCollision(QPSolver & solver, const mc_rbdyn::Collision & col)
{
  addCollisions(solver, {col});
}

void CollisionsConstraint::addCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Collision> & cols)
{
  for(const auto & c : cols)
  {
    __addCollision(solver.robots(), c);
  }
  collConstr->updateNrVars({}, solver.data());
  solver.updateConstrSize();
}

void CollisionsConstraint::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver)
{
  if(collConstr)
  {
    collConstr->addToSolver(mbs, solver);
  }
}

void CollisionsConstraint::removeFromSolver(tasks::qp::QPSolver & solver)
{
  if(collConstr)
  {
    collConstr->removeFromSolver(solver);
  }
}

void CollisionsConstraint::reset()
{
  cols.clear();
  collIdDict.clear();
  collConstr->reset();
}

std::string CollisionsConstraint::__keyByNames(const std::string & name1, const std::string & name2)
{
  return name1 + name2;
}

int CollisionsConstraint::__createCollId(const mc_rbdyn::Collision & col)
{
  std::string key = __keyByNames(col.body1, col.body2);
  int collId = this->collId;
  collIdDict[key] = std::pair<unsigned int, mc_rbdyn::Collision>(collId, col);
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

RobotEnvCollisionsConstraint::RobotEnvCollisionsConstraint(const mc_rbdyn::Robots & robots, double timeStep)
: selfCollConstrMng(robots, robots.robotIndex(), robots.robotIndex(), timeStep),
  envCollConstrMng(robots, robots.robotIndex(), robots.envIndex(), timeStep)
{
}

bool RobotEnvCollisionsConstraint::removeEnvCollision(QPSolver & solver,
                                                      const std::string & rBodyName,
                                                      const std::string & eBodyName)
{
  return envCollConstrMng.removeCollision(solver, rBodyName, eBodyName);
}

bool RobotEnvCollisionsConstraint::removeEnvCollisionByBody(QPSolver & solver,
                                                            const std::string & rBodyName,
                                                            const std::string & eBodyName)
{
  return envCollConstrMng.removeCollisionByBody(solver, rBodyName, eBodyName);
}

bool RobotEnvCollisionsConstraint::removeSelfCollision(QPSolver & solver,
                                                       const std::string & body1Name,
                                                       const std::string & body2Name)
{
  return selfCollConstrMng.removeCollision(solver, body1Name, body2Name);
}

void RobotEnvCollisionsConstraint::addEnvCollision(QPSolver & solver, const mc_rbdyn::Collision & col)
{
  envCollConstrMng.addCollision(solver, col);
}

void RobotEnvCollisionsConstraint::addSelfCollision(QPSolver & solver, const mc_rbdyn::Collision & col)
{
  selfCollConstrMng.addCollision(solver, col);
}

void RobotEnvCollisionsConstraint::setEnvCollisions(QPSolver & solver,
                                                    const std::vector<mc_rbdyn::Contact> & contacts,
                                                    const std::vector<mc_rbdyn::Collision> & cols)
{
  const mc_rbdyn::Robot & robot = solver.robots().robot();
  const std::vector<mc_rbdyn::Collision> & envCols = envCollConstrMng.cols;
  // Avoid reset to keep damping
  auto contactBodies = __bodiesFromContacts(robot, contacts);

  auto idFromName = [&robot](const std::string & name) { return robot.convex(name).first; };

  for(size_t i = 0; i < envCols.size(); ++i)
  {
    const mc_rbdyn::Collision & col = envCols[i];
    // Remove body that are not in the new collision list
    if(std::find(cols.begin(), cols.end(), col) == cols.end())
    {
      removeEnvCollision(solver, col.body1, col.body2);
      i -= 1;
    }
    // Remove body that are in contacts
    else if(contactBodies.find(idFromName(col.body1)) != contactBodies.end())
    {
      removeEnvCollision(solver, col.body1, col.body2);
      i -= 1;
    }
  }

  for(const mc_rbdyn::Collision & col : cols)
  {
    // New collision not in the old set and not in contactBodies
    if(std::find(envCols.begin(), envCols.end(), col) == envCols.end()
       && contactBodies.find(idFromName(col.body1)) == contactBodies.end())
    {
      addEnvCollision(solver, col);
    }
  }
}

void RobotEnvCollisionsConstraint::setSelfCollisions(QPSolver & solver,
                                                     const std::vector<mc_rbdyn::Contact> & contacts,
                                                     const std::vector<mc_rbdyn::Collision> & cols)
{
  const mc_rbdyn::Robot & robot = solver.robots().robot();
  const std::vector<mc_rbdyn::Collision> & selfCols = selfCollConstrMng.cols;
  // Avoid reset to keep damping
  auto contactBodies = __bodiesFromContacts(robot, contacts);

  auto idFromName = [&robot](const std::string & name) { return robot.convex(name).first; };

  for(size_t i = 0; i < selfCols.size(); ++i)
  {
    const mc_rbdyn::Collision & col = selfCols[i];
    // Remove body that are not in the new collision list
    if(std::find(cols.begin(), cols.end(), col) == cols.end())
    {
      removeSelfCollision(solver, col.body1, col.body2);
      i -= 1;
    }
    // Remove body that are in contacts
    else if(contactBodies.find(idFromName(col.body1)) != contactBodies.end()
            && contactBodies.find(idFromName(col.body2)) != contactBodies.end())
    {
      removeSelfCollision(solver, col.body1, col.body2);
      i -= 1;
    }
  }

  for(const mc_rbdyn::Collision & col : cols)
  {
    // New collision not in the old set and not in contactBodies
    if(std::find(selfCols.begin(), selfCols.end(), col) == selfCols.end()
       && contactBodies.find(idFromName(col.body1)) == contactBodies.end()
       && contactBodies.find(idFromName(col.body2)) == contactBodies.end())
    {
      addSelfCollision(solver, col);
    }
  }
}

void RobotEnvCollisionsConstraint::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver)
{
  selfCollConstrMng.addToSolver(mbs, solver);
  envCollConstrMng.addToSolver(mbs, solver);
}

void RobotEnvCollisionsConstraint::removeFromSolver(tasks::qp::QPSolver & solver)
{
  selfCollConstrMng.removeFromSolver(solver);
  envCollConstrMng.removeFromSolver(solver);
}

std::set<std::string> RobotEnvCollisionsConstraint::__bodiesFromContacts(const mc_rbdyn::Robot & robot,
                                                                         const std::vector<mc_rbdyn::Contact> & contacts)
{
  std::set<std::string> res;
  for(const auto & c : contacts)
  {
    const std::string & s = c.r1Surface()->name();
    res.insert(robot.surface(s).bodyName());
  }
  return res;
}

} // namespace mc_solver

namespace
{

static auto registered = mc_solver::ConstraintSetLoader::register_load_function(
    "collision",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto ret = std::make_shared<mc_solver::CollisionsConstraint>(
          solver.robots(), robotIndexFromConfig(config, solver.robots(), "collision", false, "r1Index", "r1", ""),
          robotIndexFromConfig(config, solver.robots(), "collision", false, "r2Index", "r2", ""), solver.dt());
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
}
