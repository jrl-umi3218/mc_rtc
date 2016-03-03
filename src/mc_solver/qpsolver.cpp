#include <mc_solver/qpsolver.h>

#include <Tasks/Bounds.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <mc_solver/contact_util.h>

#include <mc_rbdyn/stance.h>
#include <mc_rbdyn/Surface.h>

#include <mc_rtc/logging.h>

namespace mc_solver
{

bool operator==(const qpcallback_t & lhs, const qpcallback_t & rhs)
{
  return lhs.first == rhs.first;
}

bool operator!=(const qpcallback_t & lhs, const qpcallback_t & rhs)
{
  return !(lhs == rhs);
}

ContactConstraint::ContactConstraint(double timeStep, ContactType contactType, bool dynamics)
{
  if(contactType == Acceleration)
  {
    contactConstr.reset(new tasks::qp::ContactAccConstr());
  }
  else if(contactType == Velocity)
  {
    contactConstr.reset(new tasks::qp::ContactSpeedConstr(timeStep));
  }
  else if(contactType == Position)
  {
    contactConstr.reset(new tasks::qp::ContactPosConstr(timeStep));
  }
  else
  {
    LOG_ERROR("Trying to create a contact constraint from an unknown contact constraint type")
    throw(std::string("bad constraint type"));
  }
  if(dynamics)
  {
    posLambdaConstr.reset(new tasks::qp::PositiveLambda());
  }
}

void ContactConstraint::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const
{
  if(contactConstr)
  {
    contactConstr->addToSolver(mbs, solver);
  }
  if(posLambdaConstr)
  {
    posLambdaConstr->addToSolver(mbs, solver);
  }
}

void ContactConstraint::removeFromSolver(tasks::qp::QPSolver & solver) const
{
  if(contactConstr)
  {
    contactConstr->removeFromSolver(solver);
  }
  if(posLambdaConstr)
  {
    posLambdaConstr->removeFromSolver(solver);
  }
}

KinematicsConstraint::KinematicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep, bool isStatic,
                                           const std::vector<double> & damper, double velocityPercent)
: damped(damper.size() >= 3)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);

  if(isStatic)
  {
    /* FIXME Python code does self.constraints.append(self.contactConstr)
             it does not make sense here so we will do nothing for now... */
  }
  else
  {
    tasks::QBound qBound(robot.ql(), robot.qu());
    if(damped)
    {
      double percentInter = damper[0];
      double percentSecur = damper[1];
      double offset = damper[2];

      std::vector< std::vector<double> > vl = robot.vl();
      std::vector< std::vector<double> > vu = robot.vu();
      for(auto & vi : vl)
      {
        for(auto & v : vi)
        {
          v = v*velocityPercent;
        }
      }
      for(auto & vi : vu)
      {
        for(auto & v : vi)
        {
          v = v*velocityPercent;
        }
      }
      tasks::AlphaBound alphaBound(vl, vu);

      damperJointLimitsConstr.reset(new tasks::qp::DamperJointLimitsConstr(robots.mbs(), static_cast<int>(robotIndex), qBound, alphaBound, percentInter, percentSecur, offset, timeStep));
    }
    else
    {
      jointLimitsConstr.reset(new tasks::qp::JointLimitsConstr(robots.mbs(), static_cast<int>(robotIndex), qBound, timeStep));
    }
  }
}

void KinematicsConstraint::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const
{
  if(damperJointLimitsConstr)
  {
    damperJointLimitsConstr->addToSolver(mbs, solver);
  }
  if(jointLimitsConstr)
  {
    jointLimitsConstr->addToSolver(mbs, solver);
  }
}

void KinematicsConstraint::removeFromSolver(tasks::qp::QPSolver & solver) const
{
  if(damperJointLimitsConstr)
  {
    damperJointLimitsConstr->removeFromSolver(solver);
  }
  if(jointLimitsConstr)
  {
    jointLimitsConstr->removeFromSolver(solver);
  }
}

DynamicsConstraint::DynamicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep, bool isStatic,
                                       const std::vector<double> & damper, double velocityPercent, bool infTorque)
: KinematicsConstraint(robots, robotIndex, timeStep, isStatic, damper, velocityPercent),
  is_spring(false), is_poly(false)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  std::vector< std::vector<double> > tl = robot.tl();
  std::vector< std::vector<double> > tu = robot.tu();
  if(infTorque)
  {
    for(auto & ti : tl)
    {
      for(auto & t : ti)
      {
        t = -INFINITY;
      }
    }
    for(auto & ti : tu)
    {
      for(auto & t : ti)
      {
        t = INFINITY;
      }
    }
  }
  tasks::TorqueBound tBound(tl, tu);
  if(robot.flexibility().size() != 0)
  {
    is_spring = true;
    std::vector<tasks::qp::SpringJoint> sjList;
    for(const auto & flex : robot.flexibility())
    {
      sjList.push_back(tasks::qp::SpringJoint(robot.jointIdByName(flex.jointName), flex.K, flex.C, flex.O));
    }
    motionSpringConstr.reset(new tasks::qp::MotionSpringConstr(robots.mbs(), static_cast<int>(robotIndex), tBound, sjList));
  }
  /*FIXME Implement?
  else if(robot.tlPoly.size() != 0)
  {
    is_poly = true;
  } */
  else
  {
    motionConstr.reset(new tasks::qp::MotionConstr(robots.mbs(), static_cast<int>(robotIndex), tBound));
  }
}

void DynamicsConstraint::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const
{
  KinematicsConstraint::addToSolver(mbs, solver);
  if(motionConstr)
  {
    motionConstr->addToSolver(mbs, solver);
  }
  if(motionSpringConstr)
  {
    motionSpringConstr->addToSolver(mbs, solver);
  }
}

void DynamicsConstraint::removeFromSolver(tasks::qp::QPSolver & solver) const
{
  KinematicsConstraint::removeFromSolver(solver);
  if(motionConstr)
  {
    motionConstr->removeFromSolver(solver);
  }
  if(motionSpringConstr)
  {
    motionSpringConstr->removeFromSolver(solver);
  }
}

CollisionsConstraint::CollisionsConstraint(const mc_rbdyn::Robots & robots, unsigned int r1Index, unsigned int r2Index, double timeStep)
: collConstr(new tasks::qp::CollisionConstr(robots.mbs(), timeStep)),
  r1Index(r1Index), r2Index(r2Index), collId(0), collIdDict()
{
}

bool CollisionsConstraint::removeCollision(QPSolver & solver, const std::string & b1Name, const std::string & b2Name)
{
  auto p = __popCollId(b1Name, b2Name);
  if(!p.second.isNone())
  {
    cols.erase(std::find(cols.begin(), cols.end(), p.second));
    bool ret = collConstr->rmCollision(p.first);
    if(collConstr->nrInEq())
    {
      collConstr->updateNrCollisions();
      solver.updateConstrSize();
    }
    return ret;
  }
  return false;
}

bool CollisionsConstraint::removeCollisionByBody(QPSolver & solver, const std::string & b1Name, const std::string & b2Name)
{
  const mc_rbdyn::Robot & r1 = solver.robots().robot(r1Index);
  const mc_rbdyn::Robot & r2 = solver.robots().robot(r2Index);
  int b1Id = r1.bodyIdByName(b1Name);
  int b2Id = r2.bodyIdByName(b2Name);
  std::vector<mc_rbdyn::Collision> toRm;
  for(const auto & col : cols)
  {
    if(r1.convex(col.body1).first == b1Id &&
       r2.convex(col.body2).first == b2Id)
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
  if(toRm.size() && collConstr->nrInEq())
  {
    collConstr->updateNrCollisions();
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
  collConstr->addCollision(robots.mbs(), collId, static_cast<int>(r1Index), body1.first, const_cast<sch::S_Polyhedron*>(body1.second.get()), X_b1_c, static_cast<int>(r2Index), body2.first, const_cast<sch::S_Polyhedron*>(body2.second.get()), X_b2_c, col.iDist, col.sDist, col.damping, defaultDampingOffset);
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
  if(collConstr->nrInEq())
  {
    collConstr->updateNrCollisions();
    solver.updateConstrSize();
  }
}

void CollisionsConstraint::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const
{
  if(collConstr)
  {
    collConstr->addToSolver(mbs, solver);
  }
}

void CollisionsConstraint::removeFromSolver(tasks::qp::QPSolver & solver) const
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

std::pair<int, mc_rbdyn::Collision> CollisionsConstraint::__popCollId(const std::string & name1, const std::string & name2)
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

bool RobotEnvCollisionsConstraint::removeEnvCollision(QPSolver & solver, const std::string & rBodyName, const std::string & eBodyName)
{
  return envCollConstrMng.removeCollision(solver, rBodyName, eBodyName);
}

bool RobotEnvCollisionsConstraint::removeEnvCollisionByBody(QPSolver & solver, const std::string & rBodyName, const std::string & eBodyName)
{
  return envCollConstrMng.removeCollisionByBody(solver, rBodyName, eBodyName);
}

bool RobotEnvCollisionsConstraint::removeSelfCollision(QPSolver & solver, const std::string & body1Name, const std::string & body2Name)
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

void RobotEnvCollisionsConstraint::setEnvCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Contact> & contacts,
                                                    const std::vector<mc_rbdyn::Collision> & cols)
{
  const mc_rbdyn::Robot & robot = solver.robots().robot();
  const std::vector<mc_rbdyn::Collision> & envCols = envCollConstrMng.cols;
  // Avoid reset to keep damping
  auto contactBodies = __bodiesFromContacts(robot, contacts);

  auto idFromName = [&robot](const std::string & name){ return robot.convex(name).first; };

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
    if(std::find(envCols.begin(), envCols.end(), col) == envCols.end() &&
       contactBodies.find(idFromName(col.body1)) == contactBodies.end())
    {
      addEnvCollision(solver, col);
    }
  }
}

void RobotEnvCollisionsConstraint::setSelfCollisions(QPSolver & solver, const std::vector<mc_rbdyn::Contact> & contacts,
                                                     const std::vector<mc_rbdyn::Collision> & cols)
{
  const mc_rbdyn::Robot & robot = solver.robots().robot();
  const std::vector<mc_rbdyn::Collision> & selfCols = selfCollConstrMng.cols;
  // Avoid reset to keep damping
  auto contactBodies = __bodiesFromContacts(robot, contacts);

  auto idFromName = [&robot](const std::string & name){ return robot.convex(name).first; };

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
    if(std::find(selfCols.begin(), selfCols.end(), col) == selfCols.end() &&
       contactBodies.find(idFromName(col.body1)) == contactBodies.end() &&
       contactBodies.find(idFromName(col.body2)) == contactBodies.end())
    {
      addSelfCollision(solver, col);
    }
  }
}

void RobotEnvCollisionsConstraint::addToSolver(const std::vector<rbd::MultiBody> & mbs, tasks::qp::QPSolver & solver) const
{
  selfCollConstrMng.addToSolver(mbs, solver);
  envCollConstrMng.addToSolver(mbs, solver);
}

void RobotEnvCollisionsConstraint::removeFromSolver(tasks::qp::QPSolver & solver) const
{
  selfCollConstrMng.removeFromSolver(solver);
  envCollConstrMng.removeFromSolver(solver);
}

std::set<int> RobotEnvCollisionsConstraint::__bodiesFromContacts(const mc_rbdyn::Robot & robot, const std::vector<mc_rbdyn::Contact> & contacts)
{
  std::set<int> res;
  for(const auto & c : contacts)
  {
    const std::string & s = c.r1Surface()->name();
    res.insert(robot.bodyIdByName(robot.surface(s).bodyName()));
  }
  return res;
}

QPSolver::QPSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double timeStep)
: robots_p(robots), timeStep(timeStep), solver()
{
}

void QPSolver::addConstraintSet(const ConstraintSet & cs)
{
  cs.addToSolver(robots().mbs(), solver);
}

void QPSolver::removeConstraintSet(const ConstraintSet & cs)
{
  cs.removeFromSolver(solver);
}

void QPSolver::addTask(tasks::qp::Task * task)
{
  solver.addTask(robots().mbs(), task);
}

void QPSolver::removeTask(tasks::qp::Task * task)
{
  solver.removeTask(task);
}

std::pair<int, const tasks::qp::BilateralContact&> QPSolver::contactById(const tasks::qp::ContactId & id)
{
  const std::vector<tasks::qp::BilateralContact> & contacts = solver.data().allContacts();
  for(size_t i = 0; i < contacts.size(); ++i)
  {
    if(id == contacts[i].contactId)
    {
      return std::pair<int, const tasks::qp::BilateralContact&>(i, contacts[i]);
    }
  }
  // Of course this ref has no value here...
  return std::pair<int, const tasks::qp::BilateralContact&>(-1, tasks::qp::BilateralContact());
}

void QPSolver::setContacts(const std::vector<mc_rbdyn::Contact> & contacts)
{
  uniContacts.clear();
  biContacts.clear();
  qpRes.contacts.clear();

  for(const mc_rbdyn::Contact & c : contacts)
  {
    QPContactPtr qcptr = c.taskContact(*robots_p);
    if(qcptr.unilateralContact)
    {
      uniContacts.push_back(tasks::qp::UnilateralContact(*qcptr.unilateralContact));
      delete qcptr.unilateralContact;
      qcptr.unilateralContact = 0;
    }
    else
    {
      biContacts.push_back(tasks::qp::BilateralContact(*qcptr.bilateralContact));
      delete qcptr.bilateralContact;
      qcptr.bilateralContact = 0;
    }
  }

  solver.nrVars(robots_p->mbs(), uniContacts, biContacts);
  const tasks::qp::SolverData & data = solver.data();
  qpRes.contacts = contactsMsgFromContacts(*robots_p, contacts);
  qpRes.contacts_lambda_begin.clear();
  for(int i = 0; i < data.nrContacts(); ++i)
  {
    qpRes.contacts_lambda_begin.push_back(data.lambdaBegin(i) - data.lambdaBegin());
  }
  updateConstrSize();
}

bool QPSolver::run()
{
  bool success = false;
  if(solver.solveNoMbcUpdate(robots_p->mbs(), robots_p->mbcs()))
  {
    for(size_t i = 0; i < robots_p->mbs().size(); ++i)
    {
      rbd::MultiBody & mb = robots_p->mbs()[i];
      rbd::MultiBodyConfig & mbc = robots_p->mbcs()[i];
      if(mb.nrDof() > 0)
      {
        solver.updateMbc(mbc, static_cast<int>(i));
        rbd::eulerIntegration(mb, mbc, timeStep);
        rbd::forwardKinematics(mb, mbc);
        rbd::forwardVelocity(mb, mbc);
      }
      success = true;
    }
  }
  return success;
}

const mc_control::QPResultMsg & QPSolver::send(double/*curTime*/)
{
  __fillResult();
  /*FIXME Use curTime? */
  return qpRes;
}

void QPSolver::__fillResult()
{
  qpRes.robots_state.resize(0);
  for(unsigned int i = 0; i < robots_p->robots().size(); ++i)
  {
    const mc_rbdyn::Robot & robot = robots_p->robot(i);
    qpRes.robots_state.push_back(mc_control::RobotMsg());
    std::vector<double> & q = qpRes.robots_state[i].q;
    for(const auto & qv : robot.mbc().q)
    {
      for(const auto & qi : qv)
      {
        q.push_back(qi);
      }
    }
    std::vector<double> & alphaVec = qpRes.robots_state[i].alphaVec;
    for(const auto & av : robot.mbc().alpha)
    {
      for(const auto & ai : av)
      {
        alphaVec.push_back(ai);
      }
    }
    qpRes.robots_state[i].alphaDVec = solver.alphaDVec(static_cast<int>(i));
  }
  qpRes.lambdaVec = solver.lambdaVec();
}

const mc_rbdyn::Robot & QPSolver::robot() const
{
  return robots_p->robot();
}
mc_rbdyn::Robot & QPSolver::robot()
{
  return robots_p->robot();
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

void QPSolver::updateConstrSize()
{
  solver.updateConstrSize();
}

}
