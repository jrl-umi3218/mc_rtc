#include <mc_control/mc_solver/qpsolver.h>

#include <Tasks/Bounds.h>
#include <Tasks/QPConstr.h>
#include <Tasks/QPContactConstr.h>
#include <Tasks/QPMotionConstr.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <mc_control/mc_solver/contact_util.h>

#include <mc_rbdyn/stance.h>

namespace mc_solver
{

ContactConstraint::ContactConstraint(double timeStep, ContactType contactType, bool dynamics)
{
  if(contactType == Acceleration)
  {
    contactConstr = std::shared_ptr<tasks::qp::Constraint>(new tasks::qp::ContactAccConstr());
  }
  else if(contactType == Velocity)
  {
    contactConstr = std::shared_ptr<tasks::qp::Constraint>(new tasks::qp::ContactSpeedConstr(timeStep));
  }
  else if(contactType == Position)
  {
    contactConstr = std::shared_ptr<tasks::qp::Constraint>(new tasks::qp::ContactPosConstr(timeStep));
  }
  else
  {
    std::cerr << "Trying to create a contact constraint from an unknown contact constraint type" << std::endl;
    throw(std::string("bad constraint type"));
  }
  constraints.push_back(contactConstr);
  if(dynamics)
  {
    constraints.push_back(std::shared_ptr<tasks::qp::Constraint>(new tasks::qp::PositiveLambda()));
  }
}

KinematicsConstraint::KinematicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep, bool isStatic,
                                           std::vector<double> damper, double velocityPercent)
: damped(damper.size() >= 3)
{
  const mc_rbdyn::Robot & robot = robots.robots[robotIndex];

  if(isStatic)
  {
    /* FIXME Python code does self.constraints.append(self.contactConstr)
             it does not make sense here so we will do nothing for now... */
  }
  else
  {
    tasks::QBound qBound(robot.ql, robot.qu);
    if(damped)
    {
      double percentInter = damper[0];
      double percentSecur = damper[1];
      double offset = damper[2];

      std::vector< std::vector<double> > vl = robot.vl;
      std::vector< std::vector<double> > vu = robot.vu;
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

      jointLimitsConstr = std::shared_ptr<tasks::qp::Constraint>(new tasks::qp::DamperJointLimitsConstr(robots.mbs, robotIndex,
                                                                                                  qBound, alphaBound, percentInter,
                                                                                                  percentSecur, offset, timeStep));
      tasks::qp::DamperJointLimitsConstr * test = new tasks::qp::DamperJointLimitsConstr(robots.mbs, robotIndex,
                                                                                                  qBound, alphaBound, percentInter,
                                                                                                  percentSecur, offset, timeStep);
    }
    else
    {
      jointLimitsConstr = std::shared_ptr<tasks::qp::Constraint>(new tasks::qp::JointLimitsConstr(robots.mbs, robotIndex, qBound, timeStep));
    }
    constraints.push_back(jointLimitsConstr);
  }
}

DynamicsConstraint::DynamicsConstraint(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double timeStep, bool isStatic,
                                       std::vector<double> damper, double velocityPercent, bool infTorque)
: KinematicsConstraint(robots, robotIndex, timeStep, isStatic, damper, velocityPercent),
  is_spring(false), is_poly(false)
{
  const mc_rbdyn::Robot & robot = robots.robots[robotIndex];
  std::vector< std::vector<double> > tl = robot.tl;
  std::vector< std::vector<double> > tu = robot.tu;
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
  if(robot.flexibility.size() != 0)
  {
    is_spring = true;
    std::vector<tasks::qp::SpringJoint> sjList;
    for(const auto & flex : robot.flexibility)
    {
      sjList.push_back(tasks::qp::SpringJoint(robot.jointIdByName(flex.jointName), flex.K, flex.C, flex.O));
    }
    motionConstr = std::shared_ptr<tasks::qp::Constraint>(new tasks::qp::MotionSpringConstr(robots.mbs, robotIndex, tBound, sjList));
  }
  else if(robot.tlPoly.size() != 0)
  {
    is_poly = true;
    /*FIXME Implement? */
  }
  else
  {
    motionConstr = std::shared_ptr<tasks::qp::Constraint>(new tasks::qp::MotionConstr(robots.mbs, robotIndex, tBound));
  }
  constraints.push_back(motionConstr);
}

double CollisionsConstraint::defaultDampingOffset = 0.1;

CollisionsConstraint::CollisionsConstraint(const mc_rbdyn::Robots & robots, unsigned int r1Index, unsigned int r2Index, double timeStep)
: collConstr(new tasks::qp::CollisionConstr(robots.mbs, timeStep)),
  r1Index(r1Index), r2Index(r2Index), collId(0), collIdDict()
{
  constraints.push_back(collConstr);
}

bool CollisionsConstraint::removeCollision(const mc_rbdyn::Robots &/*robots*/, const std::string & b1Name, const std::string & b2Name)
{
  auto p = __popCollId(b1Name, b2Name);
  if(not p.second.isNone())
  {
    cols.erase(std::find(cols.begin(), cols.end(), p.second));
    return (dynamic_cast<tasks::qp::CollisionConstr*>(collConstr.get()))->rmCollision(p.first);
  }
  return false;
}

bool CollisionsConstraint::removeCollisionByBody(const mc_rbdyn::Robots & robots, const std::string & b1Name, const std::string & b2Name)
{
  const mc_rbdyn::Robot & r1 = robots.robots[r1Index];
  const mc_rbdyn::Robot & r2 = robots.robots[r2Index];
  unsigned int b1Id = r1.bodyIdByName(b1Name);
  unsigned int b2Id = r2.bodyIdByName(b2Name);
  std::vector<Collision> toRm;
  for(const auto & col : cols)
  {
    if(r1.convex.at(col.body1).first == b1Id and
       r2.convex.at(col.body2).first == b2Id)
    {
      auto out = __popCollId(col.body1, col.body2);
      toRm.push_back(out.second);
      dynamic_cast<tasks::qp::CollisionConstr*>(collConstr.get())->rmCollision(out.first);
    }
  }
  for(const auto & it : toRm)
  {
    cols.erase(std::find(cols.begin(), cols.end(), it));
  }
  return toRm.size() > 0;
}

void CollisionsConstraint::addCollision(const mc_rbdyn::Robots & robots, const Collision & col)
{
  const mc_rbdyn::Robot & r1 = robots.robots[r1Index];
  const mc_rbdyn::Robot & r2 = robots.robots[r2Index];
  cols.push_back(col);
  const auto & body1 = r1.convex.at(col.body1);
  const auto & body2 = r2.convex.at(col.body2);
  const sva::PTransformd & X_b1_c = r1.collisionTransforms.at(body1.first);
  const sva::PTransformd & X_b2_c = r2.collisionTransforms.at(body2.first);
  unsigned int collId = __createCollId(col);
  dynamic_cast<tasks::qp::CollisionConstr*>(collConstr.get())->addCollision(robots.mbs, collId, r1Index, body1.first, const_cast<sch::S_Polyhedron*>(&body1.second), X_b1_c, r2Index, body2.first, const_cast<sch::S_Polyhedron*>(&body2.second), X_b2_c, col.iDist, col.sDist, col.damping, defaultDampingOffset);
}

void CollisionsConstraint::addCollisions(const mc_rbdyn::Robots & robots, const std::vector<Collision> & cols)
{
  for(const auto & c : cols)
  {
    addCollision(robots, c);
  }
}

std::string CollisionsConstraint::__keyByNames(const std::string & name1, const std::string & name2)
{
  return name1 + name2;
}

unsigned int CollisionsConstraint::__createCollId(const Collision & col)
{
  std::string key = __keyByNames(col.body1, col.body2);
  unsigned int collId = this->collId;
  collIdDict[key] = std::pair<unsigned int, Collision>(collId, col);
  this->collId += 1;
  return collId;
}

std::pair<unsigned int, Collision> CollisionsConstraint::__popCollId(const std::string & name1, const std::string & name2)
{
  std::string key = __keyByNames(name1, name2);
  if(collIdDict.count(key))
  {
    std::pair<unsigned int, Collision> p = collIdDict[key];
    collIdDict.erase(key);
    return p;
  }
  return std::pair<unsigned int, Collision>(0,Collision());
}

RobotEnvCollisionsConstraint::RobotEnvCollisionsConstraint(const mc_rbdyn::Robots & robots, double timeStep)
: robot(robots.robot()), env(robots.env()),
  selfCollConstrMng(robots, robots.robotIndex, robots.robotIndex, timeStep),
  envCollConstrMng(robots, robots.robotIndex, robots.envIndex, timeStep)
{
  constraints.push_back(selfCollConstrMng.collConstr);
  constraints.push_back(envCollConstrMng.collConstr);
}

bool RobotEnvCollisionsConstraint::removeEnvCollision(const mc_rbdyn::Robots & robots, const std::string & rBodyName, const std::string & eBodyName)
{
  return envCollConstrMng.removeCollision(robots, rBodyName, eBodyName);
}

bool RobotEnvCollisionsConstraint::removeEnvCollisionByBody(const mc_rbdyn::Robots & robots, const std::string & rBodyName, const std::string & eBodyName)
{
  return envCollConstrMng.removeCollisionByBody(robots, rBodyName, eBodyName);
}

bool RobotEnvCollisionsConstraint::removeSelfCollision(const mc_rbdyn::Robots & robots, const std::string & body1Name, const std::string & body2Name)
{
  return selfCollConstrMng.removeCollision(robots, body1Name, body2Name);
}

void RobotEnvCollisionsConstraint::addEnvCollision(const mc_rbdyn::Robots & robots, const Collision & col)
{
  envCollConstrMng.addCollision(robots, col);
}

void RobotEnvCollisionsConstraint::addSelfCollision(const mc_rbdyn::Robots & robots, const Collision & col)
{
  selfCollConstrMng.addCollision(robots, col);
}

void RobotEnvCollisionsConstraint::setEnvCollisions(const mc_rbdyn::Robots & robots, const std::vector<mc_rbdyn::Contact> & contacts,
                                                    const std::vector<Collision> & cols)
{
  const mc_rbdyn::Robot & robot = robots.robot();
  const std::vector<Collision> & envCols = envCollConstrMng.cols;
  // Avoid reset to keep damping
  auto contactBodies = __bodiesFromContacts(robot, contacts);

  auto idFromName = [robot](const std::string & name){ return robot.convex.at(name).first; };

  for(size_t i = 0; i < envCols.size(); ++i)
  {
    const Collision & col = envCols[i];
    // Remove body that are not in the new collision list
    if(std::find(cols.begin(), cols.end(), col) == cols.end())
    {
      removeEnvCollision(robots, col.body1, col.body2);
      i -= 1;
    }
    // Remove body that are in contacts
    else if(contactBodies.find(idFromName(col.body1)) != contactBodies.end())
    {
      removeEnvCollision(robots, col.body1, col.body2);
      i -= 1;
    }
  }

  for(const Collision & col : cols)
  {
    // New collision not in the old set and not in contactBodies
    if(std::find(envCols.begin(), envCols.end(), col) == envCols.end() and
       contactBodies.find(idFromName(col.body1)) == contactBodies.end())
    {
      addEnvCollision(robots, col);
    }
  }
}

void RobotEnvCollisionsConstraint::setSelfCollisions(const mc_rbdyn::Robots & robots, const std::vector<mc_rbdyn::Contact> & contacts,
                                                     const std::vector<Collision> & cols)
{
  const mc_rbdyn::Robot & robot = robots.robot();
  const std::vector<Collision> & selfCols = selfCollConstrMng.cols;
  // Avoid reset to keep damping
  auto contactBodies = __bodiesFromContacts(robot, contacts);

  auto idFromName = [robot](const std::string & name){ return robot.convex.at(name).first; };

  for(size_t i = 0; i < selfCols.size(); ++i)
  {
    const Collision & col = selfCols[i];
    // Remove body that are not in the new collision list
    if(std::find(cols.begin(), cols.end(), col) == cols.end())
    {
      removeSelfCollision(robots, col.body1, col.body2);
      i -= 1;
    }
    // Remove body that are in contacts
    else if(contactBodies.find(idFromName(col.body1)) != contactBodies.end()
        and contactBodies.find(idFromName(col.body2)) != contactBodies.end())
    {
      removeSelfCollision(robots, col.body1, col.body2);
      i -= 1;
    }
  }

  for(const Collision & col : cols)
  {
    // New collision not in the old set and not in contactBodies
    if(std::find(selfCols.begin(), selfCols.end(), col) == selfCols.end() and
       contactBodies.find(idFromName(col.body1)) == contactBodies.end() and
       contactBodies.find(idFromName(col.body2)) == contactBodies.end())
    {
      addSelfCollision(robots, col);
    }
  }
}

std::set<unsigned int> RobotEnvCollisionsConstraint::__bodiesFromContacts(const mc_rbdyn::Robot & robot, const std::vector<mc_rbdyn::Contact> & contacts)
{
  std::set<unsigned int> res;
  for(const auto & c : contacts)
  {
    const std::string & s = c.robotSurface->name;
    res.insert(robot.bodyIdByName(robot.surfaces.at(s)->bodyName));
  }
  return res;
}

QPSolver::QPSolver(const mc_rbdyn::Robots & robots, double timeStep)
: robots(robots), timeStep(timeStep), solver()
{
}

void QPSolver::addConstraintSet(ConstraintSet & cs)
{
  for(std::shared_ptr<tasks::qp::Constraint> & cptr : cs.constraints)
  {
    if(auto c = dynamic_cast<tasks::qp::PositiveLambda*>(cptr.get())) { c->addToSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::MotionConstrCommon*>(cptr.get())) { c->addToSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::ContactConstr*>(cptr.get())) { c->addToSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::JointLimitsConstr*>(cptr.get())) { c->addToSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::DamperJointLimitsConstr*>(cptr.get())) { c->addToSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::CollisionConstr*>(cptr.get())) { c->addToSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::CoMIncPlaneConstr*>(cptr.get())) { c->addToSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::GripperTorqueConstr*>(cptr.get())) { c->addToSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::BoundedSpeedConstr*>(cptr.get())) { c->addToSolver(solver); }
    else
    {
      std::cerr << "Tried to add a constraint but its type is not recognized..." << std::endl;
    }
  }
  for(auto & precb : cs.preQPCb)
  {
    preQPCb.push_back(precb);
  }
  for(auto & postcb : cs.postQPCb)
  {
    postQPCb.push_back(postcb);
  }
}

void QPSolver::removeConstraintSet(ConstraintSet & cs)
{
  for(std::shared_ptr<tasks::qp::Constraint> & cptr : cs.constraints)
  {
    if(auto c = dynamic_cast<tasks::qp::PositiveLambda*>(cptr.get())) { c->removeFromSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::MotionConstrCommon*>(cptr.get())) { c->removeFromSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::ContactConstr*>(cptr.get())) { c->removeFromSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::JointLimitsConstr*>(cptr.get())) { c->removeFromSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::DamperJointLimitsConstr*>(cptr.get())) { c->removeFromSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::CollisionConstr*>(cptr.get())) { c->removeFromSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::CoMIncPlaneConstr*>(cptr.get())) { c->removeFromSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::GripperTorqueConstr*>(cptr.get())) { c->removeFromSolver(solver); }
    else if(auto c = dynamic_cast<tasks::qp::BoundedSpeedConstr*>(cptr.get())) { c->removeFromSolver(solver); }
    else
    {
      std::cerr << "Tried to remove a constraint but its type is not recognized..." << std::endl;
    }
  }
  for(auto & precb : cs.preQPCb)
  {
    for(std::vector<qpcallback_t>::iterator it = preQPCb.begin();
        it != preQPCb.end(); ++it)
    {
      if(it->first == precb.first)
      {
        preQPCb.erase(it);
        break;
      }
    }
  }
  for(auto & postcb : cs.postQPCb)
  {
    for(std::vector<qpcallback_t>::iterator it = postQPCb.begin();
        it != postQPCb.end(); ++it)
    {
      if(it->first == postcb.first)
      {
        postQPCb.erase(it);
        break;
      }
    }
  }
}

void QPSolver::updateNrVars()
{
  solver.nrVars(robots.mbs, uniContacts, biContacts);
}

void QPSolver::setContacts(const std::vector<mc_rbdyn::Contact> & contacts, bool useFkPos)
{
  uniContacts.clear();
  biContacts.clear();
  qpRes.contacts.clear();

  for(const mc_rbdyn::Contact & c : contacts)
  {
    sva::PTransformd * X_es_rs = 0;
    if(useFkPos)
    {
      X_es_rs = new sva::PTransformd(c.compute_X_es_rs(robots.robot(), robots.env()));
    }
    std::pair<mc_solver::QPContactPtr, std::vector<sva::PTransformd> > tasksC = tasksContactFromMcContact(robots, c, X_es_rs);
    mc_control::ContactMsg msg;
    msg.robot_surf = c.robotSurface->name;
    msg.env_surf = c.envSurface->name;
    msg.robot_surf_points = tasksC.second;
    msg.nr_generators = static_cast<uint16_t>(mc_rbdyn::Stance::nrConeGen);
    msg.mu = mc_rbdyn::Stance::defaultFriction;
    if(tasksC.first.unilateralContact)
    {
      uniContacts.push_back(*tasksC.first.unilateralContact);
      delete tasksC.first.unilateralContact;
      tasksC.first.unilateralContact = 0;
    }
    else
    {
      biContacts.push_back(*tasksC.first.bilateralContact);
      delete tasksC.first.bilateralContact;
      tasksC.first.bilateralContact = 0;
    }
    qpRes.contacts.push_back(msg);
    delete X_es_rs;
  }
  solver.nrVars(robots.mbs, uniContacts, biContacts);
}

std::vector<double> QPSolver::unilateralContactForce()
{
  std::vector<double> forces;
  Eigen::VectorXd lambda_ = solver.lambdaVec();
  unsigned int lIndex = 0;
  for(const tasks::qp::UnilateralContact & uc : uniContacts)
  {
    unsigned int newLindex = lIndex + uc.nrLambda();
    Eigen::Vector3d f = uc.force(lambda_.block(lIndex,0,newLindex,1), uc.r1Cone);
    lIndex = newLindex;
    forces.push_back(f(0));
    forces.push_back(f(1));
    forces.push_back(f(2));
  }
  return forces;
}

void QPSolver::update()
{
  solver.updateConstrSize();
}

bool QPSolver::run()
{
  mc_rbdyn::Robot & robot = robots.robot();
  for(auto & pcb : preQPCb)
  {
    pcb.second(*(robot.mb), *(robot.mbc), solver);
  }
  bool ret = false;
  if(solver.solveNoMbcUpdate(robots.mbs, robots.mbcs))
  {
    solver.updateMbc(*(robot.mbc), robots.robotIndex);
    rbd::eulerIntegration(*(robot.mb), *(robot.mbc), timeStep);
    rbd::forwardKinematics(*(robot.mb), *(robot.mbc));
    rbd::forwardVelocity(*(robot.mb), *(robot.mbc));
    ret = true;
  }
  for(auto & pcb : postQPCb)
  {
    pcb.second(*(robot.mb), *(robot.mbc), solver);
  }
  return ret;
}

const mc_control::QPResultMsg & QPSolver::send(double/*curTime*/, unsigned int stanceIndex)
{
  __fillResult();
  /*FIXME Maybe we need time in header*/
  qpRes.stance = static_cast<uint16_t>(stanceIndex);
  return qpRes;
}

void QPSolver::__fillResult()
{
  const mc_rbdyn::Robot & robot = robots.robot();
  const std::vector< std::vector<double> > & q = robot.mbc->q;
  const std::vector< std::vector<double> > & torque = robot.mbc->jointTorque;
  qpRes.q.clear();
  for(const auto & qi : q)
  {
    for(const auto & qv : qi)
    {
      qpRes.q.push_back(qv);
    }
  }
  qpRes.alphaDVec = solver.alphaDVec(robots.robotIndex);
  qpRes.lambdaVec = solver.lambdaVec();
  qpRes.torqueVec.clear();
  for(size_t i = 1; i < torque.size(); ++i)
  {
    for(const auto & tv : torque[i])
    {
      qpRes.torqueVec.push_back(tv);
    }
  }
}

MRQPSolver::MRQPSolver(const mc_rbdyn::Robots & robots, double timeStep)
: robots(robots), timeStep(timeStep), solver()
{
}

void MRQPSolver::addConstraintSet(const ConstraintSet & cs)
{
  for(const std::shared_ptr<tasks::qp::Constraint> & cptr : cs.constraints)
  {
    if(auto c = dynamic_cast<tasks::qp::ConstraintFunction<tasks::qp::Bound>*>(cptr.get()))
    {
      c->addToSolver(solver);
    }
    if(auto c = dynamic_cast<tasks::qp::ConstraintFunction<tasks::qp::Equality>*>(cptr.get()))
    {
      c->addToSolver(solver);
    }
    if(auto c = dynamic_cast<tasks::qp::ConstraintFunction<tasks::qp::GenInequality>*>(cptr.get()))
    {
      c->addToSolver(solver);
    }
    if(auto c = dynamic_cast<tasks::qp::ConstraintFunction<tasks::qp::Inequality>*>(cptr.get()))
    {
      c->addToSolver(solver);
    }
  }
}

void MRQPSolver::removeConstraintSet(const ConstraintSet & cs)
{
  for(const std::shared_ptr<tasks::qp::Constraint> & cptr : cs.constraints)
  {
    if(auto c = dynamic_cast<tasks::qp::ConstraintFunction<tasks::qp::Bound>*>(cptr.get()))
    {
      c->removeFromSolver(solver);
    }
    if(auto c = dynamic_cast<tasks::qp::ConstraintFunction<tasks::qp::Equality>*>(cptr.get()))
    {
      c->removeFromSolver(solver);
    }
    if(auto c = dynamic_cast<tasks::qp::ConstraintFunction<tasks::qp::GenInequality>*>(cptr.get()))
    {
      c->removeFromSolver(solver);
    }
    if(auto c = dynamic_cast<tasks::qp::ConstraintFunction<tasks::qp::Inequality>*>(cptr.get()))
    {
      c->removeFromSolver(solver);
    }
  }
}

void MRQPSolver::updateNrVars()
{
  solver.nrVars(robots.mbs, uniContacts, biContacts);
}

std::pair<int, const tasks::qp::BilateralContact&> MRQPSolver::contactById(const tasks::qp::ContactId & id)
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

void MRQPSolver::setContacts(const std::vector<mc_rbdyn::MRContact> & contacts)
{
  uniContacts.clear();
  biContacts.clear();
  qpRes.contacts.clear();

  for(const mc_rbdyn::MRContact & c : contacts)
  {
    QPContactPtr qcptr = mrTasksContactFromMcContact(robots, c);
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

  solver.nrVars(robots.mbs, uniContacts, biContacts);
  const tasks::qp::SolverData & data = solver.data();
  qpRes.contacts = mrContactsMsgFromMrContacts(robots, contacts);
  qpRes.contacts_lambda_begin.clear();
  for(int i = 0; i < data.nrContacts(); ++i)
  {
    qpRes.contacts_lambda_begin.push_back(data.lambdaBegin(i) - data.lambdaBegin());
  }
}

void MRQPSolver::update()
{
  solver.updateConstrSize();
}

bool MRQPSolver::run()
{
  bool success = false;
  if(solver.solveNoMbcUpdate(robots.mbs, robots.mbcs))
  {
    for(size_t i = 0; i < robots.mbs.size(); ++i)
    {
      rbd::MultiBody & mb = robots.mbs[i];
      rbd::MultiBodyConfig & mbc = robots.mbcs[i];
      if(mb.nrDof() > 0)
      {
        solver.updateMbc(mbc, i);
        rbd::eulerIntegration(mb, mbc, timeStep);
        rbd::forwardKinematics(mb, mbc);
        rbd::forwardVelocity(mb, mbc);
      }
      success = true;
    }
  }
  return success;
}

const mc_control::MRQPResultMsg & MRQPSolver::send(double/*curTime*/)
{
  __fillResult();
  /*FIXME Use curTime? */
  return qpRes;
}

void MRQPSolver::__fillResult()
{
  qpRes.robots_state.resize(0);
  for(size_t i = 0; i < robots.robots.size(); ++i)
  {
    const mc_rbdyn::Robot & robot = robots.robots[i];
    qpRes.robots_state.push_back(mc_control::RobotMsg());
    std::vector<double> & q = qpRes.robots_state[i].q;
    for(const auto & qv : robot.mbc->q)
    {
      for(const auto & qi : qv)
      {
        q.push_back(qi);
      }
    }
    std::vector<double> & alphaVec = qpRes.robots_state[i].alphaVec;
    for(const auto & av : robot.mbc->alpha)
    {
      for(const auto & ai : av)
      {
        alphaVec.push_back(ai);
      }
    }
    qpRes.robots_state[i].alphaDVec = solver.alphaDVec(i);
  }
  qpRes.lambdaVec = solver.lambdaVec();
}

}
