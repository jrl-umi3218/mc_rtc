#include <mc_solver/QPSolver.h>

#include <Tasks/Bounds.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <mc_rbdyn/stance.h>
#include <mc_rbdyn/Surface.h>

#include <mc_rtc/logging.h>

#include <mc_solver/KinematicsConstraint.h>

#include <mc_tasks/MetaTask.h>

namespace
{

std::vector<mc_solver::ContactMsg> contactsMsgFromContacts
  (const mc_rbdyn::Robots & robots, const std::vector<mc_rbdyn::Contact> & contacts)
{
  std::vector<mc_solver::ContactMsg> res;

  for(const auto & c : contacts)
  {
    const auto & r1 = robots.robot(c.r1Index());
    const auto & r2 = robots.robot(c.r2Index());

    unsigned int r1BodyIndex = r1.bodyIndexByName(c.r1Surface()->bodyName());
    unsigned int r2BodyIndex = r2.bodyIndexByName(c.r2Surface()->bodyName());

    sva::PTransformd X_0_b1 = r1.mbc().bodyPosW[r1BodyIndex];
    sva::PTransformd X_0_b2 = r2.mbc().bodyPosW[r2BodyIndex];
    sva::PTransformd X_b1_b2 = X_0_b2*X_0_b1.inv();

    mc_solver::ContactMsg msg;
    msg.r1_index = static_cast<uint16_t>(c.r1Index());
    msg.r2_index = static_cast<uint16_t>(c.r2Index());
    msg.r1_body = c.r1Surface()->bodyName();
    msg.r2_body = c.r2Surface()->bodyName();
    msg.r1_surface = c.r1Surface()->name();
    msg.r2_surface = c.r2Surface()->name();
    msg.r1_points = const_cast<const mc_rbdyn::Surface&>(*(c.r1Surface())).points();
    msg.X_b1_b2 = X_b1_b2;
    msg.nr_generators = static_cast<uint16_t>(mc_rbdyn::Contact::nrConeGen);
    msg.mu = mc_rbdyn::Contact::defaultFriction;
    res.push_back(msg);
  }

  return res;
}

} // anonymous namespace

namespace mc_solver
{
QPSolver::QPSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double timeStep)
: robots_p(robots), timeStep(timeStep), solver()
{
  if(timeStep <= 0)
  {
    LOG_ERROR_AND_THROW(std::invalid_argument, "timeStep has to be > 0! timeStep = " << timeStep)
  }
}

QPSolver::QPSolver(double timeStep)
: QPSolver{std::make_shared<mc_rbdyn::Robots>(), timeStep}
{
}

void QPSolver::addConstraintSet(ConstraintSet & cs)
{
  cs.addToSolver(robots().mbs(), solver);
}

void QPSolver::removeConstraintSet(ConstraintSet & cs)
{
  cs.removeFromSolver(solver);
}

void QPSolver::addTask(tasks::qp::Task * task)
{
  solver.addTask(robots().mbs(), task);
}

void QPSolver::addTask(mc_tasks::MetaTask * task)
{
  if(std::find(metaTasks.begin(), metaTasks.end(), task) == metaTasks.end())
  {
    metaTasks.push_back(task);
    task->addToSolver(*this);
  }
}

void QPSolver::removeTask(tasks::qp::Task * task)
{
  solver.removeTask(task);
  shPtrTasksStorage.erase(std::remove_if(
    shPtrTasksStorage.begin(), shPtrTasksStorage.end(),
    [task](const std::shared_ptr<void> & p)
    {
      return task == p.get();
    }));
}

void QPSolver::removeTask(mc_tasks::MetaTask * task)
{
  auto it = std::find(metaTasks.begin(), metaTasks.end(), task);
  if(it != metaTasks.end())
  {
    metaTasks.erase(it);
    task->removeFromSolver(*this);
    shPtrTasksStorage.erase(std::remove_if(
      shPtrTasksStorage.begin(), shPtrTasksStorage.end(),
      [task](const std::shared_ptr<void> & p)
      {
        return task == p.get();
      }));
  }
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

Eigen::VectorXd QPSolver::lambdaVec(int cIndex) const
{
  return solver.lambdaVec(cIndex);
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
  for(auto & t : metaTasks)
  {
    t->update();
  }
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
    __fillResult();
  }
  return success;
}

const QPResultMsg & QPSolver::send(double/*curTime*/)
{
  return qpRes;
}

void QPSolver::__fillResult()
{
  qpRes.robots_state.resize(robots().robots().size());
  for(unsigned int i = 0; i < robots().robots().size(); ++i)
  {
    const mc_rbdyn::Robot & robot = robots().robot(i);
    auto & q = qpRes.robots_state[i].q;
    auto & alphaVec = qpRes.robots_state[i].alphaVec;
    for(const auto & j : robot.mb().joints())
    {
      auto jIndex = robot.jointIndexByName(j.name());
      q[j.name()] = robot.mbc().q[jIndex];
      alphaVec[j.name()] = robot.mbc().alpha[jIndex];
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

void QPSolver::updateConstrSize()
{
  solver.updateConstrSize();
}

void QPSolver::updateNrVars()
{
  solver.nrVars(robots_p->mbs(), uniContacts, biContacts);
}

double QPSolver::dt() const
{
  return timeStep;
}

tasks::qp::SolverData & QPSolver::data()
{
  return solver.data();
}

void QPSolver::fillTorque(const mc_solver::DynamicsConstraint& dynamicsConstraint)
{
  if(dynamicsConstraint.inSolver())
  {
    dynamicsConstraint.motionConstr->computeTorque(solver.alphaDVec(), solver.lambdaVec());
    robot().mbc().jointTorque = rbd::vectorToDof(robot().mb(), dynamicsConstraint.motionConstr->torque());
  }
  else
  {
    robot().mbc().jointTorque = rbd::vectorToDof(robot().mb(), Eigen::VectorXd::Zero(robot().mb().nrDof()));
  }
}

boost::timer::cpu_times QPSolver::solveTime()
{
  return solver.solveTime();
}

boost::timer::cpu_times QPSolver::solveAndBuildTime()
{
  return solver.solveAndBuildTime();
}

}
