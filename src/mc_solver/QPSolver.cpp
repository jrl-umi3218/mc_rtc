#include <mc_solver/QPSolver.h>

#include <Tasks/Bounds.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <mc_solver/contact_util.h>

#include <mc_rbdyn/stance.h>
#include <mc_rbdyn/Surface.h>

#include <mc_rtc/logging.h>

#include <mc_solver/KinematicsConstraint.h>

namespace mc_solver
{
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
