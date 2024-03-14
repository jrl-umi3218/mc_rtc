#include <mc_solver/TasksQPSolver.h>

#include <mc_rtc/gui.h>
#include <mc_rtc/log/Logger.h>

#include <mc_solver/ConstraintSet.h>
#include <mc_solver/DynamicsConstraint.h>

#include <mc_tasks/MetaTask.h>

#include <boost/chrono.hpp>

namespace mc_solver
{

void TasksQPSolver::addTask(tasks::qp::Task * task)
{
  solver_.addTask(robots().mbs(), task);
}

void TasksQPSolver::removeTask(tasks::qp::Task * task)
{
  solver_.removeTask(task);
  shPtrTasksStorage.erase(std::remove_if(shPtrTasksStorage.begin(), shPtrTasksStorage.end(),
                                         [task](const std::shared_ptr<void> & p) { return task == p.get(); }),
                          shPtrTasksStorage.end());
}

std::pair<int, const tasks::qp::BilateralContact &> TasksQPSolver::contactById(const tasks::qp::ContactId & id) const
{
  const std::vector<tasks::qp::BilateralContact> & contacts = solver_.data().allContacts();
  for(size_t i = 0; i < contacts.size(); ++i)
  {
    if(id == contacts[i].contactId)
    {
      return std::pair<int, const tasks::qp::BilateralContact &>(static_cast<int>(i), contacts[i]);
    }
  }
  // Of course this ref has no value here...
  return std::pair<int, const tasks::qp::BilateralContact &>(-1, tasks::qp::BilateralContact());
}

Eigen::VectorXd TasksQPSolver::lambdaVec(int cIndex) const
{
  return solver_.lambdaVec(cIndex);
}

void TasksQPSolver::setContacts(ControllerToken, const std::vector<mc_rbdyn::Contact> & contacts)
{
  if(logger_)
  {
    for(const auto & contact : contacts_)
    {
      const std::string & r1 = robots().robot(contact.r1Index()).name();
      const std::string & r1S = contact.r1Surface()->name();
      const std::string & r2 = robots().robot(contact.r2Index()).name();
      const std::string & r2S = contact.r2Surface()->name();
      logger_->removeLogEntry("contact_" + r1 + "::" + r1S + "_" + r2 + "::" + r2S);
    }
  }
  if(gui_)
  {
    for(const auto & contact : contacts_)
    {
      const std::string & r1 = robots().robot(contact.r1Index()).name();
      const std::string & r1S = contact.r1Surface()->name();
      const std::string & r2 = robots().robot(contact.r2Index()).name();
      const std::string & r2S = contact.r2Surface()->name();
      gui_->removeElement({"Contacts", "Forces"}, fmt::format("{}::{}/{}::{}", r1, r1S, r2, r2S));
    }
  }
  contacts_ = contacts;
  for(auto & c : contacts_)
  {
    const auto & r1 = robots().robot(c.r1Index());
    if(r1.mb().nrDof() == 0) { c = c.swap(robots()); }
  }
  if(logger_)
  {
    for(const auto & contact : contacts_)
    {
      const std::string & r1 = robots().robot(contact.r1Index()).name();
      const std::string & r1S = contact.r1Surface()->name();
      const std::string & r2 = robots().robot(contact.r2Index()).name();
      const std::string & r2S = contact.r2Surface()->name();
      logger_->addLogEntry("contact_" + r1 + "::" + r1S + "_" + r2 + "::" + r2S,
                           [this, &contact]() { return desiredContactForce(contact); });
    }
  }
  if(gui_)
  {
    for(const auto & contact : contacts_)
    {
      const std::string & r1 = robots().robot(contact.r1Index()).name();
      const std::string & r1S = contact.r1Surface()->name();
      const std::string & r2 = robots().robot(contact.r2Index()).name();
      const std::string & r2S = contact.r2Surface()->name();
      gui_->addElement({"Contacts", "Forces"},
                       mc_rtc::gui::Force(
                           fmt::format("{}::{}/{}::{}", r1, r1S, r2, r2S),
                           [this, &contact]() { return desiredContactForce(contact); }, [this, &contact]()
                           { return robots().robot(contact.r1Index()).surfacePose(contact.r1Surface()->name()); }));
    }
  }
  uniContacts_.clear();
  biContacts_.clear();

  for(const mc_rbdyn::Contact & c : contacts_)
  {
    QPContactPtr qcptr = c.taskContact(*robots_p);
    if(qcptr.unilateralContact)
    {
      uniContacts_.push_back(tasks::qp::UnilateralContact(*qcptr.unilateralContact));
      delete qcptr.unilateralContact;
      qcptr.unilateralContact = 0;
    }
    else
    {
      biContacts_.push_back(tasks::qp::BilateralContact(*qcptr.bilateralContact));
      delete qcptr.bilateralContact;
      qcptr.bilateralContact = 0;
    }
  }

  solver_.nrVars(robots_p->mbs(), uniContacts_, biContacts_);
  updateConstrSize();
}

const sva::ForceVecd TasksQPSolver::desiredContactForce(const mc_rbdyn::Contact & contact) const
{
  const auto & cId = contact.contactId(robots());
  auto qp_contact = contactById(cId);
  if(qp_contact.first != -1)
  {
    const auto & qp_c = qp_contact.second;
    const auto & lambdaV = lambdaVec(qp_contact.first);
    if(lambdaV.size() > 0)
    {
      const auto & qpWrenchInBodyFrame = qp_c.force(lambdaV, qp_c.r1Points, qp_c.r1Cones);
      const auto & qpWrenchInSurfaceFrame = contact.r1Surface()->X_b_s().dualMul(qpWrenchInBodyFrame);
      return qpWrenchInSurfaceFrame;
    }
    else
    {
      mc_rtc::log::error_and_throw("QPSolver - cannot compute desired contact force for surface {}",
                                   contact.r1Surface()->name());
    }
  }
  else { mc_rtc::log::error_and_throw("QPSolver - cannot handle cases where qp_contact.first != -1"); }
}

bool TasksQPSolver::run_impl(FeedbackType fType)
{
  bool success = false;
  switch(fType)
  {
    case FeedbackType::None:
      success = runOpenLoop();
      break;

    case FeedbackType::Joints:
      success = runJointsFeedback(false);
      break;

    case FeedbackType::JointsWVelocity:
      success = runJointsFeedback(true);
      break;

    case FeedbackType::ObservedRobots:
      success = runClosedLoop(true);
      break;

    case FeedbackType::ClosedLoopIntegrateReal:
      success = runClosedLoop(false);
      break;

    default:
      mc_rtc::log::error("FeedbackType set to unknown value");
      success = false;
      break;
  }
  for(auto * dyn : dynamicsConstraints_)
  {
    dyn->motionConstr().computeTorque(solver_.alphaDVec(), solver_.lambdaVec());
    rbd::vectorToParam(dyn->motionConstr().torque(), robot(dyn->robotIndex()).mbc().jointTorque);
  }
  return success;
}

bool TasksQPSolver::runOpenLoop()
{
  for(auto & c : constraints_) { c->update(*this); }
  for(auto & t : metaTasks_)
  {
    t->update(*this);
    t->incrementIterInSolver();
  }
  if(solver_.solveNoMbcUpdate(robots_p->mbs(), robots_p->mbcs()))
  {
    for(size_t i = 0; i < robots_p->mbs().size(); ++i)
    {
      auto & robot = robots().robot(i);
      if(robot.mb().nrDof() > 0)
      {
        solver_.updateMbc(robot.mbc(), static_cast<int>(i));
        robot.eulerIntegration(timeStep);
        robot.forwardKinematics();
        robot.forwardVelocity();
        robot.forwardAcceleration();
      }
    }
    return true;
  }
  return false;
}

bool TasksQPSolver::runJointsFeedback(bool wVelocity)
{
  if(control_q_.size() < robots().size())
  {
    prev_encoders_.resize(robots().size());
    encoders_alpha_.resize(robots().size());
    control_q_.resize(robots().size());
    control_alpha_.resize(robots().size());
  }
  for(size_t i = 0; i < robots().size(); ++i)
  {
    auto & robot = robots().robot(i);
    control_q_[i] = robot.mbc().q;
    control_alpha_[i] = robot.mbc().alpha;
    const auto & encoders = robot.encoderValues();
    if(encoders.size())
    {
      // FIXME Not correct for every joint types
      if(prev_encoders_[i].size() == 0)
      {
        prev_encoders_[i] = robot.encoderValues();
        encoders_alpha_[i].resize(prev_encoders_[i].size());
        if(logger_ && i == 0)
        {
          logger_->addLogEntry("alphaIn", [this]() -> const std::vector<double> & { return encoders_alpha_[0]; });
        }
      }
      for(size_t j = 0; j < encoders.size(); ++j)
      {
        encoders_alpha_[i][j] = (encoders[j] - prev_encoders_[i][j]) / timeStep;
        prev_encoders_[i][j] = encoders[j];
      }
      for(size_t j = 0; j < robot.refJointOrder().size(); ++j)
      {
        const auto & jN = robot.refJointOrder()[j];
        if(!robot.hasJoint(jN)) { continue; }
        auto jI = robot.jointIndexByName(jN);
        robot.mbc().q[jI][0] = encoders[j];
        if(wVelocity) { robot.mbc().alpha[jI][0] = encoders_alpha_[i][j]; }
      }
      robot.forwardKinematics();
      robot.forwardVelocity();
      robot.forwardAcceleration();
    }
  }
  for(auto & c : constraints_) { c->update(*this); }
  for(auto & t : metaTasks_)
  {
    t->update(*this);
    t->incrementIterInSolver();
  }
  if(solver_.solveNoMbcUpdate(robots_p->mbs(), robots_p->mbcs()))
  {
    for(size_t i = 0; i < robots_p->mbs().size(); ++i)
    {
      auto & robot = robots().robot(i);
      robot.mbc().q = control_q_[i];
      robot.mbc().alpha = control_alpha_[i];
      if(robot.mb().nrDof() > 0)
      {
        solver_.updateMbc(robot.mbc(), static_cast<int>(i));
        robot.eulerIntegration(timeStep);
        robot.forwardKinematics();
        robot.forwardVelocity();
        robot.forwardAcceleration();
      }
    }
    return true;
  }
  return false;
}

bool TasksQPSolver::runClosedLoop(bool integrateControlState)
{
  if(control_q_.size() < robots().size())
  {
    control_q_.resize(robots().size());
    control_alpha_.resize(robots().size());
  }

  for(size_t i = 0; i < robots().size(); ++i)
  {
    auto & robot = robots().robot(i);
    const auto & realRobot = realRobots().robot(i);

    // Save old integrator state
    if(integrateControlState)
    {
      control_q_[i] = robot.mbc().q;
      control_alpha_[i] = robot.mbc().alpha;
    }

    // Set robot state from estimator
    robot.mbc().q = realRobot.mbc().q;
    robot.mbc().alpha = realRobot.mbc().alpha;
    robot.forwardKinematics();
    robot.forwardVelocity();
    robot.forwardAcceleration();
  }

  // Update tasks and constraints from estimated robots
  for(auto & c : constraints_) { c->update(*this); }
  for(auto & t : metaTasks_)
  {
    t->update(*this);
    t->incrementIterInSolver();
  }

  // Solve QP and integrate
  if(solver_.solveNoMbcUpdate(robots_p->mbs(), robots_p->mbcs()))
  {
    for(size_t i = 0; i < robots_p->mbs().size(); ++i)
    {
      auto & robot = robots().robot(i);
      if(integrateControlState)
      {
        robot.mbc().q = control_q_[i];
        robot.mbc().alpha = control_alpha_[i];
      }
      if(robot.mb().nrDof() > 0)
      {
        solver_.updateMbc(robot.mbc(), static_cast<int>(i));
        robot.eulerIntegration(timeStep);
        robot.forwardKinematics();
        robot.forwardVelocity();
        robot.forwardAcceleration();
      }
    }
    return true;
  }
  return false;
}

void TasksQPSolver::updateConstrSize()
{
  solver_.updateConstrSize();
}

void TasksQPSolver::updateNrVars()
{
  solver_.nrVars(robots_p->mbs(), uniContacts_, biContacts_);
}

void TasksQPSolver::updateNrVars(const mc_rbdyn::Robots & robots)
{
  solver_.updateNrVars(robots.mbs());
}

using boost_ms = boost::chrono::duration<double, boost::milli>;
using boost_ns = boost::chrono::duration<double, boost::nano>;

double TasksQPSolver::solveTime()
{
  return boost_ms(boost_ns(solver_.solveTime().wall)).count();
}

double TasksQPSolver::solveAndBuildTime()
{
  return boost_ms(boost_ns(solver_.solveAndBuildTime().wall)).count();
}

const Eigen::VectorXd & TasksQPSolver::result() const
{
  return solver_.result();
}

void TasksQPSolver::addDynamicsConstraint(mc_solver::DynamicsConstraint * dynamics)
{
  dynamicsConstraints_.push_back(dynamics);
}

void TasksQPSolver::removeDynamicsConstraint(mc_solver::ConstraintSet * cs)
{
  auto it = std::find_if(dynamicsConstraints_.begin(), dynamicsConstraints_.end(),
                         [&cs](DynamicsConstraint * dyn) { return static_cast<ConstraintSet *>(dyn) == cs; });
  if(it != dynamicsConstraints_.end()) { dynamicsConstraints_.erase(it); }
}

} // namespace mc_solver
