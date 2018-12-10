#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rtc/logging.h>
#include <mc_solver/KinematicsConstraint.h>
#include <mc_solver/QPSolver.h>
#include <mc_tasks/MetaTask.h>

#include <Tasks/Bounds.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace
{

std::vector<mc_solver::ContactMsg> contactsMsgFromContacts(const mc_rbdyn::Robots & robots,
                                                           const std::vector<mc_rbdyn::Contact> & contacts)
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
    sva::PTransformd X_b1_b2 = X_0_b2 * X_0_b1.inv();

    mc_solver::ContactMsg msg;
    msg.r1_index = static_cast<uint16_t>(c.r1Index());
    msg.r2_index = static_cast<uint16_t>(c.r2Index());
    msg.r1_body = c.r1Surface()->bodyName();
    msg.r2_body = c.r2Surface()->bodyName();
    msg.r1_surface = c.r1Surface()->name();
    msg.r2_surface = c.r2Surface()->name();
    msg.r1_points = const_cast<const mc_rbdyn::Surface &>(*(c.r1Surface())).points();
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

QPSolver::QPSolver(double timeStep) : QPSolver{std::make_shared<mc_rbdyn::Robots>(), timeStep} {}

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
  if(std::find(metaTasks_.begin(), metaTasks_.end(), task) == metaTasks_.end())
  {
    metaTasks_.push_back(task);
    task->addToSolver(*this);
    if(logger_)
    {
      task->addToLogger(*logger_);
    }
    if(gui_)
    {
      addTaskToGUI(task);
    }
    LOG_INFO("Added task " << task->name())
  }
}

void QPSolver::removeTask(tasks::qp::Task * task)
{
  solver.removeTask(task);
  shPtrTasksStorage.erase(std::remove_if(shPtrTasksStorage.begin(), shPtrTasksStorage.end(),
                                         [task](const std::shared_ptr<void> & p) { return task == p.get(); }),
                          shPtrTasksStorage.end());
}

void QPSolver::removeTask(mc_tasks::MetaTask * task)
{
  auto it = std::find(metaTasks_.begin(), metaTasks_.end(), task);
  if(it != metaTasks_.end())
  {
    task->removeFromSolver(*this);
    if(logger_)
    {
      task->removeFromLogger(*logger_);
    }
    if(gui_)
    {
      task->removeFromGUI(*gui_);
    }
    LOG_INFO("Removed task " << task->name())
    metaTasks_.erase(it);
    shPtrTasksStorage.erase(std::remove_if(shPtrTasksStorage.begin(), shPtrTasksStorage.end(),
                                           [task](const std::shared_ptr<void> & p) { return task == p.get(); }),
                            shPtrTasksStorage.end());
  }
}

std::pair<int, const tasks::qp::BilateralContact &> QPSolver::contactById(const tasks::qp::ContactId & id) const
{
  const std::vector<tasks::qp::BilateralContact> & contacts = solver.data().allContacts();
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

Eigen::VectorXd QPSolver::lambdaVec(int cIndex) const
{
  return solver.lambdaVec(cIndex);
}

void QPSolver::setContacts(const std::vector<mc_rbdyn::Contact> & contacts)
{
  if(logger_)
  {
    for(const auto & contact : contacts_)
    {
      logger_->removeLogEntry("contact_" + contact.r1Surface()->name() + "_" + contact.r2Surface()->name());
    }
  }
  contacts_ = contacts;
  if(logger_)
  {
    for(const auto & contact : contacts_)
    {
      logger_->addLogEntry("contact_" + contact.r1Surface()->name() + "_" + contact.r2Surface()->name(),
                           [this, &contact]() { return desiredContactForce(contact); });
    }
  }
  uniContacts.clear();
  biContacts.clear();
  qpRes.contacts.clear();

  if(gui_)
  {
    gui_->removeCategory({"Contacts", "Remove"});
  }
  auto allBut = [](const std::vector<mc_rbdyn::Contact> & cs, const mc_rbdyn::Contact & c) {
    std::vector<mc_rbdyn::Contact> ret = cs;
    auto it = std::find(ret.begin(), ret.end(), c);
    ret.erase(it);
    return ret;
  };

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
    if(gui_)
    {
      std::string bName = robot(c.r1Index()).name() + "::" + c.r1Surface()->name() + " & " + robot(c.r2Index()).name()
                          + "::" + c.r2Surface()->name();
      auto nContacts = allBut(contacts, c);
      gui_->addElement({"Contacts", "Remove"},
                       mc_rtc::gui::Button(bName, [nContacts, this]() { setContacts(nContacts); }));
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

const std::vector<mc_rbdyn::Contact> & QPSolver::contacts() const
{
  return contacts_;
}

const std::vector<mc_tasks::MetaTask *> & QPSolver::tasks() const
{
  return metaTasks_;
}

const sva::ForceVecd QPSolver::desiredContactForce(const mc_rbdyn::Contact & contact) const
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
      LOG_ERROR_AND_THROW(std::runtime_error, "QPSolver - cannot compute desired contact force for surface "
                                                  << contact.r1Surface()->name());
    }
  }
  else
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "QPSolver - cannot handle cases where qp_contact.first != -1");
  }
}

bool QPSolver::run(FeedbackType fType)
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
    default:
      LOG_ERROR("FeedbackType set to unknown value")
      break;
  }
  if(success)
  {
    __fillResult();
  }
  return success;
}

bool QPSolver::runOpenLoop()
{
  for(auto & t : metaTasks_)
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
    }
    return true;
  }
  return false;
}

bool QPSolver::runJointsFeedback(bool wVelocity)
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
        if(!robot.hasJoint(jN))
        {
          continue;
        }
        auto jI = robot.jointIndexByName(jN);
        robot.mbc().q[jI][0] = encoders[j];
        if(wVelocity)
        {
          robot.mbc().alpha[jI][0] = encoders_alpha_[i][j];
        }
      }
      robot.forwardKinematics();
      robot.forwardVelocity();
      robot.forwardAcceleration();
    }
  }
  for(auto & t : metaTasks_)
  {
    t->update();
  }
  if(solver.solveNoMbcUpdate(robots_p->mbs(), robots_p->mbcs()))
  {
    for(size_t i = 0; i < robots_p->mbs().size(); ++i)
    {
      auto & robot = robots().robot(i);
      robot.mbc().q = control_q_[i];
      robot.mbc().alpha = control_alpha_[i];
      if(robot.mb().nrDof() > 0)
      {
        solver.updateMbc(robot.mbc(), static_cast<int>(i));
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

bool QPSolver::runClosedLoop(std::shared_ptr<mc_rbdyn::Robots> real_robots)
{
  // =============================
  // 1 - Save old integrator state
  // =============================
  std::vector<std::vector<double>> oldQ(robot().mbc().q);
  std::vector<std::vector<double>> oldQd(robot().mbc().alpha);

  // Set robot state to estimator
  robot().mbc().q = real_robots->robot().mbc().q;
  robot().mbc().alpha = real_robots->robot().mbc().alpha;

  // COMPUTE QP on estimated robot
  for(auto & t : metaTasks_)
  {
    t->update();
  }
  bool success = solver.solveNoMbcUpdate(robots().mbs(), robots().mbcs());
  solver.updateMbc(robot().mbc(), static_cast<int>(robots().robotIndex()));

  // Apply computed acceleration to integrator state
  robot().mbc().q = oldQ;
  robot().mbc().alpha = oldQd;

  // Integrate Qdd on top of integrator state q, qd
  rbd::eulerIntegration(robot().mb(), robot().mbc(), timeStep);
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());

  __fillResult();
  return success;
}

const QPResultMsg & QPSolver::send(double /*curTime*/)
{
  return qpRes;
}

void QPSolver::__fillResult()
{
  qpRes.zmps.resize(robots().robots().size());
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

    qpRes.zmps[i].x = robot.zmpTarget().x();
    qpRes.zmps[i].y = robot.zmpTarget().y();
    qpRes.zmps[i].z = robot.zmpTarget().z();
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

void QPSolver::fillTorque(const mc_solver::DynamicsConstraint & dynamicsConstraint)
{
  if(dynamicsConstraint.inSolver())
  {
    dynamicsConstraint.motionConstr->computeTorque(solver.alphaDVec(), solver.lambdaVec());
    auto & robot = robots().robot(dynamicsConstraint.robotIndex());
    robot.mbc().jointTorque = rbd::vectorToDof(robot.mb(), dynamicsConstraint.motionConstr->torque());
  }
  else
  {
    auto & robot = robots().robot(dynamicsConstraint.robotIndex());
    robot.mbc().jointTorque = rbd::vectorToDof(robot.mb(), Eigen::VectorXd::Zero(robot.mb().nrDof()));
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

const Eigen::VectorXd & QPSolver::result() const
{
  return solver.result();
}

void QPSolver::logger(std::shared_ptr<mc_rtc::Logger> logger)
{
  if(logger_)
  {
    for(auto t : metaTasks_)
    {
      t->removeFromLogger(*logger_);
    }
  }
  logger_ = logger;
  if(logger_)
  {
    for(auto t : metaTasks_)
    {
      t->addToLogger(*logger_);
    }
  }
}

void QPSolver::gui(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
{
  if(gui_)
  {
    for(auto t : metaTasks_)
    {
      t->removeFromGUI(*gui_);
    }
  }
  gui_ = gui;
  if(gui_)
  {
    for(auto t : metaTasks_)
    {
      addTaskToGUI(t);
    }
    gui_->addElement({"Contacts", "Add"},
                     mc_rtc::gui::Form("Add contact",
                                       [this](const mc_rtc::Configuration & data) {
                                         LOG_INFO("Add contact " << data("R0") << "::" << data("R0 surface") << "/"
                                                                 << data("R1") << "::" << data("R1 surface"))
                                         auto str2idx = [this](const std::string & rName) {
                                           for(const auto & r : robots())
                                           {
                                             if(r.name() == rName)
                                             {
                                               return r.robotIndex();
                                             }
                                           }
                                           LOG_ERROR("The robot name you provided does not match any in the solver")
                                           return static_cast<unsigned int>(robots().size());
                                         };
                                         unsigned int r0Index = str2idx(data("R0"));
                                         unsigned int r1Index = str2idx(data("R1"));
                                         std::string r0Surface = data("R0 surface");
                                         std::string r1Surface = data("R1 surface");
                                         if(r0Index < robots().size() && r1Index < robots().size())
                                         {
                                           contacts_.push_back({robots(), r0Index, r1Index, r0Surface, r1Surface});
                                           setContacts(contacts_);
                                         }
                                       },
                                       mc_rtc::gui::FormDataComboInput{"R0", true, {"robots"}},
                                       mc_rtc::gui::FormDataComboInput{"R0 surface", true, {"surfaces", "$R0"}},
                                       mc_rtc::gui::FormDataComboInput{"R1", true, {"robots"}},
                                       mc_rtc::gui::FormDataComboInput{"R1 surface", true, {"surfaces", "$R1"}}));
  }
}

void QPSolver::addTaskToGUI(mc_tasks::MetaTask * t)
{
  assert(gui_);
  t->addToGUI(*gui_);
  gui_->addElement({"Tasks", t->name()},
                   mc_rtc::gui::Button("Remove from solver", [this, t]() { this->removeTask(t); }));
}

} // namespace mc_solver
