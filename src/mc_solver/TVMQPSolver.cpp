/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/gui/Button.h>
#include <mc_solver/TVMQPSolver.h>

#include <mc_solver/ContactConstraint.h>
#include <mc_solver/DynamicsConstraint.h>

#include <mc_tasks/MetaTask.h>

#include <mc_tvm/ContactFunction.h>
#include <mc_tvm/FeasiblePolytope.h>
#include <mc_tvm/ForceInPolytopeFunction.h>
#include <mc_tvm/Robot.h>

#include <mc_rtc/gui/Force.h>

#include <tvm/solver/defaultLeastSquareSolver.h>
#include <tvm/task_dynamics/ProportionalDerivative.h>

#include <filesystem>
namespace fs = std::filesystem;

namespace mc_solver
{

inline static Eigen::MatrixXd discretizedFrictionCone(double muI)
{
  // 4 faces, 3 forces
  Eigen::MatrixXd C(4, 3);
  double mu = muI / std::sqrt(2);
  C << Eigen::Matrix2d::Identity(), Eigen::Vector2d::Constant(mu), -Eigen::Matrix2d::Identity(),
      Eigen::Vector2d::Constant(mu);
  return C;
}

TVMQPSolver::TVMQPSolver(mc_rbdyn::RobotsPtr robots, double dt)
: QPSolver(robots, dt, Backend::TVM), solver_(tvm::solver::DefaultLSSolverOptions{})
{
  tvm::graph::internal::Logger::logger().enable();
}

TVMQPSolver::TVMQPSolver(double dt) : QPSolver(dt, Backend::TVM), solver_(tvm::solver::DefaultLSSolverOptions{}) {}

void TVMQPSolver::gui(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
{
  QPSolver::gui(gui);

  gui->removeElements(this);

  gui_->addElement(this, {"Solver", "TVM"},
                   mc_rtc::gui::Button("Generate Graph dot file (graphviz)", [this]() { this->saveGraphDotFile(); }));
}

size_t TVMQPSolver::getContactIdx(const mc_rbdyn::Contact & contact)
{
  for(size_t i = 0; i < contacts_.size(); ++i)
  {
    if(*contacts_[i] == contact) { return i; }
  }
  return contacts_.size();
}

void TVMQPSolver::setContacts(ControllerToken, const std::vector<mc_rbdyn::Contact> & contacts)
{
  for(const auto & c : contacts) { addContact(c); }
  size_t i = 0;
  for(auto it = contacts_.begin(); it != contacts_.end();)
  {
    const auto & c = *it;
    if(std::find_if(contacts.cbegin(), contacts.cend(), [&c](const auto & contact) { return *c == contact; })
       == contacts.cend())
    {
      const std::string & r1 = robots().robot(c->r1Index()).name();
      const std::string & r1S = c->r1Surface()->name();
      const std::string & r2 = robots().robot(c->r2Index()).name();
      const std::string & r2S = c->r2Surface()->name();
      logger_->removeLogEntry("contact_" + r1 + "::" + r1S + "_" + r2 + "::" + r2S);
      logger_->removeLogEntry("contact_" + r2 + "::" + r2S + "_" + r1 + "::" + r1S);
      if(gui_)
      {
        gui_->removeElement({"Contacts", "Forces"}, fmt::format("{}::{}/{}::{}", r1, r1S, r2, r2S));
        gui_->removeElement({"Contacts", "Forces"}, fmt::format("{}::{}/{}::{}", r2, r2S, r1, r1S));
      }
      it = removeContact(i);
    }
    else
    {
      ++i;
      ++it;
    }
  }
}

const sva::ForceVecd TVMQPSolver::desiredContactForce(const mc_rbdyn::Contact & id) const
{
  const auto & r1 = robot(id.r1Index());
  auto it = dynamics_.find(r1.name());
  if(it != dynamics_.end()) { return it->second->dynamicFunction().contactForce(r1.frame(id.r1Surface()->name())); }
  return sva::ForceVecd::Zero();
}

const sva::ForceVecd TVMQPSolver::desiredContactForce2(const mc_rbdyn::Contact & id) const
{
  const auto & r2 = robot(id.r2Index());
  auto it = dynamics_.find(r2.name());
  if(it != dynamics_.end()) { return it->second->dynamicFunction().contactForce(r2.frame(id.r2Surface()->name())); }
  return sva::ForceVecd::Zero();
}

double TVMQPSolver::solveTime()
{
  return solve_dt_.count();
}

double TVMQPSolver::solveAndBuildTime()
{
  return solve_dt_.count();
}

bool TVMQPSolver::run_impl(FeedbackType fType)
{
  switch(fType)
  {
    case FeedbackType::None:
      return runOpenLoop();
    case FeedbackType::Joints:
      return runJointsFeedback(false);
    case FeedbackType::JointsWVelocity:
      return runJointsFeedback(true);
    case FeedbackType::ObservedRobots:
      return runClosedLoop(true);
    case FeedbackType::ClosedLoopIntegrateReal:
      return runClosedLoop(false);
    default:
      mc_rtc::log::error("FeedbackType set to unknown value");
      return false;
  }
}

bool TVMQPSolver::runCommon()
{
  for(auto & c : constraints_) { c->update(*this); }
  for(auto & t : metaTasks_)
  {
    t->update(*this);
    t->incrementIterInSolver();
  }
  auto start_t = mc_rtc::clock::now();
  auto r = solver_.solve(problem_);
  solve_dt_ = mc_rtc::clock::now() - start_t;
  return r;
}

bool TVMQPSolver::runOpenLoop()
{
  if(runCommon())
  {
    for(auto & robot : *robots_p)
    {
      auto & mb = robot.mb();
      if(mb.nrDof() > 0) { updateRobot(robot); }
    }
    return true;
  }
  return false;
}

bool TVMQPSolver::runJointsFeedback(bool wVelocity)
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
    auto & robot = robots_p->robot(i);
    control_q_[i] = robot.q();
    control_alpha_[i] = robot.alpha();
    const auto & encoders = robot.encoderValues();
    if(encoders.size())
    {
      // FIXME Not correct for every joint types
      if(prev_encoders_[i].size() == 0)
      {
        prev_encoders_[i] = robot.encoderValues();
        encoders_alpha_[i].resize(prev_encoders_[i].size());
      }
      for(size_t j = 0; j < encoders.size(); ++j)
      {
        encoders_alpha_[i][j] = (encoders[j] - prev_encoders_[i][j]) / timeStep;
        prev_encoders_[i][j] = encoders[j];
      }
      const auto & rjo = robot.module().ref_joint_order();
      for(size_t j = 0; j < rjo.size(); ++j)
      {
        auto jI = robot.jointIndexInMBC(j);
        if(jI == -1) { continue; }
        robot.q()[static_cast<size_t>(jI)][0] = encoders[j];
        if(wVelocity) { robot.alpha()[static_cast<size_t>(jI)][0] = encoders_alpha_[i][j]; }
      }
      robot.forwardKinematics();
      robot.forwardVelocity();
      robot.forwardAcceleration();
    }
  }
  if(runCommon())
  {
    for(size_t i = 0; i < robots_p->size(); ++i)
    {
      auto & robot = robots_p->robot(i);
      if(robot.mb().nrDof() == 0) { continue; }
      robot.q() = control_q_[i];
      robot.alpha() = control_alpha_[i];
      updateRobot(robot);
    }
    return true;
  }
  return false;
}

bool TVMQPSolver::runClosedLoop(bool integrateControlState)
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

  // Solve QP and integrate
  if(runCommon())
  {
    for(size_t i = 0; i < robots_p->size(); ++i)
    {
      auto & robot = robots_p->robot(i);
      if(robot.mb().nrDof() == 0) { continue; }
      if(integrateControlState)
      {
        robot.q() = control_q_[i];
        robot.alpha() = control_alpha_[i];
      }
      updateRobot(robot);
    }
    return true;
  }
  return false;
}

void TVMQPSolver::updateRobot(mc_rbdyn::Robot & robot)
{
  auto & tvm_robot = robot.tvmRobot();
  rbd::vectorToParam(tvm_robot.tau()->value(), robot.controlTorque());
  rbd::vectorToParam(tvm_robot.alphaD()->value(), robot.alphaD());
  robot.eulerIntegration(timeStep);
  robot.forwardKinematics();
  robot.forwardVelocity();
  robot.forwardAcceleration();
}

void TVMQPSolver::addDynamicsConstraint(mc_solver::DynamicsConstraint * dyn)
{
  const auto & r = robot(dyn->robotIndex());
  if(dynamics_.count(r.name()))
  {
    mc_rtc::log::error_and_throw("Only one dynamic constraint can be added for a given robot and {} already has one",
                                 r.name());
  }
  dynamics_[r.name()] = dyn;
  for(size_t i = 0; i < contacts_.size(); ++i)
  {
    const auto & contact = contacts_[i];
    auto & data = contactsData_[i];
    bool isR1 = contact->r1Index() == dyn->robotIndex();
    bool isR2 = contact->r2Index() == dyn->robotIndex();
    if(isR1 || isR2)
    {
      const auto & r1 = robot(contact->r1Index());
      const auto & r2 = robot(contact->r2Index());
      const auto & s1 = *contact->r1Surface();
      const auto & s2 = *contact->r2Surface();
      const auto & f1 = r1.frame(s1.name());
      const auto & f2 = r2.frame(s2.name());
      // FIXME Debug mc_rbdyn::intersection
      // auto s1Points = mc_rbdyn::intersection(s1, s2);
      const auto & s1Points = s1.points();
      if(isR1)
      {
        addContactToDynamics(r1.name(), f1, s1Points, data.f1_, data.f1Constraints_, data.f1Targets_, *contact, 1.0);
      }
      if(isR2)
      {
        std::vector<sva::PTransformd> s2Points;
        s2Points.reserve(s1Points.size());
        auto X_b2_b1 =
            r1.mbc().bodyPosW[r1.bodyIndexByName(f1.body())] * r2.mbc().bodyPosW[r2.bodyIndexByName(f2.body())].inv();
        for(const auto & X_b1_p : s1Points) { s2Points.push_back(X_b1_p * X_b2_b1); }
        addContactToDynamics(r2.name(), f2, s2Points, data.f2_, data.f2Constraints_, data.f2Targets_, *contact, -1.0);
      }
    }
  }
}

void TVMQPSolver::removeDynamicsConstraint(mc_solver::ConstraintSet * maybe_dyn)
{
  for(const auto & [r, dyn] : dynamics_)
  {
    if(static_cast<const mc_solver::ConstraintSet *>(dyn) == maybe_dyn)
    {
      return removeDynamicsConstraint(static_cast<mc_solver::DynamicsConstraint *>(maybe_dyn));
    }
  }
}

void TVMQPSolver::removeDynamicsConstraint(mc_solver::DynamicsConstraint * dyn)
{
  const auto & r = robot(dyn->robotIndex());
  dynamics_.erase(r.name());
  for(size_t i = 0; i < contacts_.size(); ++i)
  {
    const auto & contact = contacts_[i];
    auto & data = contactsData_[i];
    auto clearContacts = [&](const std::string & robot, tvm::VariableVector & forces,
                             std::vector<tvm::TaskWithRequirementsPtr> & constraints,
                             std::vector<tvm::TaskWithRequirementsPtr> & targets)
    {
      if(robot != r.name()) { return; }
      for(auto & c : constraints) { problem_.remove(*c); }
      constraints.clear();
      for(auto & t : targets) { problem_.remove(*t); }
      targets.clear();
      forces = tvm::VariableVector();
    };
    const auto & r1 = robot(contact->r1Index());
    clearContacts(r1.name(), data.f1_, data.f1Constraints_, data.f1Targets_);
    const auto & r2 = robot(contact->r2Index());
    clearContacts(r2.name(), data.f2_, data.f2Constraints_, data.f2Targets_);
  }
}

// FIXME remove dir argument after confirming new logic handles it correctly
void TVMQPSolver::addContactToDynamics(const std::string & robot,
                                       const mc_rbdyn::RobotFrame & frame,
                                       const std::vector<sva::PTransformd> & points,
                                       tvm::VariableVector & forces,
                                       std::vector<tvm::TaskWithRequirementsPtr> & constraints,
                                       std::vector<tvm::TaskWithRequirementsPtr> & targets,
                                       mc_rbdyn::Contact & contact,
                                       double dir)
{
  auto it = dynamics_.find(robot);
  bool hasForceVar;
  // If this robot does not have a dynamics constraint, nothing to add on this side, just return
  if(it == dynamics_.end()) { return; }
  if(constraints.size() || targets.size())
  {
    // FIXME Instead of this we should be able to change C
    // FIXME We keep this as a safety but in practice this is not called anymore (no hasWork if friction changed) and is
    // now handled by the feasible polytope constraint automatically
    mc_rtc::log::critical("[SHOULD NOT APPEAR] removing already existing contact constraint");
    for(const auto & c : constraints) { problem_.remove(*c); }
    constraints.clear();
    for(const auto & t : targets) { problem_.remove(*t); }
    targets.clear();
  }
  else
  {
    auto & dyn = it->second->dynamicFunction();

    /* Now we consider the force decision variable for the contact:
    If this is a completely new contact we want to create the force variable, but if the other side of the contact
    has already been created by the other robot's dynamic function we want to reuse it with the opposite direction
    */
    // XXX this is only handled by 6d contact var, otherwise the total of the force variables must be opposite instead
    // of just the variables, much more complex logic
    tvm::VariableVector existingVariables;
    // Iterate on all dynamics functions in the solver
    for(const auto & [_, d] : dynamics_)
    {
      // check if a dynamics function already handles this contact's other frame
      // i.e. the force decision variable already exists and we just need to reuse it in direction -1
      // instead of creating a new one

      if(frame.robot().robotIndex() == contact.r1Index())
      {
        // This means the other robot in the contact is r2
        if(d->robotIndex() == contact.r2Index())
        {
          // We found a dynamics function involving the other robot of this contact, check if it has this contact.
          // The robot frame would be r2surface of this contact
          existingVariables = d->dynamicFunction().getForceVariables(contact.r2Surface()->name());
        }
      }
      else
      {
        // This means the other robot in the contact is r1
        if(d->robotIndex() == contact.r1Index())
        {
          // We found a dynamics function involving the other robot of this contact, check if it has this contact.
          // The robot frame would be r1surface of this contact
          existingVariables = d->dynamicFunction().getForceVariables(contact.r1Surface()->name());
        }
      }
    }

    // FIXME need to check what happens to var if first dyn function is destroyed
    if(existingVariables.numberOfVariables() != 0)
    {
      // There were pre existing force variables for the other side of this contact
      hasForceVar = false;
      forces.add(existingVariables);
      dyn.addContact6d(frame, existingVariables[0], contact);
    }
    else
    {
      // Create decision variables
      hasForceVar = true;
      // forces = dyn.addContact3d(frame, points, dir);
      forces.add(dyn.addContact6d(frame, contact));
    }
  }

  for(int i = 0; i < forces.numberOfVariables(); ++i)
  {
    auto & f = forces[i];
    auto polyFunction =
        std::make_shared<mc_tvm::ForceInPolytopeFunction>(contact, f, robots().robotIndex(robot), hasForceVar);
    // We want the force to stay inside of the polytope so the distance value should stay negative
    constraints.push_back(
        problem_.add(polyFunction <= 0., tvm::task_dynamics::None(), {tvm::requirements::PriorityLevel(0)}));
    // Add a minimization on the force variable with a low weight
    // TODO maybe find a way to parametrize a desired anisotropic weight from the mc_rbdyn::Contact ?
    // TODO write a linear function to target zero (minimize) or a specific target
    // targets.push_back(problem_.add(f == 0.0, {tvm::requirements::PriorityLevel(1),
    // tvm::requirements::Weight(0.0001)}));
  }
}

auto TVMQPSolver::addVirtualContactImpl(const mc_rbdyn::Contact & contact) -> std::tuple<size_t, bool>
{
  // FIXME handle swapping of contact
  bool hasWork = false;
  auto idx = getContactIdx(contact);
  if(idx < contacts_.size())
  {
    // This contact already exists in the solver
    const auto & oldContact = *contacts_[idx];
    if(oldContact.dof() == contact.dof() && oldContact.friction() == contact.friction())
    {
      // dof and friction stayed, no copy or work to do
      return std::make_tuple(idx, hasWork);
    }
    // Update internal contact map if any change
    *contacts_[idx] = contact;
  }
  else
  {
    // New contact so need to do everything
    hasWork = true;
    contacts_.emplace_back(std::make_shared<mc_rbdyn::Contact>(contact));
  }

  const auto storedContact = contacts_[idx];

  // Get the contactData element for this contact or create it if new contact
  auto & data = idx < contactsData_.size() ? contactsData_[idx] : contactsData_.emplace_back();
  const auto & r1 = robot(contact.r1Index());
  const auto & r2 = robot(contact.r2Index());
  const auto & f1 = r1.frame(contact.r1Surface()->name());
  const auto & f2 = r2.frame(contact.r2Surface()->name());
  if(!data.contactConstraint_) // New contact
  {
    // Create a new contact function, add it to the problem and keep it in the contactData element
    auto contact_fn = std::make_shared<mc_tvm::ContactFunction>(f1, f2, contact.dof());
    // Check if a contact constraint exists in the QP
    for(const auto & constraint : constraints())
    {
      if(auto contactConstraint = dynamic_cast<mc_solver::ContactConstraint *>(constraint))
      {
        auto contactType = contactConstraint->contactType();

        // Add geometric constraint according to the type of the solver contact constraint
        switch(contactType)
        {
          case ContactConstraint::ContactType::Acceleration:
          {
            // Acceleration constraint: the second order dynamics of the contact function, ie the relative acceleration
            // between the frames tracks a reference of zero
            // XXX A non zero reference could be tracked using the tvm::task_dynamics::ReferenceAcceleration for example
            data.contactConstraint_ =
                problem_.add(contact_fn == 0., tvm::task_dynamics::PD(0., 0.), {tvm::requirements::PriorityLevel(0)});
            break;
          }
          case ContactConstraint::ContactType::Velocity:
          {
            // Velocity constraint: the dynamics of the contact function track only the velocity error
            data.contactConstraint_ = problem_.add(contact_fn == 0., tvm::task_dynamics::PD(0., 1.0 / dt()),
                                                   {tvm::requirements::PriorityLevel(0)});
            break;
          }
          case ContactConstraint::ContactType::Position:
          {
            // Position constraint: regular contact function and position error tracking with PD dynamics
            // Using a PD dynamics with these gains basically equates to a one-step to convergence
            data.contactConstraint_ = problem_.add(contact_fn == 0., tvm::task_dynamics::PD(1.0 / dt(), 1.0 / dt()),
                                                   {tvm::requirements::PriorityLevel(0)});
            break;
          }

          default:
            mc_rtc::log::error_and_throw("[TVMQPSolver] The geometric contact constraint type is invalid");
            break;
        }
        // Breaking out of the for loop in case there is more than one contact constraint in the solver
        break;
      }
    }

    logger_->addLogEntry(fmt::format("contact_{}::{}_{}::{}", r1.name(), f1.name(), r2.name(), f2.name()),
                         [this, storedContact]() { return desiredContactForce(*storedContact); });
    logger_->addLogEntry(fmt::format("contact_{}::{}_{}::{}", r2.name(), f2.name(), r1.name(), f1.name()),
                         [this, storedContact]() { return desiredContactForce2(*storedContact); });
    gui_->addElement(
        {"Contacts", "Forces"},
        mc_rtc::gui::Force(
            fmt::format("{}::{}/{}::{}", r1.name(), f1.name(), r2.name(), f2.name()),
            [this, storedContact]() { return desiredContactForce(*storedContact); }, [&f1]() { return f1.position(); }),
        mc_rtc::gui::Force(
            fmt::format("{}::{}/{}::{}", r2.name(), f2.name(), r1.name(), f1.name()), [this, storedContact]()
            { return desiredContactForce2(*storedContact); }, [&f2]() { return f2.position(); }));
  }
  else
  {
    // The contact function already exists, just update the contact dof
    auto contact_fn = std::static_pointer_cast<mc_tvm::ContactFunction>(data.contactConstraint_->task.function());
    contact_fn->dof(contact.dof());
  }
  return std::make_tuple(idx, hasWork);
}

void TVMQPSolver::addContact(const mc_rbdyn::Contact & contactTmp)
{
  // Add geometric contact constraint if it is not already present
  // hasWork becomes true if new contact, in this case dynamics function must be updated
  // dofs or friction changing do not influence the dynamics so does not matter here
  // WARNING: copies the contactTmp passed as argument into contact_[idx]
  // Any object storing a reference to this contact must use contact_[idx]
  const auto [idx, hasWork] = addVirtualContactImpl(contactTmp);
  // If !hasWork, then the contact already existed and is already in the dynamics
  // addVirtual function updated it if needed, just return now
  if(!hasWork) { return; }
  auto & data = contactsData_[idx];
  const auto & r1 = robot(contactTmp.r1Index());
  const auto & r2 = robot(contactTmp.r2Index());
  const auto & s1 = *contactTmp.r1Surface();
  const auto & s2 = *contactTmp.r2Surface();
  const auto & f1 = r1.frame(s1.name());
  const auto & f2 = r2.frame(s2.name());

  auto & addedContact = *contacts_[idx];

  auto addContactForce = [&addedContact, this](const std::string & robot, const mc_rbdyn::RobotFrame & frame,
                                               const std::vector<sva::PTransformd> & points,
                                               tvm::VariableVector & forces,
                                               std::vector<tvm::TaskWithRequirementsPtr> & constraints,
                                               std::vector<tvm::TaskWithRequirementsPtr> & targets, double dir)
  { addContactToDynamics(robot, frame, points, forces, constraints, targets, addedContact, dir); };

  // FIXME These points computation are a waste of time if they are not needed
  // FIXME Debug mc_rbdyn::intersection
  // auto s1Points = mc_rbdyn::intersection(s1, s2);
  auto s1Points = s1.points();
  mc_rtc::log::info("Dynamics change: adding contact force direction 1");
  addContactForce(r1.name(), f1, s1Points, data.f1_, data.f1Constraints_, data.f1Targets_, 1.0);
  std::vector<sva::PTransformd> s2Points;
  s2Points.reserve(s1Points.size());
  auto X_b2_b1 =
      r1.mbc().bodyPosW[r1.bodyIndexByName(f1.body())] * r2.mbc().bodyPosW[r2.bodyIndexByName(f2.body())].inv();
  std::transform(s1Points.begin(), s1Points.end(), std::back_inserter(s2Points),
                 [&](const auto & X_b1_p) { return X_b1_p * X_b2_b1; });
  mc_rtc::log::info("Dynamics change: adding contact force direction -1");
  addContactForce(r2.name(), f2, s2Points, data.f2_, data.f2Constraints_, data.f2Targets_, -1.0);
}

auto TVMQPSolver::removeContact(size_t idx) -> ContactIterator
{
  auto & contact = *contacts_[idx];
  auto & data = contactsData_[idx];
  const auto & r1 = robot(contact.r1Index());
  auto r1DynamicsIt = dynamics_.find(r1.name());
  if(r1DynamicsIt != dynamics_.end())
  {
    r1DynamicsIt->second->dynamicFunction().removeContact(r1.frame(contact.r1Surface()->name()));
  }
  const auto & r2 = robot(contact.r2Index());
  auto r2DynamicsIt = dynamics_.find(r2.name());
  if(r2DynamicsIt != dynamics_.end())
  {
    r2DynamicsIt->second->dynamicFunction().removeContact(r2.frame(contact.r2Surface()->name()));
  }
  for(const auto & c : data.f1Constraints_) { problem_.remove(*c); }
  for(const auto & c : data.f2Constraints_) { problem_.remove(*c); }
  for(const auto & t : data.f1Targets_) { problem_.remove(*t); }
  for(const auto & t : data.f2Targets_) { problem_.remove(*t); }
  if(data.contactConstraint_)
  {
    problem_.remove(*data.contactConstraint_);
    data.contactConstraint_.reset();
  }
  contactsData_.erase(contactsData_.begin() + static_cast<decltype(contacts_)::difference_type>(idx));

  return contacts_.erase(contacts_.begin() + static_cast<decltype(contacts_)::difference_type>(idx));
}

bool TVMQPSolver::saveGraphDotFile() const
{
  constexpr auto prefix = "mc_rtc_tvm_graph";
  auto get_path = [&]()
  {
    std::stringstream ss;
    auto t = std::time(nullptr);
    auto tm = std::localtime(&t);
    // clang-format off
    ss << prefix
       << "-" << (1900 + tm->tm_year)
       << "-" << std::setw(2) << std::setfill('0') << (1 + tm->tm_mon)
       << "-" << std::setw(2) << std::setfill('0') << tm->tm_mday
       << "-" << std::setw(2) << std::setfill('0') << tm->tm_hour
       << "-" << std::setw(2) << std::setfill('0') << tm->tm_min
       << "-" << std::setw(2) << std::setfill('0') << tm->tm_sec
       << ".dot";
    // clang-format on
    auto directory = fs::temp_directory_path();
    auto log_path = directory / fs::path(ss.str().c_str());
    return std::pair{directory, log_path};
  };

  const auto [directory, log_path] = get_path();
  bool ret = saveGraphDotFile(log_path.string());
  if(ret)
  { // Generate symlink to the latest saved graph
    std::stringstream ss_sym;
    ss_sym << prefix << "-latest.dot";
    fs::path log_sym_path = directory / fs::path(ss_sym.str().c_str());
    if(fs::is_symlink(log_sym_path)) { fs::remove(log_sym_path); }
    if(!fs::exists(log_sym_path))
    {
      std::error_code ec;
      fs::create_symlink(log_path, log_sym_path);
      if(!ec) { mc_rtc::log::info("Updated latest graph symlink: {}", log_sym_path.string()); }
      else { mc_rtc::log::warning("Failed to create latest graph symlink: {}", ec.message()); }
    }
  }
  return ret;
}

bool TVMQPSolver::saveGraphDotFile(const std::string & filename) const
{
  try
  {
    auto graphDot = tvm::graph::internal::Logger::logger().log().generateDot(&problem_.updateGraph());
    std::ofstream myfile;
    myfile.open(filename);
    myfile << graphDot << std::endl;
    myfile.close();
    mc_rtc::log::info("[TVMQPSolver] Saved dot graph to {}", filename);
    return true;
  }
  catch(std::exception & e)
  {
    mc_rtc::log::error("[TVMQPSolver] Failed to print graph: {}", e.what());
  }
  return false;
}

} // namespace mc_solver
