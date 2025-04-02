/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/gui/Button.h>
#include <mc_solver/TVMQPSolver.h>

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
      if(gui_) { gui_->removeElement({"Contacts", "Forces"}, fmt::format("{}::{}/{}::{}", r1, r1S, r2, r2S)); }
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
  auto it1 = dynamics_.find(r1.name());
  if(it1 != dynamics_.end()) { return it1->second->dynamicFunction().contactForce(r1.frame(id.r1Surface()->name())); }
  const auto & r2 = robot(id.r2Index());
  auto it2 = dynamics_.find(r2.name());
  if(it2 != dynamics_.end()) { return it2->second->dynamicFunction().contactForce(r2.frame(id.r2Surface()->name())); }
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

  // Update force/wrench constraints if the size of the constraint changed
  for(const auto & contactTVMQP : contactsData_)
  {
    int i = 0;
    for(const auto & f1contraint : contactTVMQP.f1Constraints_)
    {
      mc_rtc::log::info("Force {} has {} planes constraints", i, f1contraint->task.function()->tSize());
      auto poly_fn = std::static_pointer_cast<mc_tvm::ForceInPolytopeFunction>(f1contraint->task.function());
      if(poly_fn->constraintSizeChanged())
      {
        // FIXME find a more automated way with a tvm event to avoid removing and re adding
        mc_rtc::log::info("Updating the solver for this task");
        problem_.remove(*f1contraint);
        problem_.add(f1contraint);
        poly_fn->constraintSizeChanged(false);
      }
      i++;
    }
    // Now also run on other side of the constraint
    for(const auto & f2contraint : contactTVMQP.f2Constraints_)
    {
      mc_rtc::log::info("Force {} has {} planes constraints", i, f2contraint->task.function()->tSize());
      auto poly_fn = std::static_pointer_cast<mc_tvm::ForceInPolytopeFunction>(f2contraint->task.function());
      if(poly_fn->constraintSizeChanged())
      {
        mc_rtc::log::info("Updating the solver for this task");
        problem_.remove(*f2contraint);
        problem_.add(f2contraint);
        poly_fn->constraintSizeChanged(false);
      }
      i++;
    }
  }
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
  // NOTE: Add polytope to contact mc_rbdyn::Contact
  // TODO: Pass polytope + offsets to addContactToDynamics
  // TODO: Reformulate friction cone constraint
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

void TVMQPSolver::addContactToDynamics(const std::string & robot,
                                       const mc_rbdyn::RobotFrame & frame,
                                       const std::vector<sva::PTransformd> & points,
                                       tvm::VariableVector & forces,
                                       std::vector<tvm::TaskWithRequirementsPtr> & constraints,
                                       std::vector<tvm::TaskWithRequirementsPtr> & targets,
                                       const mc_rbdyn::Contact & contact,
                                       double dir)
{
  auto it = dynamics_.find(robot);
  if(it == dynamics_.end()) { return; }
  if(constraints.size())
  {
    // FIXME Instead of this we should be able to change C
    for(const auto & c : constraints) { problem_.remove(*c); }
    constraints.clear();
  }
  else
  {
    it->second->removeFromSolverImpl(*this);
    auto & dyn = it->second->dynamicFunction();
    // create decision variables
    // array of decision variables for each contact
    // vec3d for each point
    // forces = dyn.addContact3d(frame, points, dir);
    forces.add(dyn.addContact6d(frame, dir));
    it->second->addToSolverImpl(*this);
  }

  // if(contact.feasiblePolytope())
  // {
  // mc_rtc::log::info("Adding polytope constraints");

  for(int i = 0; i < forces.numberOfVariables(); ++i)
  {
    auto & f = forces[i];
    auto polyFunction = std::make_shared<mc_tvm::ForceInPolytopeFunction>(contact, f, dir, robots().robotIndex(robot));
    // We want the force to stay inside of the polytope so the distance value should stay negative
    constraints.push_back(
        problem_.add(polyFunction <= 0., tvm::task_dynamics::None(), {tvm::requirements::PriorityLevel(0)}));
    // Add a minimization on the force variable with a low weight
    // TODO maybe find a way to parametrize a desired anisotropic weight from the mc_rbdyn::Contact ?
    // TODO write a linear function to target zero (minimize) or a specific target
    targets.push_back(problem_.add(f == 0.0, {tvm::requirements::PriorityLevel(1), tvm::requirements::Weight(0.0001)}));
  }

  // }
  // else
  // {
  //   const auto frictionCone = discretizedFrictionCone(contact.friction());
  //   mc_rtc::log::info("Adding simple friction cone constraints: {} constraints", frictionCone.rows());
  //   for(int i = 0; i < forces.numberOfVariables(); ++i)
  //   {
  //     auto & f = forces[i];
  //     constraints.push_back(problem_.add(dir * frictionCone * f >= 0.0, {tvm::requirements::PriorityLevel(0)}));
  //     mc_rtc::log::info("The task with requirements for force {} has a tangent space of dim {} (friction cone)", i,
  //                       constraints.back()->task.function()->tSize());
  //     mc_rtc::log::info("The corresponding jacobian for this task is\n{}",
  //                       problem_.constraint(*constraints.back().get())->jacobian(*f.get()));
  //   }
  // }
}

auto TVMQPSolver::addVirtualContactImpl(const mc_rbdyn::Contact & contact) -> std::tuple<size_t, bool>
{
  bool hasWork = false;
  auto idx = getContactIdx(contact);
  if(idx < contacts_.size())
  {
    // This contact already exists in the solver
    const auto & oldContact = *contacts_[idx];
    if(oldContact.dof() == contact.dof() && oldContact.friction() == contact.friction())
    {
      // Nothing to do unless there is a feasible polytope
      return std::make_tuple(idx, hasWork || contact.feasiblePolytope());
    }
    hasWork = contact.feasiblePolytope() || contact.friction() != oldContact.friction();
    *contacts_[idx] = contact;
  }
  else
  {
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
    data.contactConstraint_ = problem_.add(contact_fn == 0., tvm::task_dynamics::PD(1.0 / dt(), 1.0 / dt()),
                                           {tvm::requirements::PriorityLevel(0)});
    logger_->addLogEntry(fmt::format("contact_{}::{}_{}::{}", r1.name(), f1.name(), r2.name(), f2.name()),
                         [this, storedContact]() { return desiredContactForce(*storedContact); });
    gui_->addElement({"Contacts", "Forces"},
                     mc_rtc::gui::Force(
                         fmt::format("{}::{}/{}::{}", r1.name(), f1.name(), r2.name(), f2.name()),
                         [this, storedContact]() { return desiredContactForce(*storedContact); },
                         [&f1]() { return f1.position(); }));
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
  // hasWork becomes true if friction or polytope changed, in this case dynamics function must be updated
  // dofs changing do not influence the dynamics so does not matter here
  // WARNING: copies the contactTmp passed as argument into contact_[idx]
  // Any object storing a reference to this contact must use contact_[idx]
  const auto [idx, hasWork] = addVirtualContactImpl(contactTmp);
  if(!hasWork) { return; }
  auto & data = contactsData_[idx];
  const auto & r1 = robot(contactTmp.r1Index());
  const auto & r2 = robot(contactTmp.r2Index());
  const auto & s1 = *contactTmp.r1Surface();
  const auto & s2 = *contactTmp.r2Surface();
  const auto & f1 = r1.frame(s1.name());
  const auto & f2 = r2.frame(s2.name());

  const auto & addedContact = *contacts_[idx];

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
    r1DynamicsIt->second->removeFromSolverImpl(*this);
    r1DynamicsIt->second->dynamicFunction().removeContact(r1.frame(contact.r1Surface()->name()));
    r1DynamicsIt->second->addToSolverImpl(*this);
  }
  const auto & r2 = robot(contact.r2Index());
  auto r2DynamicsIt = dynamics_.find(r2.name());
  if(r2DynamicsIt != dynamics_.end())
  {
    r2DynamicsIt->second->removeFromSolverImpl(*this);
    r2DynamicsIt->second->dynamicFunction().removeContact(r2.frame(contact.r2Surface()->name()));
    r2DynamicsIt->second->addToSolverImpl(*this);
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
