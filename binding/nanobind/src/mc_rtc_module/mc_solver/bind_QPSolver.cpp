#include <mc_control/MCController.h>
#include <mc_solver/ConstraintSet.h>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_solver/QPSolver.h>

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/Robots.h>

#include <mc_tasks/MetaTask.h>

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

/**
 * Trampoline class to handle virtual functions in python
 * See: https://nanobind.readthedocs.io/en/latest/classes.html#trampolines
 **/
struct PyQPSolver : mc_solver::QPSolver
{
  NB_TRAMPOLINE(mc_solver::QPSolver, 3);

  void setContacts(mc_solver::QPSolver::ControllerToken, const std::vector<mc_rbdyn::Contact> & contacts) override
  {
    NB_OVERRIDE_PURE(setContacts, contacts);
  }

  const sva::ForceVecd desiredContactForce(const mc_rbdyn::Contact & c) const override
  {
    NB_OVERRIDE_PURE(desiredContactForce, c);
  }

  bool run_impl(mc_solver::FeedbackType fType) override { NB_OVERRIDE_PURE(run_impl, fType); }

  double solveTime() override { NB_OVERRIDE_PURE(solveTime); }

  double solveAndBuildTime() override { NB_OVERRIDE_PURE(solveAndBuildTime); }

  void addDynamicsConstraint(mc_solver::DynamicsConstraint * dyn) override
  {
    NB_OVERRIDE_PURE(addDynamicsConstraint, dyn);
  }

  void removeDynamicsConstraint(mc_solver::ConstraintSet * c) override
  {
    NB_OVERRIDE_PURE(removeDynamicsConstraint, c);
  }
};

// ============================================================================
//  Binding
// ============================================================================
void bind_QPSolver(nb::module_ & m)
{
  using mc_solver::QPSolver;

  // ================================
  // Enums
  // ================================
  nb::enum_<QPSolver::Backend>(m, "Backend",
                               R"(
Backend selection for QPSolver and tasks/constraints.

• Unset – backend not selected  
• Tasks – use mc_rtc Tasks backend  
• TVM – use the TVM backend  
)")
      .value("Unset", QPSolver::Backend::Unset)
      .value("Tasks", QPSolver::Backend::Tasks)
      .value("TVM", QPSolver::Backend::TVM);

  nb::enum_<mc_solver::FeedbackType>(m, "FeedbackType",
                                     R"(
Type of feedback used to control the robot:

• None_ / OpenLoop – no feedback  
• Joints – encoder feedback  
• JointsWVelocity – encoder + differentiated velocity  
• ObservedRobots / ClosedLoop – close the loop from observed state  
• ClosedLoopIntegrateReal – integrate real robot state  
)")
      .value("None_", mc_solver::FeedbackType::None)
      .value("OpenLoop", mc_solver::FeedbackType::OpenLoop)
      .value("Joints", mc_solver::FeedbackType::Joints)
      .value("JointsWVelocity", mc_solver::FeedbackType::JointsWVelocity)
      .value("ObservedRobots", mc_solver::FeedbackType::ObservedRobots)
      .value("ClosedLoop", mc_solver::FeedbackType::ClosedLoop)
      .value("ClosedLoopIntegrateReal", mc_solver::FeedbackType::ClosedLoopIntegrateReal);

  auto cls = nb::class_<QPSolver, PyQPSolver>(m, "QPSolver");

  cls.doc() =
      R"(
Main quadratic programming solver in mc_rtc.

It manages:
• Constraint sets  
• MetaTasks  
• Contact sets  
• Logger and GUI  
• Robot states and real-robot synchronization  
)";


  cls.def(nb::init<mc_rbdyn::RobotsPtr, double, QPSolver::Backend>(), "robots"_a, "timeStep"_a,
          "backend"_a = QPSolver::Backend::Tasks,
          R"(
Constructor with provided robots.

• robots: shared pointer to mc_rbdyn::Robots  
• timeStep: control timestep in seconds  
• backend: solver backend  
)");

  cls.def(nb::init<double, QPSolver::Backend>(), "timeStep"_a, "backend"_a = QPSolver::Backend::Tasks,
          R"(
Constructor where the solver creates its own Robots.

• timeStep: control timestep in seconds  
• backend: solver backend  
)");

  cls.def("backend", &QPSolver::backend, "Returns the backend for this solver instance");


  cls.def_static("context_backend", nb::overload_cast<>(&QPSolver::context_backend),
                 "Get the current backend context used when creating tasks/constraints "
                 "without an explicit QPSolver pointer.");

  cls.def_static("set_context_backend", nb::overload_cast<QPSolver::Backend>(&QPSolver::context_backend), "backend"_a,
                 "Set backend to be used as context for task/constraint creation.");
  // --------------------------------
  //  Constraint set management
  // --------------------------------
  cls.def("addConstraintSet", static_cast<void (mc_solver::QPSolver::*)(mc_solver::ConstraintSet &)>(
                                  &mc_solver::QPSolver::addConstraintSet));

  cls.def("removeConstraintSet", static_cast<void (mc_solver::QPSolver::*)(mc_solver::ConstraintSet &)>(
                                     &mc_solver::QPSolver::removeConstraintSet));

  // --------------------------------
  //  Task management
  // --------------------------------
  cls.def("addTask", nb::overload_cast<mc_tasks::MetaTask *>(&QPSolver::addTask), "task"_a,
          R"(
Add a MetaTask* to the solver.

Does not take ownership.
)");

  cls.def("addTask", nb::overload_cast<std::shared_ptr<mc_tasks::MetaTask>>(&QPSolver::addTask), "task"_a,
          R"(
Add a MetaTask using shared ownership.
)");

  cls.def("removeTask", nb::overload_cast<mc_tasks::MetaTask *>(&QPSolver::removeTask), "task"_a,
          "Remove a MetaTask* from the solver.");

  cls.def("removeTask", nb::overload_cast<std::shared_ptr<mc_tasks::MetaTask>>(&QPSolver::removeTask), "task"_a,
          "Remove a shared MetaTask from the solver.");

  // --------------------------------
  // Contacts
  // --------------------------------
  cls.def("setContacts", nb::overload_cast<const std::vector<mc_rbdyn::Contact> &>(&QPSolver::setContacts),
          "contacts"_a = std::vector<mc_rbdyn::Contact>{},
          R"(
Set contacts on the solver (controller-agnostic version).
)");

  cls.def("contacts", &QPSolver::contacts, nb::rv_policy::reference_internal, "Get current contact list.");

  // Pure virtual (via trampoline)
  cls.def("desiredContactForce", &QPSolver::desiredContactForce, "contact"_a);

  // --------------------------------
  // Run iteration
  // --------------------------------
  cls.def("run", &QPSolver::run, "feedback"_a = mc_solver::FeedbackType::None,
          R"(
Run one iteration of the QP.

• Updates robot configurations  
• Returns True if optimization succeeded  
)");

  // --------------------------------
  // Robot accessors
  // --------------------------------
  cls.def("robot", nb::overload_cast<>(&QPSolver::robot), nb::rv_policy::reference_internal);
  cls.def("robot", nb::overload_cast<unsigned int>(&QPSolver::robot), "index"_a, nb::rv_policy::reference_internal);

  cls.def("env", nb::overload_cast<>(&QPSolver::env), nb::rv_policy::reference_internal);

  cls.def("robots", nb::overload_cast<>(&QPSolver::robots), nb::rv_policy::reference_internal);

  cls.def("realRobots", nb::overload_cast<>(&QPSolver::realRobots), nb::rv_policy::reference_internal);

  // --------------------------------
  // Timing
  // --------------------------------
  cls.def("dt", &QPSolver::dt, "Solver timestep in seconds.");

  cls.def("solveTime", &QPSolver::solveTime, "Return solving time in milliseconds.");

  cls.def("solveAndBuildTime", &QPSolver::solveAndBuildTime, "Return building + solving time in milliseconds.");

  // --------------------------------
  // Logger + GUI
  // --------------------------------
  cls.def("logger", nb::overload_cast<std::shared_ptr<mc_rtc::Logger>>(&QPSolver::logger), "logger"_a);

  cls.def("logger", nb::overload_cast<>(&QPSolver::logger, nb::const_));

  cls.def("gui", nb::overload_cast<std::shared_ptr<mc_rtc::gui::StateBuilder>>(&QPSolver::gui), "gui"_a);

  cls.def("gui", nb::overload_cast<>(&QPSolver::gui, nb::const_));

  // --------------------------------
  // Controller access
  // --------------------------------
  cls.def("controller", static_cast<void (QPSolver::*)(mc_control::MCController *)>(&QPSolver::controller), "ctl"_a, R"(
Set the controller instance.

Parameters
----------
ctl : mc_control.MCController
    Controller object.
)");

  // MCController * controller()      (non-const)
  cls.def("controller", static_cast<mc_control::MCController * (QPSolver::*)()>(&QPSolver::controller),
          nb::rv_policy::reference_internal);

  // const MCController * controller() const
  cls.def("controller", static_cast<const mc_control::MCController * (QPSolver::*)() const>(&QPSolver::controller),
          nb::rv_policy::reference_internal);
}

} // namespace mc_rtc_python
