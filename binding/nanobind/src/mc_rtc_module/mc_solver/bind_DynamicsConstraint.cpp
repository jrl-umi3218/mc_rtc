#include <mc_solver/DynamicsConstraint.h>
#include <mc_solver/QPSolver.h>

#include <mc_rbdyn/Robots.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_DynamicsConstraint(nb::module_ & m)
{
  using mc_solver::DynamicsConstraint;

  // ------------------------------------------------------------------
  // DynamicsConstraint
  // ------------------------------------------------------------------
  nb::class_<DynamicsConstraint>(m, "DynamicsConstraint",
      R"(
Holds dynamics constraints (equation of motion) for a robot.

This constraint enforces:
• joint limits (possibly damped)  
• velocity limits  
• torque limits (optional)  
• robot-specific dynamics through motion constraints
)")
    .def(nb::init<const mc_rbdyn::Robots &,
                  unsigned int,
                  double,
                  bool>(),
         "robots"_a,
         "robotIndex"_a,
         "timeStep"_a,
         "infTorque"_a = false,
         R"(
Create a DynamicsConstraint.

Parameters
----------
robots : mc_rbdyn.Robots
    Robots including the robot affected by this constraint.
robotIndex : int
    Index of the robot affected by this constraint.
timeStep : float
    Solver timestep.
infTorque : bool, optional
    If true, ignore torque limits in the robot model.
)")

    .def(nb::init<const mc_rbdyn::Robots &,
                  unsigned int,
                  double,
                  const std::array<double, 3> &,
                  double,
                  bool>(),
         "robots"_a,
         "robotIndex"_a,
         "timeStep"_a,
         "damper"_a,
         "velocityPercent"_a = 1.0,
         "infTorque"_a = false,
         R"(
Create a DynamicsConstraint with damping.

Parameters
----------
damper : array[3]
    • damper[0] = interaction distance  
    • damper[1] = safety distance  
    • damper[2] = offset  
velocityPercent : float
    Maximum joint velocity percentage (e.g., 0.5).  
infTorque : bool
    If true, ignore torque limits in the robot model.
)")

    .def_prop_ro("robotIndex",
                           &DynamicsConstraint::robotIndex,
                           R"(Index of the robot affected by this constraint.)")

    .def("motionConstr",
         &DynamicsConstraint::motionConstr,
         R"(
Return the tasks::qp::MotionConstr.

This assumes the backend was Tasks.
)")

    .def("dynamicFunction",
         &DynamicsConstraint::dynamicFunction,
         R"(
Return the mc_tvm::DynamicFunction.

This assumes the backend was TVM.
)")

    .def("addToSolver",
         &DynamicsConstraint::addToSolverImpl,
         "solver"_a,
         R"(Add the constraint to the given QPSolver.)")
    .def("removeFromSolver",
         &DynamicsConstraint::removeFromSolverImpl,
         "solver"_a,
         R"(Remove the constraint from the given QPSolver.)");
}

} // namespace mc_rtc_python
