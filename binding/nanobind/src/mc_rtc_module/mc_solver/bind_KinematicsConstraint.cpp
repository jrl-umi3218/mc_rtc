#include <mc_solver/KinematicsConstraint.h>
#include <mc_solver/TVMKinematicsConstraint.h>
#include <mc_solver/TVMQPSolver.h>

#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/Robot.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{
  
void bind_KinematicsConstraint(nb::module_ & m)
{
  using mc_solver::KinematicsConstraint;
  using mc_solver::TVMKinematicsConstraint;

  // ------------------------------------------------------------------
  // TVMKinematicsConstraint 
  // ------------------------------------------------------------------
  nb::class_<TVMKinematicsConstraint>(m, "TVMKinematicsConstraint",
    R"(
TVM implementation of the kinematics constraint.

This constraint enforces:
• joint limits (with damper safety)  
• velocity limits  
• acceleration limits  
• mimic-joint coupling  
    )")
    .def(nb::init<const mc_rbdyn::Robot &,
                  const std::array<double,3> &,
                  double>(),
         "robot"_a,
         "damper"_a,
         "velocityPercent"_a = 1.0,
         R"(
Create a TVM kinematics constraint.

Parameters
----------
robot : mc_rbdyn.Robot
damper : array[3]
    • damper[0] = intermediate damping  
    • damper[1] = safety damping  
    • damper[2] = offset  
velocityPercent : float
    Scaling applied to velocity limits
)")
    .def("addToSolver",
         &TVMKinematicsConstraint::addToSolver,
         "solver"_a,
         "Add the constraint to the given TVMQPSolver")
    .def("removeFromSolver",
         &TVMKinematicsConstraint::removeFromSolver,
         "solver"_a,
         "Remove the constraint from the given TVMQPSolver");


  // ------------------------------------------------------------------
  // KinematicsConstraint
  // ------------------------------------------------------------------
  nb::class_<KinematicsConstraint>(
      m, "KinematicsConstraint",
      R"(
High-level kinematic limits constraint (joint pos/vel/acc and mimic joints).

This class dispatches internally to either:
• tasks backend (TasksQPSolver)  
• tvm backend (TVMQPSolver)

depending on QPSolver::Backend.
)")
    .def(nb::init<const mc_rbdyn::Robots &,
                  unsigned int,
                  double>(),
         "robots"_a,
         "robotIndex"_a,
         "timeStep"_a,
         R"(
Create a KinematicsConstraint.

Uses default damper = {0.1, 0.01, 0.5}, velocityPercent = 0.5.
)")

    // Constructor: explicit damper + velocity percent
    .def(nb::init<const mc_rbdyn::Robots &,
                  unsigned int,
                  double,
                  const std::array<double,3> &,
                  double>(),
         "robots"_a,
         "robotIndex"_a,
         "timeStep"_a,
         "damper"_a,
         "velocityPercent"_a,
         R"(
Create a KinematicsConstraint with custom parameters.

Parameters
----------
damper : array[3]
velocityPercent : float
)");
}

} // namespace mc_rtc_python
