#include <mc_solver/CompoundJointConstraint.h>
#include <mc_solver/QPSolver.h>

#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/CompoundJointConstraintDescription.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_CompoundJointConstraint(nb::module_ & m)
{
  using mc_solver::CompoundJointConstraint;
  using mc_rbdyn::CompoundJointConstraintDescriptionVector;

  // ------------------------------------------------------------------
  // CompoundJointConstraint
  // ------------------------------------------------------------------
  nb::class_<CompoundJointConstraint>(m, "CompoundJointConstraint",
      R"(
Enforces compound joint constraints for a robot.

This constraint allows specifying relationships between multiple joints in a robot.
It is backend-aware:
- Tasks backend: uses details::CompoundJointConstraint
- TVM backend: uses details::TVMCompoundJointConstraint
)")

    .def(nb::init<const mc_rbdyn::Robots &,
                  unsigned int,
                  double>(),
         "robots"_a,
         "robotIndex"_a,
         "timeStep"_a,
         R"(
Create a CompoundJointConstraint.

Parameters
----------
robots : mc_rbdyn.Robots
robotIndex : int
timeStep : float
)")

    .def(nb::init<const mc_rbdyn::Robots &,
                  unsigned int,
                  double,
                  const CompoundJointConstraintDescriptionVector &>(),
         "robots"_a,
         "robotIndex"_a,
         "timeStep"_a,
         "descriptions"_a,
         R"(
Create a CompoundJointConstraint with initial compound joint descriptions.

Parameters
----------
descriptions : list of CompoundJointConstraintDescription
)")

    .def("addToSolver",
         &CompoundJointConstraint::addToSolverImpl,
         "solver"_a,
         R"(Add the constraint to the given QPSolver.)")
    .def("removeFromSolver",
         &CompoundJointConstraint::removeFromSolverImpl,
         "solver"_a,
         R"(Remove the constraint from the given QPSolver.)");
}

} // namespace mc_rtc_python
