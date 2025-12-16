#include <mc_solver/ContactConstraint.h>
#include <mc_solver/QPSolver.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_ContactConstraint(nb::module_ & m)
{
  using mc_solver::ContactConstraint;

  // ContactType enum
  nb::enum_<ContactConstraint::ContactType>(m, "ContactType")
    .value("Acceleration", ContactConstraint::ContactType::Acceleration)
    .value("Velocity", ContactConstraint::ContactType::Velocity)
    .value("Position", ContactConstraint::ContactType::Position);

  // ------------------------------------------------------------------
  // ContactConstraint
  // ------------------------------------------------------------------
  nb::class_<ContactConstraint>(m, "ContactConstraint",
      R"(
Handle geometric constraints on contacts.

This constraint enforces contact constraints on a robot, depending on the backend.
- Tasks backend: tasks::qp::ContactConstr  
- TVM backend: no-op
)")
    // Constructor
    .def(nb::init<double,
                  ContactConstraint::ContactType>(),
         "timeStep"_a,
         "contactType"_a = ContactConstraint::ContactType::Velocity,
         R"(
Create a ContactConstraint.

Parameters
----------
timeStep : float
    Solver timestep.
contactType : ContactConstraint.ContactType, optional
    Type of geometric constraint (Acceleration, Velocity, Position).
)")

    .def("addToSolver",
         &ContactConstraint::addToSolverImpl,
         "solver"_a,
         R"(Add the constraint to the given QPSolver.)")
    .def("removeFromSolver",
         &ContactConstraint::removeFromSolverImpl,
         "solver"_a,
         R"(Remove the constraint from the given QPSolver.)")

    .def("contactConstr",
         &ContactConstraint::contactConstr,
         R"(
Return the underlying tasks::qp::ContactConstr.

Raises an error if the backend is not Tasks.
)");
}

} // namespace mc_rtc_python
