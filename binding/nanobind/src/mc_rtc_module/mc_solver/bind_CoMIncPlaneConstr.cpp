#include <mc_solver/CoMIncPlaneConstr.h>
#include <mc_solver/QPSolver.h>

#include <mc_rbdyn/Robots.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/eigen/dense.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_CoMIncPlaneConstr(nb::module_ & m)
{
  using mc_solver::CoMIncPlaneConstr;

  // ------------------------------------------------------------------
  // CoMIncPlaneConstr
  // ------------------------------------------------------------------
  nb::class_<CoMIncPlaneConstr>(m, "CoMIncPlaneConstr",
      R"(
Constraint ensuring that the robot's CoM remains on the positive side of given planes.

This is backend-aware:
- Tasks backend: uses tasks::qp::CoMIncPlaneConstr
- TVM backend: uses details::TVMCoMIncPlaneConstr
)")

    // Constructor
    .def(nb::init<const mc_rbdyn::Robots &,
                  unsigned int,
                  double>(),
         "robots"_a,
         "robotIndex"_a,
         "timeStep"_a,
         R"(
Create a CoMIncPlaneConstr.

Parameters
----------
robots : mc_rbdyn.Robots
robotIndex : int
timeStep : float
)")

    // Add/remove from solver
    .def("addToSolver",
         &CoMIncPlaneConstr::addToSolverImpl,
         "solver"_a,
         R"(Add the constraint to the given QPSolver.)")
    .def("removeFromSolver",
         &CoMIncPlaneConstr::removeFromSolverImpl,
         "solver"_a,
         R"(Remove the constraint from the given QPSolver.)")

    // Active/inactive joints
    .def("setActiveJoints",
         &CoMIncPlaneConstr::setActiveJoints,
         "joints"_a,
         R"(Set the joints that participate in the constraint.)")
    .def("setInactiveJoints",
         &CoMIncPlaneConstr::setInactiveJoints,
         "joints"_a,
         R"(Set the joints that do not participate in the constraint.)")
    .def("resetActiveJoints",
         &CoMIncPlaneConstr::resetActiveJoints,
         R"(Enable all joints for the constraint.)")

    // Set planes
    .def("setPlanes",
         &CoMIncPlaneConstr::setPlanes,
         "solver"_a,
         "planes"_a = std::vector<mc_rbdyn::Plane>{},
         "speeds"_a = std::vector<Eigen::Vector3d>{},
         "normalsDots"_a = std::vector<Eigen::Vector3d>{},
         "iDist"_a = 0.05,
         "sDist"_a = 0.01,
         "damping"_a = 0.1,
         "dampingOff"_a = 0.0,
         R"(
Set the planes for the CoM constraint.

Parameters
----------
planes : list of mc_rbdyn.Plane
speeds : optional list of Vector3d
normalsDots : optional list of Vector3d
iDist : float, optional
sDist : float, optional
damping : float, optional
dampingOff : float, optional
)");
}

} // namespace mc_rtc_python
