#include <mc_solver/CollisionsConstraint.h>
#include <mc_solver/QPSolver.h>
#include <mc_rbdyn/Collision.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unordered_set.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_CollisionsConstraint(nb::module_ & m)
{
  using mc_solver::CollisionsConstraint;
  using mc_rbdyn::Collision;

  // ------------------------------------------------------------------
  // CollisionsConstraint
  // ------------------------------------------------------------------
  nb::class_<CollisionsConstraint>(m, "CollisionsConstraint",
      R"(
Manages collision constraints between two robots (or self-collision).

This constraint is backend-aware:
- Tasks backend: uses tasks::qp::CollisionConstr
- TVM backend: uses details::TVMCollisionConstraint
)")
    .def(nb::init<const mc_rbdyn::Robots &, unsigned int, unsigned int, double>(),
         "robots"_a, "r1Index"_a, "r2Index"_a, "timeStep"_a,
         R"(
Create a CollisionsConstraint.

Parameters
----------
robots : mc_rbdyn.Robots
r1Index : int
r2Index : int
timeStep : float
)")

    .def("addToSolver", &CollisionsConstraint::addToSolverImpl, "solver"_a)
    .def("removeFromSolver", &CollisionsConstraint::removeFromSolverImpl, "solver"_a)
    .def("update", &CollisionsConstraint::update, "solver"_a)

    .def("addCollision", &CollisionsConstraint::addCollision, "solver"_a, "collision"_a)
    .def("addCollisions", &CollisionsConstraint::addCollisions, "solver"_a, "collisions"_a)
    .def("removeCollision", &CollisionsConstraint::removeCollision, "solver"_a, "b1Name"_a, "b2Name"_a)
    .def("removeCollisions", &CollisionsConstraint::removeCollisions, "solver"_a, "collisions"_a)
    .def("removeCollisionByBody", &CollisionsConstraint::removeCollisionByBody, "solver"_a, "b1Name"_a, "b2Name"_a)
    .def("hasCollision", &CollisionsConstraint::hasCollision, "c1"_a, "c2"_a)
    .def("reset", &CollisionsConstraint::reset)

    .def_prop_rw("automaticMonitor",
                  [](CollisionsConstraint & cc) {return cc.automaticMonitor();},
                  [](CollisionsConstraint & cc, bool a) {cc.automaticMonitor(a);},
                  R"(Get/set automatic collision monitoring.

If True, collisions are automatically monitored in the GUI.
If False, the user must manage monitoring manually.)")

    .def_rw("cols", &CollisionsConstraint::cols,
                   R"(Current set of collisions (list of mc_rbdyn.Collision).)");

}

} // namespace mc_rtc_python
