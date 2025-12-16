#include <mc_solver/BoundedSpeedConstr.h>
#include <mc_solver/QPSolver.h>
#include <mc_rbdyn/RobotFrame.h>

#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_BoundedSpeedConstr(nb::module_ & m)
{
  using mc_solver::BoundedSpeedConstr;
  using mc_solver::QPSolver;
  using mc_rbdyn::RobotFrame;

  // ------------------------------------------------------------------
  // BoundedSpeedConstr
  // ------------------------------------------------------------------
  nb::class_<BoundedSpeedConstr>(m, "BoundedSpeedConstr",
      R"(
Constraint that manages bounded speeds of a given frame or body.

Backend-aware:
- Tasks: tasks::qp::BoundedSpeedConstr
- TVM: details::TVMBoundedSpeedConstr
)")

    // Constructor
    .def(nb::init<const mc_rbdyn::Robots &, unsigned int, double>(),
         "robots"_a, "robotIndex"_a, "timeStep"_a)

    .def("addToSolver", &BoundedSpeedConstr::addToSolverImpl, "solver"_a)
    .def("removeFromSolver", &BoundedSpeedConstr::removeFromSolverImpl, "solver"_a)

    .def("addBoundedSpeed",
         nb::overload_cast<QPSolver &, const std::string &, const Eigen::Vector3d &, const Eigen::MatrixXd &, const Eigen::VectorXd &>(&BoundedSpeedConstr::addBoundedSpeed),
         "solver"_a, "bodyName"_a, "bodyPoint"_a, "dof"_a, "speed"_a)
    .def("addBoundedSpeed",
         nb::overload_cast<QPSolver &, const std::string &, const Eigen::Vector3d &, const Eigen::MatrixXd &, const Eigen::VectorXd &, const Eigen::VectorXd &>(&BoundedSpeedConstr::addBoundedSpeed),
         "solver"_a, "bodyName"_a, "bodyPoint"_a, "dof"_a, "lowerSpeed"_a, "upperSpeed"_a)

    .def("addBoundedSpeed",
         nb::overload_cast<QPSolver &, const RobotFrame &, const Eigen::MatrixXd &, const Eigen::VectorXd &>(&BoundedSpeedConstr::addBoundedSpeed),
         "solver"_a, "frame"_a, "dof"_a, "speed"_a)
    .def("addBoundedSpeed",
         nb::overload_cast<QPSolver &, const RobotFrame &, const Eigen::MatrixXd &, const Eigen::VectorXd &, const Eigen::VectorXd &>(&BoundedSpeedConstr::addBoundedSpeed),
         "solver"_a, "frame"_a, "dof"_a, "lowerSpeed"_a, "upperSpeed"_a)

    .def("removeBoundedSpeed", &BoundedSpeedConstr::removeBoundedSpeed, "solver"_a, "frameName"_a)
    .def("reset", &BoundedSpeedConstr::reset, "solver"_a)

    .def("nrBoundedSpeeds", &BoundedSpeedConstr::nrBoundedSpeeds);
}

} // namespace mc_rtc_python
