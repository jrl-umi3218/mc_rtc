#include <mc_tasks/TrajectoryTaskGeneric.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

void bind_TrajectoryTaskGeneric(nb::module_ & m)
{
  using mc_tasks::TrajectoryTaskGeneric;

  nb::class_<TrajectoryTaskGeneric, mc_tasks::MetaTask>(m, "TrajectoryTaskGeneric")
      .def(nb::init<const mc_rbdyn::Robots &, unsigned int, double, double>(), "robots"_a, "robotIndex"_a,
           "stiffness"_a, "weight"_a)
      .def(nb::init<const mc_rbdyn::RobotFrame &, double, double>(), "frame"_a, "stiffness"_a, "weight"_a)

      .def("reset", &TrajectoryTaskGeneric::reset)

      .def("refVel", nb::overload_cast<const Eigen::VectorXd &>(&TrajectoryTaskGeneric::refVel), "vel"_a)
      .def("refVel", nb::overload_cast<>(&TrajectoryTaskGeneric::refVel, nb::const_))
      .def("refAccel", nb::overload_cast<const Eigen::VectorXd &>(&TrajectoryTaskGeneric::refAccel), "accel"_a)
      .def("refAccel", nb::overload_cast<>(&TrajectoryTaskGeneric::refAccel, nb::const_))

      .def("stiffness", nb::overload_cast<double>(&TrajectoryTaskGeneric::stiffness), "stiffness"_a)
      .def("stiffness", nb::overload_cast<const Eigen::VectorXd &>(&TrajectoryTaskGeneric::stiffness), "stiffness"_a)
      .def("stiffness", nb::overload_cast<>(&TrajectoryTaskGeneric::stiffness, nb::const_))

      .def("damping", nb::overload_cast<double>(&TrajectoryTaskGeneric::damping), "damping"_a)
      .def("damping", nb::overload_cast<const Eigen::VectorXd &>(&TrajectoryTaskGeneric::damping), "damping"_a)
      .def("damping", nb::overload_cast<>(&TrajectoryTaskGeneric::damping, nb::const_))

      .def("setGains", nb::overload_cast<double, double>(&TrajectoryTaskGeneric::setGains), "stiffness"_a, "damping"_a)
      .def("setGains",
           nb::overload_cast<const Eigen::VectorXd &, const Eigen::VectorXd &>(&TrajectoryTaskGeneric::setGains),
           "stiffness"_a, "damping"_a)

      .def("weight", nb::overload_cast<double>(&TrajectoryTaskGeneric::weight), "w"_a)
      .def("weight", nb::overload_cast<>(&TrajectoryTaskGeneric::weight, nb::const_))

      .def("selectActiveJoints",
           nb::overload_cast<const std::vector<std::string> &,
                             const std::map<std::string, std::vector<std::array<int, 2>>> &, bool>(
               &TrajectoryTaskGeneric::selectActiveJoints),
           "activeJointsName"_a, "activeDofs"_a = std::map<std::string, std::vector<std::array<int, 2>>>{},
           "checkJoints"_a = true)

      .def("eval", &TrajectoryTaskGeneric::eval)
      .def("speed", &TrajectoryTaskGeneric::speed);
}

} // namespace mc_rtc_python