#include <mc_tasks/PostureTask.h>
#include <mc_solver/QPSolver.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/trampoline.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

// --------------------------------------------
// Trampoline class: exposes virtual methods
// --------------------------------------------
struct PyPostureTask : mc_tasks::PostureTask
{
  NB_TRAMPOLINE(mc_tasks::PostureTask, 3);

  void reset() override { NB_OVERRIDE_PURE(reset); }
  void dimWeight(const Eigen::VectorXd & w) override { NB_OVERRIDE_PURE(dimWeight, w); }
  Eigen::VectorXd dimWeight() const override { NB_OVERRIDE_PURE(dimWeight); }

  Eigen::VectorXd eval() const override { NB_OVERRIDE(eval); }
  Eigen::VectorXd speed() const override { NB_OVERRIDE(speed); }

  void selectActiveJoints(mc_solver::QPSolver & solver,
                          const std::vector<std::string> & names,
                          const std::map<std::string, std::vector<std::array<int, 2>>> & dofs) override
  {
    NB_OVERRIDE(selectActiveJoints, solver, names, dofs);
  }

  void selectUnactiveJoints(mc_solver::QPSolver & solver,
                            const std::vector<std::string> & names,
                            const std::map<std::string, std::vector<std::array<int, 2>>> & dofs) override
  {
    NB_OVERRIDE(selectUnactiveJoints, solver, names, dofs);
  }

  void resetJointsSelector(mc_solver::QPSolver & solver) override
  {
    NB_OVERRIDE(resetJointsSelector, solver);
  }

  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & c) override
  {
    NB_OVERRIDE(load, solver, c);
  }

protected:
  void addToSolver(mc_solver::QPSolver & solver) override { NB_OVERRIDE(addToSolver, solver); }
  void removeFromSolver(mc_solver::QPSolver & solver) override { NB_OVERRIDE(removeFromSolver, solver); }
  void update(mc_solver::QPSolver & solver) override { NB_OVERRIDE(update, solver); }
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override { NB_OVERRIDE(addToGUI, gui); }
  void addToLogger(mc_rtc::Logger & logger) override { NB_OVERRIDE(addToLogger, logger); }
};

// --------------------------------------------
// Binding code
// --------------------------------------------
void bind_PostureTask(nb::module_ & m)
{
  using mc_tasks::PostureTask;

  auto cls = nb::class_<PostureTask, PyPostureTask>(m, "PostureTask");
  
  cls.doc() = R"(
A posture task for a given robot.

Note:
* In Tasks backend: dimension = robot.mb().nrParams()
* In TVM backend:   dimension = robot.tvmRobot().qJoints().size()
)";

  // Constructors
  cls.def(nb::init<const mc_solver::QPSolver &, unsigned int, double, double>(),
          "solver"_a, "rIndex"_a, "stiffness"_a=1.0, "weight"_a=10.0);

  // --------------------------------------------------
  // Virtual methods
  // --------------------------------------------------
  cls.def("reset", &PostureTask::reset);

  cls.def("dimWeight",
          nb::overload_cast<const Eigen::VectorXd &>(&PostureTask::dimWeight),
          "weights"_a);
  cls.def("dimWeight",
          nb::overload_cast<>(&PostureTask::dimWeight, nb::const_));

  cls.def("eval", &PostureTask::eval);
  cls.def("speed", &PostureTask::speed);

  cls.def("selectActiveJoints", &PostureTask::selectActiveJoints,
          "solver"_a, "active_joints"_a, "active_dofs"_a = std::map<std::string, std::vector<std::array<int, 2>>>{});

  cls.def("selectUnactiveJoints", &PostureTask::selectUnactiveJoints,
          "solver"_a, "inactive_joints"_a, "inactive_dofs"_a = std::map<std::string, std::vector<std::array<int, 2>>>{});

  cls.def("resetJointsSelector", &PostureTask::resetJointsSelector,
          "solver"_a);

  cls.def("load", &PostureTask::load, "solver"_a, "config"_a);

  // --------------------------------------------------
  // Posture and reference commands
  // --------------------------------------------------
  cls.def("posture", nb::overload_cast<const std::vector<std::vector<double>> &>(&PostureTask::posture),
          "target_posture"_a);

  cls.def("posture", nb::overload_cast<>(&PostureTask::posture, nb::const_));

  cls.def("refVel", nb::overload_cast<const Eigen::VectorXd &>(&PostureTask::refVel),
          "ref_velocity"_a);
  cls.def("refVel", nb::overload_cast<>(&PostureTask::refVel, nb::const_));

  cls.def("refAccel", nb::overload_cast<const Eigen::VectorXd &>(&PostureTask::refAccel),
          "ref_accel"_a);
  cls.def("refAccel", nb::overload_cast<>(&PostureTask::refAccel, nb::const_));

  // Joint gains / stiffness / weights
  cls.def("jointGains", &PostureTask::jointGains, "solver"_a, "gains"_a);
  cls.def("jointStiffness", &PostureTask::jointStiffness, "solver"_a, "stiffness"_a);
  cls.def("jointWeights", &PostureTask::jointWeights, "weights"_a);

  // Joint targets
  cls.def("target", &PostureTask::target, "joints"_a);

  // Stiffness / damping / weight
  cls.def("stiffness", nb::overload_cast<>(&PostureTask::stiffness, nb::const_));
  cls.def("stiffness", nb::overload_cast<double>(&PostureTask::stiffness));

  cls.def("damping", nb::overload_cast<>(&PostureTask::damping, nb::const_));
  cls.def("damping", nb::overload_cast<double>(&PostureTask::damping));

  cls.def("setGains", &PostureTask::setGains, "stiffness"_a, "damping"_a);

  cls.def("weight", nb::overload_cast<>(&PostureTask::weight, nb::const_));
  cls.def("weight", nb::overload_cast<double>(&PostureTask::weight));

  cls.def("inSolver", &PostureTask::inSolver);
}


}