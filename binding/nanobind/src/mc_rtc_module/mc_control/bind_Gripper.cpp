#include <mc_control/generic_gripper.h>
#include <mc_rbdyn/Robot.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

namespace nb = nanobind;
using namespace nb::literals;
using Gripper = mc_control::Gripper;

namespace mc_rtc_python
{

void bind_Gripper(nb::module_ & m)
{
  auto g = nb::class_<Gripper>(m, "Gripper");

  g.doc() = R"(
A robot's gripper reprensentation

A gripper is composed of a set of joints that we want to control only
through "manual" operations. It may include passive joints.

By default, a gripper is considered "open" when its joints' values
are at maximum value and "closed" at minimum value. This behaviour
can be reversed when the gripper is created.

In real operations the actuated joints are also monitored to avoid
potential servo errors.
  )";

  g.def(nb::init<const mc_rbdyn::Robot &,
          const std::vector<std::string> &,
          const std::string &,
          bool,
          const mc_rbdyn::RobotModule::Gripper::Safety &>(),
            "robot"_a, "jointNames"_a, "robot_urdf"_a, "reverseLimits"_a, "safety"_a,
            R"(
   :param robot: The full robot including uncontrolled joints
   :param jointNames: Name of the active joints involved in the gripper
   :param robot_urdf: URDF of the robot
   :param reverseLimits: If set to true, then the gripper is considered "open" when the joints' values are minimal
   :param safety: Default gripper safety parameters
            )")
  .def(nb::init<const mc_rbdyn::Robot & ,
          const std::vector<std::string> & ,
          const std::vector<mc_rbdyn::Mimic> & ,
          bool ,
          const mc_rbdyn::RobotModule::Gripper::Safety &>(),
          "robot"_a, "jointNames"_a, "mimics"_a, "reverseLimits"_a, "safety"_a,
          R"(
   This constructor does not use information from the URDF file

   :param robot: Robot, must have the active joints of the gripper to work properly
   :param jointNames: Name of the active joints involved in the gripper
   :param mimics: Mimic joints for the gripper
   :param reverseLimits: If true, the gripper is considered "open" when the joints values are minimal
   :param safety: Default gripper safety parameters
          )");

  g.def("resetDefaults", &Gripper::resetDefaults, "Resets the gripper parameters to their default value (percentVMax, actualCommandDiffTrigger)")
  .def("saveConfig", &Gripper::saveConfig, "Saves the current gripper configuration parameters contained in Config")
  .def("restoreConfig", &Gripper::restoreConfig, R"(
  Restores the gripper configuration parameters from their saved value

  See :py:func:`saveConfig`
  )")
  .def("configure", &Gripper::configure, "config"_a, "Applies a new gripper configuration (safeties and targets)");

  g.def("reset", nb::overload_cast<const std::vector<double> &>(&Gripper::reset), "currentQ"_a, R"(
  Reset the gripper state to the current actual state of the gripper

  :param currentQ: Current encoder values for the robot
  )")
  .def("reset", nb::overload_cast<const Gripper &>(&Gripper::reset), "gripper"_a, R"(
  Reset from another gripper

  :param gripper: Gripper used to reset this one

  )");

  g.def("run", &Gripper::run, "timeStep"_a, "robot"_a, "real"_a,
      R"(
  Run one iteration of control

  :param robot: Robot for which this gripper control is running

  :param real: Real robot for which this gripper control is running

  The gripper control updates both the robot's configuration and the output
      )");

  g.def("setTargetQ",
      nb::overload_cast<const std::vector<double> &>(&Gripper::setTargetQ), "targetQ"_a,
      R"(
  Set the target configuration of the active joints involved in the gripper

  :param targetQ: Desired values of the active joints involved in the gripper
  :throws: If the targetQ size does not match the number of active joints
      )")
.def("setTargetQ",
      nb::overload_cast<const std::string &, double>(&Gripper::setTargetQ), "jointName"_a, "targetQ"_a,
      R"(
  Set the target configuration of the specified active joint

  :param jointName: Name of the gripper's active joint to move
  :param targetQ: Desired value of the active joint
  :throw: if the joint name does not match any of the gripper's active joints
      )");

g.def("getTargetQ",
    [](Gripper & self, const std::string & jointName)
    {
      return self.getTargetQ(jointName);
    },
    "jointName"_a, R"(
   Get a joint's target angle

  :throws: if the joint name does not match any of the
  gripper's active joints
    )")
.def("getTargetQ",
    [](Gripper & self)
    {
      return self.getTargetQ();
    },
    R"(
   :returns: the current gripper's target. Note: returns the current gripper position if no target has been set.)");

g.def("setTargetOpening", nb::overload_cast<double>(&Gripper::setTargetOpening),
      "targetOpening"_a,
      R"(
  Set the target opening of all gripper joints simultaneously

  :param targetOpening: Opening value ranging from 0 (closed) to 1 (open)

  :note: If the individual joint targets were set manually, they will move to match this new opening target. Depending on the maximum allowed velocity, this may result in fast gripper joint motions. Use with care if the gripper is currently grasping objects or close to collisions with the environment.
      )")
    .def("setTargetOpening", nb::overload_cast<const std::string &, double>(&Gripper::setTargetOpening), "jointName"_a, "targetOpening"_a,
        R"(
  Set the target opening of a single gripper joint

  :param jointName: Name of the active joint to move
  :param targetOpening: Opening value ranging from 0 (closed) to 1 (open)
  :throws: if the joint name does not match any of the gripper's active joints
        )");

    g.def("getTargetOpening", &Gripper::getTargetOpening, "jointName"_a,
        R"(
  Get the target opening of a single gripper joint

  :param jointName: Name of the active joint
  :throws: if the joint name does not match any of the gripper's active joints
  :returns: The joint's target opening percentage
        )");

    g.def("curPosition", nb::overload_cast<>(&Gripper::curPosition, nb::const_), R"(
  Get current configuration

  :returns: Current values of the active joints involved in the gripper
    )")
    .def("curOpening", nb::overload_cast<>(&Gripper::curOpening, nb::const_), R"(
  Get current opening percentage

  :returns: Current opening percentage of the active joints involved in the gripper
    )")
    .def("curOpening", nb::overload_cast<const std::string &>(&Gripper::curOpening, nb::const_), "jointName"_a,
        R"(
  Get the current opening of a single gripper joint

  :param jointName: Name of the active joint
  :throw: if the joint name does not match any of the
  gripper's active joints
  :returns: The joint's current opening percentage
        )");

    g.def("joints", &Gripper::joints, "Returns all joints involved in the gripper")
    .def("activeJoints", &Gripper::activeJoints, ":returns: all active joints involved in the gripper")
    .def("hasActiveJoint", &Gripper::hasActiveJoint, "jointName"_a, "Checks whether a joint is an active gripper joint");

  g.def("q", &Gripper::q, R"(
  Return all gripper joints configuration

  :returns: Current values of all the gripper's joints, including passive joints
  )")
  .def("opening", &Gripper::opening, R"(
  Get the current opening percentage

  :note: Returns an average of the current opening percentage of each joint

  :returns: Current opening percentage
  )");

  g
  .def_prop_rw("percentVMAX",
      [](Gripper & self) { self.percentVMAX(); },
      [](Gripper & self, double percent) { self.percentVMAX(percent); },
      "percent"_a, "gripper speed as a percentage of maximum velocity")
  .def_prop_rw("actualCommandDiffTrigger",
      [](Gripper & self) { self.actualCommandDiffTrigger(); },
      [](Gripper & self, double d) { self.actualCommandDiffTrigger(d); },
      "commandDiffTrigger"_a, R"(
  Safety trigger threshold (difference between the command and the reality that triggers the safety)

  This safety is meant to prevent over-torques on position controlled
  grippers with no torque readings by checking how far the encoder output is
  from the desired command. If it is over the limit, it can only stay there
  for overCommandLimitIterN iterations before being released
  )")
  .def_prop_rw("overCommandLimitIterN",
      [](Gripper & self) { self.overCommandLimitIterN(); },
      [](Gripper & self, unsigned int N) { self.overCommandLimitIterN(N); },
      "iterN"_a, R"(
  Number of iterations where actualCommandDiffTrigger() threshold may be exceeded before the security is triggered)")
  .def_prop_rw("releaseSafetyOffset",
      [](Gripper & self) { self.releaseSafetyOffset(); },
      [](Gripper & self, double offset) { self.releaseSafetyOffset(offset); },
      "percent"_a, R"(
   Offset (in [rad]) by which the gripper is released if overCommandDiffTrigger is trigger for more than overCommandLimitIterN

  :param offset: offset angle in [rad] or distance in [meter]
      )");

  g.def("complete", &Gripper::complete, R"(
  Checks if the gripper motion stopped moving.

  The gripper will stop if:

  * the desired motion is finished
  * the gripper encountered an obstacle and gripper safety was triggered.
    This is defined by an encoder error threshold (actualCommandDiffTrigger) and a maximum number of iterations where the gripper is allowed to be at this threshold (overCommandLimitIterN)

  :returns: True if gripper is not moving, False if it is moving
  )")
  .def("is_metric", &Gripper::is_metric, R"(When true the gripper is considered "open" when the joints' values are minimal)")
  .def("reversed", &Gripper::reversed, R"(When true the gripper is considered "open" when the joints' values are minimal)");

}

} // namespace mc_rtc_python
