#include <mc_tasks/TransformTask.h>
#include <mc_rbdyn/RobotFrame.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/Frame.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/trampoline.h>

namespace nb = nanobind;
using namespace nb::literals;

namespace mc_rtc_python
{

struct PyTransformTask : public mc_tasks::TransformTask {
    NB_TRAMPOLINE(mc_tasks::TransformTask, 2);

    void target(const mc_rbdyn::Frame & frame, const sva::PTransformd & offset) override {
        NB_OVERRIDE(target, frame, offset);
    }

    void targetVel(const sva::MotionVecd & worldVel) override {
        NB_OVERRIDE(targetVel, worldVel);
    }
};

void bind_TransformTask(nb::module_ & m)
{
    using mc_tasks::TransformTask;

    // ------------------------------------------------------------------
    // TransformTask
    // ------------------------------------------------------------------
    nb::class_<TransformTask, PyTransformTask>(m, "TransformTask",
        R"(
Control a frame 6D pose.

This task allows controlling the position and orientation of a robot frame. It can target:
- A robot frame
- A surface frame
- A target frame or frame velocity
- A pose offset in the target frame

Backend-aware:
- Tasks backend: uses TrajectoryTaskGeneric
)")

        // Constructors
        .def(nb::init<const mc_rbdyn::RobotFrame &, double, double>(),
             "frame"_a, "stiffness"_a = 2.0, "weight"_a = 500.0,
             R"(
Create a TransformTask for a given RobotFrame.

Parameters
----------
frame : mc_rbdyn.RobotFrame
    The frame to control.
stiffness : float, optional
    Task stiffness (default 2.0).
weight : float, optional
    Task weight (default 500.0).
)")

        .def(nb::init<const std::string &, const mc_rbdyn::Robots &, unsigned int, double, double>(),
             "surfaceName"_a, "robots"_a, "robotIndex"_a, "stiffness"_a = 2.0, "weight"_a = 500.0,
             R"(
Create a TransformTask using a surface name.

Parameters
----------
surfaceName : str
    Name of the surface frame to control.
robots : mc_rbdyn.Robots
    Robots controlled by this task.
robotIndex : int
    Index of the robot to control.
stiffness : float, optional
    Task stiffness (default 2.0).
weight : float, optional
    Task weight (default 500.0).
)")

        // Core methods
        .def("reset", &TransformTask::reset,
             R"(Reset the task target to the current frame position, with zero velocity and acceleration.)")

        .def("target", nb::overload_cast<>(&TransformTask::target, nb::const_),
             R"(Get the task's target pose (sva.PTransformd).)")

        .def("target", nb::overload_cast<const sva::PTransformd &>(&TransformTask::target),
             "worldPos"_a,
             R"(Set the task's target pose in the world frame.)")

        .def("targetSurface", &TransformTask::targetSurface,
             "robotIndex"_a, "surfaceName"_a, "offset"_a,
             R"(Target a robot surface with an optional offset.)")

        .def("targetFrame", &TransformTask::targetFrame,
             "targetFrame"_a, "offset"_a,
             R"(Target a given frame with an optional offset.)")

        .def("targetFrameVelocity", &TransformTask::targetFrameVelocity,
             "targetFrame"_a, "offset"_a,
             R"(Target a given frame velocity with an optional offset.)")

        // Properties
        .def_prop_ro("surface", &TransformTask::surface,
                               R"(Retrieve the controlled frame name.)")

        .def_prop_ro("frame", &TransformTask::frame,
                               R"(Return the controlled frame (mc_rbdyn.RobotFrame).)")

        .def_prop_ro("surfacePose", &TransformTask::surfacePose,
                               R"(Get the pose of the controlled frame in the inertial frame.)")

        // Stiffness / Damping
        .def("setGains", nb::overload_cast<const sva::MotionVecd &, const sva::MotionVecd &>(&TransformTask::setGains),
             "stiffness"_a, "damping"_a,
             R"(Set dimensional stiffness and damping.)")

        .def("stiffness", nb::overload_cast<const sva::MotionVecd &>(&TransformTask::stiffness),
             "stiffness"_a,
             R"(Set dimensional stiffness. Damping is automatically 2*sqrt(stiffness).)")

        .def("mvStiffness", &TransformTask::mvStiffness,
             R"(Get dimensional stiffness as a MotionVecd.)")

        .def("damping", nb::overload_cast<const sva::MotionVecd &>(&TransformTask::damping),
             "damping"_a,
             R"(Set dimensional damping.)")

        .def("mvDamping", &TransformTask::mvDamping,
             R"(Get dimensional damping as a MotionVecd.)")

        // Reference velocity / acceleration
        // Setter: refVelB(velB)
        .def("refVelB",
            nb::overload_cast<const sva::MotionVecd &>(&TransformTask::refVelB),
            "velB"_a,
            R"(Set the task's reference velocity in frame coordinates.)")

        // Getter: refVelB()
        .def("refVelB",
            nb::overload_cast<>(&TransformTask::refVelB, nb::const_),
            R"(Get the task's reference velocity in frame coordinates.)")


        .def("refAccel", nb::overload_cast<const sva::MotionVecd &>(&TransformTask::refAccel), "accel"_a,
             R"(Set the task's reference acceleration in frame coordinates.)")

        // Logger and configuration
        .def("load", &TransformTask::load, "solver"_a, "config"_a,
             R"(Load task parameters from a configuration object.)")

        .def("addToLogger", &TransformTask::addToLogger,
             R"(Add the task data to the logger.)");
}

}