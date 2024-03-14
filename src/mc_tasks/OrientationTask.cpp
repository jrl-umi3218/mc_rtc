/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/OrientationTask.h>

#include <mc_tvm/OrientationFunction.h>

#include <mc_rtc/gui/Rotation.h>

namespace mc_tasks
{

static inline mc_rtc::void_ptr_caster<tasks::qp::OrientationTask> tasks_error{};
static inline mc_rtc::void_ptr_caster<mc_tvm::OrientationFunction> tvm_error{};

OrientationTask::OrientationTask(const std::string & bodyName,
                                 const mc_rbdyn::Robots & robots,
                                 unsigned int robotIndex,
                                 double stiffness,
                                 double weight)
: OrientationTask(robots.robot(robotIndex).frame(bodyName), stiffness, weight)
{
}

OrientationTask::OrientationTask(const mc_rbdyn::RobotFrame & frame, double stiffness, double weight)
: TrajectoryTaskGeneric(frame, stiffness, weight), frame_(frame)
{
  switch(backend_)
  {
    case Backend::Tasks:
      finalize<Backend::Tasks, tasks::qp::OrientationTask>(robots.mbs(), static_cast<int>(rIndex), frame.body(),
                                                           (frame.X_b_f().inv() * frame.position()).rotation());
      break;
    case Backend::TVM:
      finalize<Backend::TVM, mc_tvm::OrientationFunction>(frame);
      break;
    default:
      mc_rtc::log::error_and_throw("[OrientationTask] Not implemented for backend: {}", backend_);
  }
  type_ = "orientation";
  name_ = "orientation_" + frame.robot().name() + "_" + frame.name();
}

void OrientationTask::reset()
{
  TrajectoryTaskGeneric::reset();
  switch(backend_)
  {
    case Backend::Tasks:
      orientation((frame_->X_b_f().inv() * frame_->position()).rotation());
      break;
    case Backend::TVM:
      orientation(frame_->position().rotation());
      break;
    default:
      break;
  }
}

void OrientationTask::orientation(const Eigen::Matrix3d & ori)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(errorT)->orientation((frame_->X_b_f().inv() * sva::PTransformd{ori}).rotation());
      break;
    case Backend::TVM:
      tvm_error(errorT)->orientation(ori);
      break;
    default:
      break;
  }
}

Eigen::Matrix3d OrientationTask::orientation()
{
  switch(backend_)
  {
    case Backend::Tasks:
      return (frame_->X_b_f() * tasks_error(errorT)->orientation()).rotation();
    case Backend::TVM:
      return tvm_error(errorT)->orientation();
    default:
      mc_rtc::log::error_and_throw("Not implemented");
  }
}

void OrientationTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Rotation(
                     "ori_target", [this]() -> sva::PTransformd
                     { return sva::PTransformd(this->orientation(), frame_->position().translation()); },
                     [this](const Eigen::Quaterniond & ori) { this->orientation(ori.toRotationMatrix()); }),
                 mc_rtc::gui::Rotation("ori", [this]() -> sva::PTransformd { return frame_->position(); }));
}

void OrientationTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_target", this, [this]() { return Eigen::Quaterniond(orientation()); });
  logger.addLogEntry(name_, this, [this]() { return Eigen::Quaterniond(frame_->position().rotation()); });
}

} // namespace mc_tasks
