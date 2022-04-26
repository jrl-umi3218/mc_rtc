/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/OrientationTask.h>

#include <mc_rtc/gui/Rotation.h>

namespace mc_tasks
{

OrientationTask::OrientationTask(const std::string & bodyName,
                                 const mc_rbdyn::Robots & robots,
                                 unsigned int robotIndex,
                                 double stiffness,
                                 double weight)
: OrientationTask(robots.robot(robotIndex).frame(bodyName), stiffness, weight)
{
}

OrientationTask::OrientationTask(const mc_rbdyn::RobotFrame & frame, double stiffness, double weight)
: TrajectoryTaskGeneric<tasks::qp::OrientationTask>(frame, stiffness, weight), frame_(frame)
{
  finalize(robots.mbs(), static_cast<int>(rIndex), frame.body(), (frame.X_b_f().inv() * frame.position()).rotation());
  type_ = "orientation";
  name_ = "orientation_" + frame.robot().name() + "_" + frame.name();
}

void OrientationTask::reset()
{
  TrajectoryTaskGeneric::reset();
  errorT->orientation((frame_->X_b_f().inv() * frame_->position()).rotation());
}

void OrientationTask::orientation(const Eigen::Matrix3d & ori)
{
  errorT->orientation((frame_->X_b_f() * sva::PTransformd{ori}).rotation());
}

Eigen::Matrix3d OrientationTask::orientation()
{
  return (frame_->X_b_f().inv() * errorT->orientation()).rotation();
}

void OrientationTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric<tasks::qp::OrientationTask>::addToGUI(gui);
  gui.addElement({"Tasks", name_},
                 mc_rtc::gui::Rotation(
                     "ori_target",
                     [this]() -> sva::PTransformd {
                       return sva::PTransformd(this->orientation(), frame_->position().translation());
                     },
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
