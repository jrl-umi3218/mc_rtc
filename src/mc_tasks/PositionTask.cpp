/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/PositionTask.h>

#include <mc_rtc/gui/Point3D.h>

namespace mc_tasks
{

PositionTask::PositionTask(const std::string & bodyName,
                           const mc_rbdyn::Robots & robots,
                           unsigned int robotIndex,
                           double stiffness,
                           double weight)
: PositionTask(bodyName, Eigen::Vector3d::Zero(), robots, robotIndex, stiffness, weight)
{
}

PositionTask::PositionTask(const std::string & bodyName,
                           const Eigen::Vector3d & bodyPoint,
                           const mc_rbdyn::Robots & robots,
                           unsigned int robotIndex,
                           double stiffness,
                           double weight)
: TrajectoryTaskGeneric<tasks::qp::PositionTask>(robots, robotIndex, stiffness, weight), bodyName(bodyName), bIndex(0)
{
  if(robotIndex >= robots.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_tasks::PositionTask] No robot with index {}, robots.size() {}", robotIndex, robots.size());
  }
  const auto & robot = robots.robot(robotIndex);
  if(!robot.hasBody(bodyName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_tasks::PositionTask] No body named {} in {}", bodyName,
                                                     robot.name());
  }
  bIndex = robot.bodyIndexByName(bodyName);

  Eigen::Vector3d curPos = (sva::PTransformd{bodyPoint} * robot.mbc().bodyPosW[bIndex]).translation();
  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, curPos, bodyPoint);
  type_ = "position";
  name_ = "position_" + robot.name() + "_" + bodyName;
}

void PositionTask::reset()
{
  TrajectoryTaskGeneric::reset();
  const auto & robot = robots.robot(rIndex);
  Eigen::Vector3d curPos = (sva::PTransformd{errorT->bodyPoint()} * robot.mbc().bodyPosW[bIndex]).translation();
  errorT->position(curPos);
}

Eigen::Vector3d PositionTask::position()
{
  return errorT->position();
}

void PositionTask::position(const Eigen::Vector3d & pos)
{
  errorT->position(pos);
}

Eigen::Vector3d PositionTask::bodyPoint() const
{
  return errorT->bodyPoint();
}

void PositionTask::bodyPoint(const Eigen::Vector3d & bodyPoint)
{
  errorT->bodyPoint(bodyPoint);
}

void PositionTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  logger.addLogEntry(name_ + "_target", [this]() { return position(); });
  logger.addLogEntry(
      name_, [this]() -> const Eigen::Vector3d & { return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation(); });
}

void PositionTask::removeFromLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::removeFromLogger(logger);
  logger.removeLogEntry(name_ + "_target");
  logger.removeLogEntry(name_);
}

void PositionTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryTaskGeneric<tasks::qp::PositionTask>::addToGUI(gui);
  gui.addElement(
      {"Tasks", name_},
      mc_rtc::gui::Point3D("pos_target", [this]() { return this->position(); },
                           [this](const Eigen::Vector3d & pos) { this->position(pos); }),
      mc_rtc::gui::Point3D("pos", [this]() { return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation(); }));
}

} // namespace mc_tasks
