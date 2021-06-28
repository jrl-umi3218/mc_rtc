/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/VectorOrientationTask.h>

#include <mc_rtc/gui/Arrow.h>
#include <mc_rtc/gui/Point3D.h>

namespace mc_tasks
{

VectorOrientationTask::VectorOrientationTask(const std::string & bodyName,
                                             const Eigen::Vector3d & bodyVector,
                                             const Eigen::Vector3d & targetVector,
                                             const mc_rbdyn::Robots & robots,
                                             unsigned int robotIndex,
                                             double stiffness,
                                             double weight)
: TrajectoryTaskGeneric<tasks::qp::VectorOrientationTask>(robots, robotIndex, stiffness, weight), bodyName(bodyName),
  bIndex(0)
{
  if(robotIndex >= robots.size())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_tasks::VectorOrientationTask] No robot with index {}, robots.size() {}", robotIndex, robots.size());
  }
  const auto & robot = robots.robot(robotIndex);
  if(!robot.hasBody(bodyName))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_tasks::VectorOrientationTask] No body named {} in {}",
                                                     bodyName, robot.name());
  }
  finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, bodyVector, targetVector);
  bIndex = robot.bodyIndexByName(bodyName);
  type_ = "vectorOrientation";
  name_ = "vector_orientation_" + robot.name() + "_" + bodyName;
}

VectorOrientationTask::VectorOrientationTask(const std::string & bodyName,
                                             const Eigen::Vector3d & bodyVector,
                                             const mc_rbdyn::Robots & robots,
                                             unsigned int robotIndex,
                                             double stiffness,
                                             double weight)
: VectorOrientationTask(bodyName, bodyVector, bodyVector, robots, robotIndex, stiffness, weight)
{
  reset();
}

void VectorOrientationTask::reset()
{
  TrajectoryTaskGeneric::reset();
  // Should be errorT->actual(), but it is not computed until the first call to
  // errorT::update()
  Eigen::Matrix3d E_0_b = robots.robot().bodyPosW()[bIndex].rotation().transpose();
  Eigen::Vector3d actualVector = E_0_b * this->bodyVector();
  this->targetVector(actualVector);
}

void VectorOrientationTask::bodyVector(const Eigen::Vector3d & vector)
{
  errorT->bodyVector(vector);
}

const Eigen::Vector3d & VectorOrientationTask::bodyVector() const
{
  return errorT->bodyVector();
}

void VectorOrientationTask::targetVector(const Eigen::Vector3d & ori)
{
  errorT->target(ori.normalized());
}

const Eigen::Vector3d & VectorOrientationTask::targetVector() const
{
  return errorT->target();
}

const Eigen::Vector3d & VectorOrientationTask::actual() const
{
  return errorT->actual();
}

void VectorOrientationTask::addToLogger(mc_rtc::Logger & logger)
{
  TrajectoryBase::addToLogger(logger);
  MC_RTC_LOG_GETTER(name_ + "_target", targetVector);
  MC_RTC_LOG_HELPER(name_ + "_current", actual);
  MC_RTC_LOG_HELPER(name_ + "_error", eval);
}

void VectorOrientationTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  TrajectoryBase::addToGUI(gui);
  gui.addElement(
      {"Tasks", name_},
      mc_rtc::gui::ArrayInput(
          "Target Direction", {"x", "y", "z"}, [this]() { return targetVector(); },
          [this](const Eigen::Vector3d & target) { targetVector(target); }),
      mc_rtc::gui::Arrow(
          "Actual", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(0., 0., 1.)),
          [this]() -> const Eigen::Vector3d & { return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation(); },
          [this]() -> Eigen::Vector3d {
            return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation() + .25 * actual();
          }),
      mc_rtc::gui::Arrow(
          "Target", mc_rtc::gui::ArrowConfig(mc_rtc::gui::Color(1., 0., 0.)),
          [this]() -> const Eigen::Vector3d & { return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation(); },
          [this]() -> Eigen::Vector3d {
            return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation() + .25 * targetVector();
          }),
      mc_rtc::gui::Point3D(
          "Arrow end point",
          [this]() -> Eigen::Vector3d {
            return robots.robot(rIndex).mbc().bodyPosW[bIndex].translation() + .25 * targetVector();
          },
          [this](const Eigen::Vector3d & point) {
            Eigen::Vector3d direction = point - robots.robot(rIndex).mbc().bodyPosW[bIndex].translation();
            targetVector(direction);
          }));
}
} // namespace mc_tasks

namespace
{

static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
    "vectorOrientation",
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) {
      auto t = std::make_shared<mc_tasks::VectorOrientationTask>(
          config("body"), config("bodyVector"), solver.robots(),
          robotIndexFromConfig(config, solver.robots(), "vectorOrientation"));
      t->load(solver, config);
      if(config.has("targetVector"))
      {
        t->targetVector(config("targetVector"));
      }
      return t;
    });
}
