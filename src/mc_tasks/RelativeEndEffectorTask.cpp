/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tasks/RelativeEndEffectorTask.h>

#include <mc_tasks/MetaTaskLoader.h>

#include <mc_rtc/gui/Transform.h>

#include <mc_rtc/deprecated.h>

namespace mc_tasks
{

RelativeEndEffectorTask::RelativeEndEffectorTask(const std::string & bodyName,
                                                 const mc_rbdyn::Robots & robots,
                                                 unsigned int robotIndex,
                                                 const std::string & relBodyName,
                                                 double stiffness,
                                                 double weight)
: RelativeEndEffectorTask(
    robots.robot(robotIndex).frame(bodyName),
    robots.robot(robotIndex).frame(relBodyName.size() ? relBodyName : robots.robot(robotIndex).mb().body(0).name()),
    stiffness,
    weight)
{
}

RelativeEndEffectorTask::RelativeEndEffectorTask(const mc_rbdyn::RobotFrame & frame,
                                                 const mc_rbdyn::Frame & relative,
                                                 double stiffness,
                                                 double weight)
: EndEffectorTask(frame, stiffness, weight), relative_(relative)
{
  reset();
  type_ = "relBody6d";
  auto relative_as_robot = dynamic_cast<const mc_rbdyn::RobotFrame *>(&relative);
  if(relative_as_robot)
  {
    name_ = fmt::format("body6d_{}_{}_rel_{}_{}", frame.robot().name(), frame.name(), relative_as_robot->robot().name(),
                        relative.name());
  }
  else
  {
    name_ = fmt::format("body6d_{}_{}_rel_{}", frame.robot().name(), frame.name(), relative.name());
  }
}

void RelativeEndEffectorTask::reset()
{
  sva::PTransformd X_0_body = frame().position();
  sva::PTransformd X_0_rel = relative_->position();
  curTransform = X_0_body * (X_0_rel.inv()); /* X_rel_body = X_0_body * X_rel_0 */
}

void RelativeEndEffectorTask::add_ef_pose(const sva::PTransformd & dtr)
{
  auto new_rot = curTransform.rotation() * dtr.rotation();
  Eigen::Vector3d new_t = curTransform.translation() + dtr.translation();
  curTransform = sva::PTransformd(new_rot, new_t);
}

void RelativeEndEffectorTask::set_ef_pose(const sva::PTransformd & tf)
{
  curTransform = tf;
}

void RelativeEndEffectorTask::update(mc_solver::QPSolver &)
{
  const sva::PTransformd & X_0_rel = relative_->position();
  sva::PTransformd X_0_bodyDes = curTransform * X_0_rel; /* X_0_body = X_rel_body * X_0_rel */
  positionTask->position(X_0_bodyDes.translation());
  orientationTask->orientation(X_0_bodyDes.rotation());
}

sva::PTransformd RelativeEndEffectorTask::get_ef_pose()
{
  return curTransform;
}

void RelativeEndEffectorTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  EndEffectorTask::addToGUI(gui);
  gui.removeElement({"Tasks", name_}, "pos_target");
  gui.addElement({"Tasks", name_}, mc_rtc::gui::Transform(
                                       "pos_target", [this]() { return curTransform * relative_->position(); },
                                       [this](const sva::PTransformd & X_0_target) {
                                         set_ef_pose(X_0_target * relative_->position().inv());
                                       }));
}

} // namespace mc_tasks

namespace
{

void configure_pos_task(std::shared_ptr<mc_tasks::PositionTask> & t,
                        mc_solver::QPSolver & solver,
                        const mc_rtc::Configuration & config,
                        bool load_meta)
{
  if(load_meta)
  {
    t->load(solver, config);
  }
  if(config.has("position"))
  {
    t->position(config("position"));
  }
  if(config.has("relative") && config("relative").has("position"))
  {
    const auto & robot = robotFromConfig(config, solver.robots(), t->name());
    std::string s1 = config("relative")("s1");
    std::string s2 = config("relative")("s2");
    Eigen::Vector3d position = config("relative")("position");
    auto X_0_s1 = robot.surfacePose(s1);
    auto X_0_s2 = robot.surfacePose(s2);
    auto X_s1_s2 = X_0_s2 * X_0_s1.inv();
    X_s1_s2.translation() = X_s1_s2.translation() / 2;
    auto X_0_relative = X_s1_s2 * X_0_s1;
    t->position((sva::PTransformd(position) * X_0_relative).translation());
  }
  if(config.has("positionWeight"))
  {
    t->weight(config("positionWeight"));
  }
  if(config.has("positionStiffness"))
  {
    if(config("positionStiffness").size())
    {
      Eigen::Vector3d dimStiffness = config("positionStiffness");
      t->stiffness(dimStiffness);
    }
    else
    {
      t->stiffness(static_cast<double>(config("positionStiffness")));
    }
  }
}

void configure_ori_task(std::shared_ptr<mc_tasks::OrientationTask> & t,
                        mc_solver::QPSolver & solver,
                        const mc_rtc::Configuration & config,
                        bool load_meta)
{
  if(load_meta)
  {
    t->load(solver, config);
  }
  if(config.has("orientation"))
  {
    t->orientation(config("orientation"));
  }
  if(config.has("relative") && config("relative").has("orientation"))
  {
    const auto & robot = robotFromConfig(config, solver.robots(), "orientation");
    std::string s1 = config("relative")("s1");
    std::string s2 = config("relative")("s2");
    Eigen::Matrix3d orientation = config("relative")("orientation");
    auto X_0_s1 = robot.surfacePose(s1);
    auto X_0_s2 = robot.surfacePose(s2);
    auto X_s1_s2 = X_0_s2 * X_0_s1.inv();
    X_s1_s2.translation() = X_s1_s2.translation() / 2;
    auto X_0_relative = X_s1_s2 * X_0_s1;
    t->orientation((sva::PTransformd(orientation) * X_0_relative).rotation());
  }
  if(config.has("orientationWeight"))
  {
    t->weight(config("orientationWeight"));
  }
  if(config.has("orientationStiffness"))
  {
    if(config("orientationStiffness").size())
    {
      Eigen::Vector3d dimStiffness = config("orientationStiffness");
      t->stiffness(dimStiffness);
    }
    else
    {
      t->stiffness(static_cast<double>(config("orientationStiffness")));
    }
  }
}

mc_tasks::MetaTaskPtr load_orientation_task(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  const auto & frame = [&]() -> const mc_rbdyn::RobotFrame & {
    const auto & robot = robotFromConfig(config, solver.robots(), "orientation");
    if(config.has("body"))
    {
      mc_rtc::log::deprecated("OrientationTaskLoader", "body", "frame");
      return robot.frame(config("body"));
    }
    return robot.frame(config("frame"));
  }();
  auto t = std::make_shared<mc_tasks::OrientationTask>(frame);
  configure_ori_task(t, solver, config, true);
  t->load(solver, config);
  return t;
}

mc_tasks::MetaTaskPtr load_position_task(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("bodyPoint"))
  {
    mc_rtc::log::error_and_throw("[PositionTaskLoader] bodyPoint is not supported anymore.\nYou can create a frame "
                                 "with the corresponding body point instead");
  }
  const auto & frame = [&]() -> const mc_rbdyn::RobotFrame & {
    const auto & robot = robotFromConfig(config, solver.robots(), "position");
    if(config.has("body"))
    {
      mc_rtc::log::deprecated("PositionTaskLoader", "body", "frame");
      return robot.frame(config("body"));
    }
    return robot.frame(config("frame"));
  }();
  auto t = std::make_shared<mc_tasks::PositionTask>(frame);
  configure_pos_task(t, solver, config, true);
  t->load(solver, config);
  return t;
}

mc_tasks::MetaTaskPtr load_ef_task(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("bodyPoint"))
  {
    mc_rtc::log::error_and_throw("[EndEffectorTaskLoader] bodyPoint is not supported anymore.\nYou can create a frame "
                                 "with the corresponding body point instead");
  }
  const auto & frame = [&]() -> const mc_rbdyn::RobotFrame & {
    const auto & robot = robotFromConfig(config, solver.robots(), "endEffector");
    if(config.has("body"))
    {
      mc_rtc::log::deprecated("EndEffectorTaskLoader", "body", "frame");
      return robot.frame(config("body"));
    }
    return robot.frame(config("frame"));
  }();
  auto t = std::make_shared<mc_tasks::EndEffectorTask>(frame);
  configure_pos_task(t->positionTask, solver, config, false);
  configure_ori_task(t->orientationTask, solver, config, false);
  t->load(solver, config);
  return t;
}

mc_tasks::MetaTaskPtr load_relef_task(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  if(config.has("bodyPoint"))
  {
    mc_rtc::log::error_and_throw("[RelativeEndEffectorTaskLoader] bodyPoint is not supported anymore.\nYou can create "
                                 "a frame with the corresponding body point instead");
  }
  const auto & frame = [&]() -> const mc_rbdyn::RobotFrame & {
    const auto & robot = robotFromConfig(config, solver.robots(), "relativeEndEffector");
    if(config.has("body"))
    {
      mc_rtc::log::deprecated("RelativeEndEffectorTaskLoader", "body", "frame");
      return robot.frame(config("body"));
    }
    return robot.frame(config("frame"));
  }();
  const auto & relFrame = [&]() -> const mc_rbdyn::RobotFrame & {
    if(config.has("relBody"))
    {
      mc_rtc::log::deprecated("RelativeEndEffectorTaskLoader", "relBody", "relativeFrame");
      return frame.robot().frame(config("relBody"));
    }
    auto c = config("relativeFrame");
    const auto & robot = robotFromConfig(c, solver.robots(), "relativeEndEffector::relativeFrame");
    return robot.frame(c("frame"));
  }();
  auto t = std::make_shared<mc_tasks::RelativeEndEffectorTask>(frame, relFrame);
  configure_pos_task(t->positionTask, solver, config, false);
  configure_ori_task(t->orientationTask, solver, config, false);
  t->set_ef_pose({t->orientationTask->orientation(), t->positionTask->position()});
  t->load(solver, config);
  return t;
}

static auto orientation_registered =
    mc_tasks::MetaTaskLoader::register_load_function("orientation", &load_orientation_task);
static auto position_registered = mc_tasks::MetaTaskLoader::register_load_function("position", &load_position_task);
static auto body6d_registered = mc_tasks::MetaTaskLoader::register_load_function("body6d", &load_ef_task);
static auto relbody6d_registered = mc_tasks::MetaTaskLoader::register_load_function("relBody6d", &load_relef_task);

} // namespace
