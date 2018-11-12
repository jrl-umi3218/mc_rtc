#include <mc_tasks/MetaTaskLoader.h>
#include <mc_tasks/RelativeEndEffectorTask.h>

namespace mc_tasks
{

RelativeEndEffectorTask::RelativeEndEffectorTask(const std::string & bodyName,
                                                 const mc_rbdyn::Robots & robots,
                                                 unsigned int robotIndex,
                                                 const std::string & relBodyName,
                                                 double stiffness,
                                                 double weight)
: RelativeEndEffectorTask(bodyName, Eigen::Vector3d::Zero(), robots, robotIndex, relBodyName, stiffness, weight)
{
}

RelativeEndEffectorTask::RelativeEndEffectorTask(const std::string & bodyName,
                                                 const Eigen::Vector3d & bodyPoint,
                                                 const mc_rbdyn::Robots & robots,
                                                 unsigned int robotIndex,
                                                 const std::string & relBodyName,
                                                 double stiffness,
                                                 double weight)
: EndEffectorTask(bodyName, bodyPoint, robots, robotIndex, stiffness, weight),
  relBodyIdx(
      robots.robot().bodyIndexByName(relBodyName.size() ? relBodyName : robots.robot(robotIndex).mb().body(0).name()))
{
  reset();
  const auto & robot = robots.robot(robotIndex);
  type_ = "relBody6d";
  name_ = "body6d_" + robot.name() + "_" + bodyName + "_rel_" + relBodyName;
}

void RelativeEndEffectorTask::reset()
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  sva::PTransformd X_0_body = robot.mbc().bodyPosW[bodyIndex];
  sva::PTransformd X_0_rel = robot.mbc().bodyPosW[relBodyIdx];

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

void RelativeEndEffectorTask::update()
{
  const sva::PTransformd & X_0_rel = robots.robot(robotIndex).mbc().bodyPosW[relBodyIdx];
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
  gui.addElement(
      {"Tasks", name_},
      mc_rtc::gui::Transform("pos_target",
                             [this]() { return curTransform * robots.robot(robotIndex).mbc().bodyPosW[relBodyIdx]; },
                             [this](const sva::PTransformd & X_0_target) {
                               set_ef_pose(X_0_target * robots.robot(robotIndex).mbc().bodyPosW[relBodyIdx].inv());
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
  auto t = std::make_shared<mc_tasks::OrientationTask>(config("body"), solver.robots(), config("robotIndex"));
  configure_ori_task(t, solver, config, true);
  t->load(solver, config);
  return t;
}

mc_tasks::MetaTaskPtr load_position_task(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  std::shared_ptr<mc_tasks::PositionTask> t = nullptr;
  if(config.has("bodyPoint"))
  {
    t = std::make_shared<mc_tasks::PositionTask>(config("body"), config("bodyPoint"), solver.robots(),
                                                 config("robotIndex"));
  }
  else
  {
    t = std::make_shared<mc_tasks::PositionTask>(config("body"), solver.robots(), config("robotIndex"));
  }
  configure_pos_task(t, solver, config, true);
  t->load(solver, config);
  return t;
}

mc_tasks::MetaTaskPtr load_ef_task(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  std::shared_ptr<mc_tasks::EndEffectorTask> t = nullptr;
  if(config.has("bodyPoint"))
  {
    t = std::make_shared<mc_tasks::EndEffectorTask>(config("body"), config("bodyPoint"), solver.robots(),
                                                    config("robotIndex"));
  }
  else
  {
    t = std::make_shared<mc_tasks::EndEffectorTask>(config("body"), solver.robots(), config("robotIndex"));
  }
  configure_pos_task(t->positionTask, solver, config, false);
  configure_ori_task(t->orientationTask, solver, config, false);
  t->load(solver, config);
  return t;
}

mc_tasks::MetaTaskPtr load_relef_task(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  std::shared_ptr<mc_tasks::RelativeEndEffectorTask> t = nullptr;
  if(config.has("bodyPoint"))
  {
    t = std::make_shared<mc_tasks::RelativeEndEffectorTask>(config("body"), config("bodyPoint"), solver.robots(),
                                                            config("robotIndex"), config("relBody"));
  }
  else
  {
    t = std::make_shared<mc_tasks::RelativeEndEffectorTask>(config("body"), solver.robots(), config("robotIndex"),
                                                            config("relBody"));
  }
  configure_pos_task(t->positionTask, solver, config, false);
  configure_ori_task(t->orientationTask, solver, config, false);
  t->set_ef_pose({t->orientationTask->orientation(), t->positionTask->position()});
  t->load(solver, config);
  return t;
}

static bool registered = mc_tasks::MetaTaskLoader::register_load_function("orientation", &load_orientation_task)
                         && mc_tasks::MetaTaskLoader::register_load_function("position", &load_position_task)
                         && mc_tasks::MetaTaskLoader::register_load_function("body6d", &load_ef_task)
                         && mc_tasks::MetaTaskLoader::register_load_function("relBody6d", &load_relef_task);

} // namespace
