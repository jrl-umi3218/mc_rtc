#include <mc_tasks/CoMTask.h>

#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

CoMTask::CoMTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex,
                 double stiffness, double weight)
: TrajectoryTaskGeneric<tasks::qp::CoMTask>(robots, robotIndex, stiffness, weight),
  cur_com(Eigen::Vector3d::Zero())
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);

  cur_com = rbd::computeCoM(robot.mb(), robot.mbc());

  finalize(robots.mbs(), static_cast<int>(robotIndex), cur_com);
}

void CoMTask::reset()
{
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  cur_com = rbd::computeCoM(robot.mb(), robot.mbc());
  errorT->com(cur_com);
}

void CoMTask::move_com(const Eigen::Vector3d & com)
{
  cur_com += com;
  errorT->com(cur_com);
}

void CoMTask::com(const Eigen::Vector3d & com)
{
  errorT->com(com);
}

Eigen::Vector3d CoMTask::com()
{
  return errorT->com();
}

}

namespace
{

mc_tasks::MetaTaskPtr load_com_task(const mc_rbdyn::Robots & robots,
                                    const mc_rtc::Configuration & config)
{
  if(!config.has("robotIndex"))
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Stored CoMTask does not have a robotIndex parameter")
  }
  double stiffness = 5.0;
  double weight = 100.0;
  config("stiffness", stiffness);
  config("weight", weight);
  auto t = std::make_shared<mc_tasks::CoMTask>(robots, config("robotIndex"), stiffness, weight);
  if(config.has("com"))
  {
    t->com(config("com"));
  }
  return t;
}

struct CoMLoader
{
  static bool registered;
};

bool CoMLoader::registered = mc_tasks::MetaTaskLoader::register_load_function("com", &load_com_task);

}
