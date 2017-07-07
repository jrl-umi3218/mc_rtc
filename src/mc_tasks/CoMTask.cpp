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

mc_tasks::MetaTaskPtr load_com_task(mc_solver::QPSolver & solver,
                                    const mc_rtc::Configuration & config)
{
  auto t = std::make_shared<mc_tasks::CoMTask>(solver.robots(), config("robotIndex"));
  if(config.has("com"))
  {
    t->com(config("com"));
  }
  t->load(solver, config);
  return t;
}

struct CoMLoader
{
  static bool registered;
};

bool CoMLoader::registered = mc_tasks::MetaTaskLoader::register_load_function("com", &load_com_task);

}
