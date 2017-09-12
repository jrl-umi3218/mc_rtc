#include <mc_tasks/CoMTask.h>

#include <mc_tasks/MetaTaskLoader.h>

namespace mc_tasks
{

CoMTask::CoMTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex,
                 double stiffness, double weight)
: TrajectoryTaskGeneric<tasks::qp::CoMTask>(robots, robotIndex, stiffness, weight),
  robot_index_(robotIndex),
  cur_com_(Eigen::Vector3d::Zero())
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);

  cur_com_ = rbd::computeCoM(robot.mb(), robot.mbc());

  finalize(robots.mbs(), static_cast<int>(robotIndex), cur_com_);
  name_ = "com_" + robots.robot(robot_index_).name();
}

void CoMTask::reset()
{
  const mc_rbdyn::Robot & robot = robots.robot(rIndex);
  cur_com_ = rbd::computeCoM(robot.mb(), robot.mbc());
  errorT->com(cur_com_);
}

void CoMTask::move_com(const Eigen::Vector3d & com)
{
  cur_com_ += com;
  errorT->com(cur_com_);
}

void CoMTask::com(const Eigen::Vector3d & com)
{
  cur_com_ = com;
  errorT->com(com);
}

Eigen::Vector3d CoMTask::com()
{
  return errorT->com();
}

void CoMTask::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_target",
                     [this]() -> const Eigen::Vector3d &
                     {
                     return cur_com_;
                     });
  logger.addLogEntry(name_,
                     [this]() -> Eigen::Vector3d
                     {
                     return cur_com_ - eval();
                     });
}

void CoMTask::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_target");
  logger.removeLogEntry(name_);
}

}

namespace
{

static bool registered = mc_tasks::MetaTaskLoader::register_load_function("com",
  [](mc_solver::QPSolver & solver,
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
);

}
