#include <mc_tasks/StabilityTask.h>

namespace mc_tasks
{

StabilityTask::StabilityTask(mc_rbdyn::Robots & robots)
: robots(robots), robot(robots.robot()),
  comStiff(1), extraComStiff(0),
  comObj(rbd::computeCoM(*(robot.mb), *(robot.mbc))),
  comTask(new tasks::qp::CoMTask(robots.mbs, 0, comObj)),
  comTaskSp(new tasks::qp::SetPointTask(robots.mbs, 0, comTask.get(), comStiff, 1)),
  comTaskSm(
    std::bind(static_cast<void (tasks::qp::SetPointTask::*)(double)>(&tasks::qp::SetPointTask::weight), comTaskSp.get(), std::placeholders::_1),
    std::bind(static_cast<double (tasks::qp::SetPointTask::*)() const>(&tasks::qp::SetPointTask::weight), comTaskSp.get()),
    std::bind(static_cast<void (tasks::qp::CoMTask::*)(const Eigen::Vector3d&)>(&tasks::qp::CoMTask::com), comTask.get(), std::placeholders::_1),
    std::bind(static_cast<const Eigen::Vector3d (tasks::qp::CoMTask::*)() const>(&tasks::qp::CoMTask::com), comTask.get()),
    1, comObj, 1), /*FIXME There may be a more convenient way to do this... */
  qObj(robot.mbc->q),
  postureTask(new tasks::qp::PostureTask(robots.mbs, 0, qObj, 1, 1))
{
}

void StabilityTask::target(const mc_rbdyn::Robot & env, const mc_rbdyn::Stance & stance,
                           const mc_rbdyn::StanceConfig & config, double comSmoothPercent)
{
  comObj = stance.com(robot);
  qObj = stance.q;

  Eigen::Vector3d comOffset = Eigen::Vector3d::Zero();
  for(const auto & c : stance.stabContacts)
  {
    const std::string & rsname = c.robotSurface->name;
    if(rsname == "LeftFoot" or rsname == "RightFoot" or
       rsname == "LFrontSole" or rsname == "RFrontSole" or
       rsname == "LFullSole" or rsname == "RFullSole")
    {
      sva::PTransformd pos = c.X_0_rs(env);
      sva::PTransformd posRobot = c.robotSurface->X_0_s(robot);
      comOffset = posRobot.translation() - pos.translation();
      break;
    }
  }

  comObj += comOffset;
  comObj += config.comObj.comOffset;

  postureTask->stiffness(config.postureTask.stiffness);
  postureTask->weight(config.postureTask.weight);
  postureTask->posture(qObj);

  comStiff = config.comTask.stiffness;
  extraComStiff = config.comTask.extraStiffness;
  comTaskSp->stiffness(comStiff);
  comTaskSm.reset(config.comTask.weight, comObj, comSmoothPercent);
}

void StabilityTask::addToSolver(tasks::qp::QPSolver & solver)
{
  solver.addTask(comTaskSp.get());
  solver.addTask(postureTask.get());
  solver.updateTasksNrVars(robots.mbs);
}

void StabilityTask::removeFromSolver(tasks::qp::QPSolver & solver)
{
  solver.removeTask(comTaskSp.get());
  solver.removeTask(postureTask.get());
}

void StabilityTask::update()
{
  comTaskSm.update();
  double err = comTask->eval().norm();
  double extra = extraStiffness(err, extraComStiff);
  comTaskSp->stiffness(comStiff + extra);
}

}
