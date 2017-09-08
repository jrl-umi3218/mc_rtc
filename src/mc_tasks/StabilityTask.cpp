#include <mc_tasks/StabilityTask.h>

#include <mc_rbdyn/Surface.h>

#include <mc_rtc/logging.h>

namespace mc_tasks
{

StabilityTask::StabilityTask(mc_rbdyn::Robots & robots)
: robots(robots), robot(robots.robot()),
  comStiff(1), extraComStiff(0),
  comObj(rbd::computeCoM(robot.mb(), robot.mbc())),
  comTask(new tasks::qp::CoMTask(robots.mbs(), 0, comObj)),
  comTaskSp(new tasks::qp::SetPointTask(robots.mbs(), 0, comTask.get(), comStiff, 1)),
  comTaskSm(
    std::bind(static_cast<void (tasks::qp::SetPointTask::*)(double)>(&tasks::qp::SetPointTask::weight), comTaskSp.get(), std::placeholders::_1),
    std::bind(static_cast<double (tasks::qp::SetPointTask::*)() const>(&tasks::qp::SetPointTask::weight), comTaskSp.get()),
    std::bind(static_cast<void (tasks::qp::CoMTask::*)(const Eigen::Vector3d&)>(&tasks::qp::CoMTask::com), comTask.get(), std::placeholders::_1),
    std::bind(static_cast<const Eigen::Vector3d (tasks::qp::CoMTask::*)() const>(&tasks::qp::CoMTask::com), comTask.get()),
    1, comObj, 1), /*FIXME There may be a more convenient way to do this... */
  qObj(robot.mbc().q),
  postureTask(new tasks::qp::PostureTask(robots.mbs(), 0, qObj, 1, 1))
{
  name_ = "stability_" + robot.name();
}

void StabilityTask::highStiffness(const std::vector<std::string> & stiffJoints)
{
  std::vector<tasks::qp::JointStiffness> jsv;
  for(const auto & jn : stiffJoints)
  {
    jsv.push_back({jn, 10*postureTask->stiffness()});
  }
  postureTask->jointsStiffness(robots.mbs(), jsv);
}

void StabilityTask::normalStiffness(const std::vector<std::string> & stiffJoints)
{
  std::vector<tasks::qp::JointStiffness> jsv;
  for(const auto & jn : stiffJoints)
  {
    jsv.push_back({jn, postureTask->stiffness()});
  }
  postureTask->jointsStiffness(robots.mbs(), jsv);
}

void StabilityTask::target(const mc_rbdyn::Robot &/*env*/, const mc_rbdyn::Stance & stance,
                           const mc_rbdyn::StanceConfig & config, double comSmoothPercent)
{
  comObj = stance.com(robot);
  qObj = stance.q();

  Eigen::Vector3d comOffset = Eigen::Vector3d::Zero();
  for(const auto & c : stance.stabContacts())
  {
    const std::string & rsname = c.r1Surface()->name();
    if(rsname == "LeftFoot" || rsname == "RightFoot" ||
       rsname == "LFrontSole" || rsname == "RFrontSole" ||
       rsname == "LFullSole" || rsname == "RFullSole")
    {
      sva::PTransformd pos = c.X_0_r1s(robots);
      sva::PTransformd posRobot = c.r1Surface()->X_0_s(robot);
      comOffset = posRobot.translation() - pos.translation();
      break;
    }
  }

  LOG_INFO("comOffset applied from contact displacement " << comOffset.transpose())
  comObj += comOffset;
  LOG_INFO("comOffset from config " << config.comObj.comOffset.transpose())
  comObj += config.comObj.comOffset;

  postureTask->stiffness(config.postureTask.stiffness);
  postureTask->weight(config.postureTask.weight);
  postureTask->posture(qObj);

  comStiff = config.comTask.stiffness;
  extraComStiff = config.comTask.extraStiffness;
  comTaskSp->stiffness(comStiff);
  comTaskSm.reset(config.comTask.weight, comObj, comSmoothPercent);
}

void StabilityTask::addToSolver(mc_solver::QPSolver & solver)
{
  solver.addTask(comTaskSp.get());
  solver.addTask(postureTask.get());
}

void StabilityTask::removeFromSolver(mc_solver::QPSolver & solver)
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

void StabilityTask::reset()
{
  postureTask->posture(robot.mbc().q);
  auto cur_com = rbd::computeCoM(robot.mb(), robot.mbc());
  comTask->com(cur_com);
}

Eigen::VectorXd StabilityTask::eval() const
{
  Eigen::VectorXd ret(comTask->dim() + postureTask->eval().size());
  ret << comTask->eval(), postureTask->eval();
  return ret;
}

Eigen::VectorXd StabilityTask::speed() const
{
  Eigen::VectorXd ret = Eigen::VectorXd::Zero(comTask->dim() + postureTask->eval().size());
  ret << comTask->speed();
  return ret;
}

}
