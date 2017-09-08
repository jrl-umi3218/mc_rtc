#include <mc_tasks/TrajectoryTask.h>

#include <mc_rbdyn/Surface.h>
#include <mc_trajectory/spline_utils.h>

namespace mc_tasks
{

TrajectoryTask::TrajectoryTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex,
               const mc_rbdyn::Surface & surface, const sva::PTransformd & X_0_t,
               double duration, double timeStep, double stiffness, double posWeight, double oriWeight,
               const Eigen::MatrixXd & waypoints,
               unsigned int nrWP)
: robots(robots), surface(surface), X_0_t(X_0_t), wp(waypoints),
  duration(duration), timeStep(timeStep), t(0.)
{
  const mc_rbdyn::Robot & robot = robots.robot(robotIndex);
  name_ = "trajectory_" + robot.name() + "_" + surface.name();
  X_0_start = surface.X_0_s(robot);

  if(nrWP > 0)
  {
    Eigen::Vector3d start = X_0_start.translation();
    Eigen::Vector3d end = X_0_t.translation();
    wp = mc_trajectory::generateInterpolatedWaypoints(start, end, nrWP);
  }

  transTask.reset(new tasks::qp::TransformTask(robots.mbs(), static_cast<int>(robotIndex), surface.bodyName(), X_0_start, surface.X_b_s()));
  transTrajTask.reset(new tasks::qp::TrajectoryTask(robots.mbs(), static_cast<int>(robotIndex), transTask.get(), stiffness, 2*sqrt(stiffness), 1.0));
  Eigen::VectorXd dimWeight(6);
  for(unsigned int i = 0; i < 3; ++i)
  {
    dimWeight(i) = oriWeight;
  }
  for(unsigned int i = 3; i < 6; ++i)
  {
    dimWeight(i) = posWeight;
  }
  transTrajTask->dimWeight(dimWeight);

  generateBS();
}

void TrajectoryTask::addToSolver(mc_solver::QPSolver & solver)
{
  solver.addTask(transTrajTask.get());
}

void TrajectoryTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  solver.removeTask(transTrajTask.get());
}

void TrajectoryTask::update()
{
  auto res = bspline->splev({t}, 2);
  Eigen::Vector3d & pos = res[0][0];
  Eigen::Vector3d & vel = res[0][1];
  Eigen::Vector3d & acc = res[0][2];
  sva::PTransformd interp = sva::interpolate(X_0_start, X_0_t, t/duration);
  sva::PTransformd target(interp.rotation(), pos);
  Eigen::VectorXd refVel(6);
  Eigen::VectorXd refAcc(6);
  for(unsigned int i = 0; i < 3; ++i)
  {
    refVel(i) = 0;
    refAcc(i) = 0;
    refVel(i+3) = vel(i);
    refAcc(i+3) = acc(i);
  }
  transTask->target(target);
  transTrajTask->refVel(refVel);
  transTrajTask->refAccel(refAcc);

  t = std::min(t + timeStep, duration);
}

bool TrajectoryTask::timeElapsed()
{
  return t >= duration;
}

Eigen::VectorXd TrajectoryTask::eval() const
{
  return transTask->eval();
}

Eigen::VectorXd TrajectoryTask::speed() const
{
  return transTask->speed();
}

std::vector<Eigen::Vector3d> TrajectoryTask::controlPoints()
{
  std::vector<Eigen::Vector3d> res;
  res.reserve(static_cast<unsigned int>(wp.size()) + 2);
  res.push_back(X_0_start.translation());
  for(int i = 0; i < wp.cols(); ++i)
  {
    Eigen::Vector3d tmp;
    tmp(0) = wp(0, i);
    tmp(1) = wp(1, i);
    tmp(2) = wp(2, i);
    res.push_back(tmp);
  }
  res.push_back(X_0_t.translation());
  return res;
}

void TrajectoryTask::generateBS()
{
  bspline.reset(new mc_trajectory::BSplineTrajectory(controlPoints(), duration));
}

}
