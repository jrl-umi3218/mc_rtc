#ifndef _H_TRAJECTORYTASK_H_
#define _H_TRAJECTORYTASK_H_

#include <mc_tasks/MetaTask.h>
#include <mc_trajectory/BSplineTrajectory.h>

#include <mc_rbdyn/robot.h>
#include <Tasks/QPTasks.h>

#include <mc_tasks/api.h>

namespace mc_tasks
{

struct MC_TASKS_DLLAPI TrajectoryTask : public MetaTask
{
public:
  TrajectoryTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex,
                 const mc_rbdyn::Surface & surface, const sva::PTransformd & X_0_t,
                 double duration, double timeStep, double stiffness, double posWeight, double oriWeight,
                 const std::string & name = "TrajectoryTask",
                 const Eigen::MatrixXd & waypoints = Eigen::MatrixXd(3,0),
                 unsigned int nrWP = 0);

  virtual void addToSolver(mc_solver::QPSolver & solver) override;

  virtual void removeFromSolver(mc_solver::QPSolver & solver) override;

  virtual void update() override;

  bool timeElapsed();

  const Eigen::VectorXd & eval() const;

  const Eigen::VectorXd & speed() const;

  std::vector<Eigen::Vector3d> controlPoints();

  void generateBS();
public:
  const mc_rbdyn::Robots & robots;
  const mc_rbdyn::Surface & surface;
  sva::PTransformd X_0_t;
  sva::PTransformd X_0_start;
  Eigen::MatrixXd wp;
  double duration;
  double timeStep;
  double t;
  std::shared_ptr<tasks::qp::TransformTask> transTask;
  std::shared_ptr<tasks::qp::TrajectoryTask> transTrajTask;
  std::shared_ptr<mc_trajectory::BSplineTrajectory> bspline;
};

}

#endif
