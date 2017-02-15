#include <mc_tasks/GazeTask.h>

namespace mc_tasks
{

GazeTask::GazeTask(const std::string & bodyName,
                    const Eigen::Vector2d & point2d,
                    double depthEstimate, const sva::PTransformd & X_b_gaze,
                    const mc_rbdyn::Robots & robots, unsigned int robotIndex,
                    double stiffness, double weight)
  : TrajectoryTaskGeneric<tasks::qp::GazeTask>(robots, robotIndex, stiffness, weight)
{
    finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, point2d, depthEstimate, X_b_gaze);
}

GazeTask::GazeTask(const std::string & bodyName,
                    const Eigen::Vector3d & point3d,
                    const sva::PTransformd & X_b_gaze,
                    const mc_rbdyn::Robots & robots, unsigned int robotIndex,
                    double stiffness, double weight)
  : TrajectoryTaskGeneric<tasks::qp::GazeTask>(robots, robotIndex, stiffness, weight)
{
    finalize(robots.mbs(), static_cast<int>(rIndex), bodyName, point3d, X_b_gaze);
}

void GazeTask::reset()
{
    errorT->error(Eigen::Vector2d::Zero().eval(), Eigen::Vector2d::Zero());
}

void GazeTask::error(const Eigen::Vector2d & point2d, const Eigen::Vector2d & point2d_ref)
{
  errorT->error(point2d, point2d_ref);
}

void GazeTask::error(const Eigen::Vector3d & point3d, const Eigen::Vector2d & point2d_ref)
{
  errorT->error(point3d, point2d_ref);
}



}
