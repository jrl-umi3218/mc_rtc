#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace mc_tasks
{

/*! \brief Control the Gaze of a body
 *
 * This task is thin wrapper around the appropriate tasks in Tasks.
 *
 */
struct MC_TASKS_DLLAPI GazeTask : public TrajectoryTaskGeneric<tasks::qp::GazeTask>
{
public:
  GazeTask(const std::string & bodyName,
            const Eigen::Vector2d & point2d,
            double depthEstimate, const sva::PTransformd & X_b_gaze,
            const mc_rbdyn::Robots & robots, unsigned int robotIndex, 
            double stiffness = 2.0, double weight = 500);

  GazeTask(const std::string & bodyName,
            const Eigen::Vector3d & point3d,
            const sva::PTransformd & X_b_gaze,
            const mc_rbdyn::Robots & robots, unsigned int robotIndex, 
            double stiffness = 2.0, double weight = 500);

  virtual void reset() override;

  void error(const Eigen::Vector2d & point2d, const Eigen::Vector2d & point2d_ref = Eigen::Vector2d::Zero());

  void error(const Eigen::Vector3d & point3d, const Eigen::Vector2d & point2d_ref = Eigen::Vector2d::Zero());
};

}
