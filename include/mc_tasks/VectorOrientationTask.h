#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace mc_tasks
{

/*! \brief Control the orientation of a body
 *
 * This task is thin wrapper around the appropriate tasks in Tasks.
 *
 */
struct MC_TASKS_DLLAPI VectorOrientationTask : public TrajectoryTaskGeneric<tasks::qp::VectorOrientationTask>
{
public:
  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  VectorOrientationTask(const std::string & bodyName, const Eigen::Vector3d& bodyVector,
                        const Eigen::Vector3d& targetVector,
                        const mc_rbdyn::Robots & robots,
                        unsigned int robotIndex,
                        double stiffness = 2.0, double weight = 500);

  /*! \brief Reset the task
   *
   * Set the task objective to the current body orientation
   */
  virtual void reset() override;

  /*! \brief Set the body orientation target
   *
   * \param ori Body orientation in world frame
   *
   */
  void bodyVector(const Eigen::Vector3d & ori);

  /*! \brief Get the current body orientation target
   *
   * \returns The body orientation target in world frame
   *
   */
  Eigen::Vector3d bodyVector();

  /*! \brief Return the controlled body */
  std::string body() { return bodyName; }
public:
  std::string bodyName;
  unsigned int bIndex;
};

}
