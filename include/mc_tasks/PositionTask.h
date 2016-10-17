#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace mc_tasks
{

/*! \brief Control the position of a body

 * This task is thin wrapper around the appropriate tasks in Tasks.
 *
 */
struct MC_TASKS_DLLAPI PositionTask : public TrajectoryTaskGeneric<tasks::qp::PositionTask>
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
  PositionTask(const std::string & bodyName, const mc_rbdyn::Robots
                  & robots, unsigned int robotIndex, double stiffness =
                  2.0, double weight = 500);

  /*! \brief Reset the task
   *
   * Set the task objective to the current body position
   */
  virtual void reset() override;

  /*! \brief Get the body position target */
  Eigen::Vector3d position();

  /*! \brief Set the body position target
   *
   * \param pos Body position in world frame
   *
   */
  void position(const Eigen::Vector3d & pos);
protected:
  std::string bodyName;
  unsigned int bIndex;
};

}
