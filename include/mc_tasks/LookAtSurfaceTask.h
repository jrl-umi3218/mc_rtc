#pragma once

#include <mc_tasks/LookAtTask.h>

namespace mc_tasks
{

/*! \brief Track a surface position with a "gaze" vector.
 * This task is a convenience wrapper for LookAtTask that takes care of
 * automatically updating the gaze target.
 */
struct MC_TASKS_DLLAPI LookAtSurfaceTask : public LookAtTask
{
public:
  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
   * \param bodyVector Gaze vector for the body.
        For instance [1., 0, 0] will try to align the x axis of the body with
   the target direction
   * \param  name of the target's source tf
   * \param targetFrame name of the target's tf
   * \param robots Robots controlled by this task
   * \param robotIndex Index of the robot controlled by this task
   * \param stiffness Task stiffness
   * \param weight Task weight
   *
   */
  LookAtSurfaceTask(const mc_rbdyn::Robots & robots,
                    unsigned int robotIndex,
                    const std::string & bodyName,
                    const Eigen::Vector3d & bodyVector,
                    unsigned int surfaceRobotIndex,
                    const std::string & surfaceName,
                    double stiffness = 0.5,
                    double weight = 200);

  /*! \brief Update the gaze target from TF position */
  void update() override;

  /*! \brief Access offset relative to the surface */
  const sva::PTransformd & offset() const;

  /*! \brief Set the offset relative to the surface */
  void offset(const sva::PTransformd & off);

private:
  /*! Index of robot on which the surface target is attached */
  unsigned int sRobotIndex;
  /*! Target surface name */
  std::string sName;
  /*! Offset to the surface in surface frame */
  sva::PTransformd offset_;
};

} // namespace mc_tasks
