#pragma once
#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace mc_tasks
{
/*! \brief Servo an end-effector depending on position error 
 *
 * This task is thin wrapper around the appropriate tasks in Tasks.
 *
 */
struct MC_TASKS_DLLAPI PositionBasedVisServoTask : public TrajectoryTaskGeneric<tasks::qp::PositionBasedVisServoTask>
{
public:
  /*! \brief Constructor
   *
   * \param bodyName Name of the body to control
   *
   * \param X_b_s Transformation from the controlled body to the surface being controlled
   *
   * \param X_t_s Transformation from the surface to the target
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
  PositionBasedVisServoTask(const std::string& bodyName,
    const sva::PTransformd& X_t_s, const sva::PTransformd& X_b_s,
    const mc_rbdyn::Robots & robots, unsigned int robotIndex,
    double stiffness = 2.0, double weight = 500);

  /*! \brief Constructor (from mc_rbdyn::Surface information)
   *
   * \param surfaceName Name of the surface the control
   *
   * \param X_t_s Transformation from the surface to the target
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
  PositionBasedVisServoTask(const std::string & surfaceName,
    const sva::PTransformd& X_t_s,
    const mc_rbdyn::Robots & robots, unsigned int robotIndex,
    double stiffness = 2.0, double weight = 500);

  /*! \brief Reset the task
   *
   * Set the task objective to the current body orientation
   */
  virtual void reset() override;

  /*! \brief Set the current error
   *
   * \param sva::PTransformd Desired configuration in camera frame
   *
   */
  void error(const sva::PTransformd& X_t_s);

  void addToLogger(mc_rtc::Logger & logger) override;
  void removeFromLogger(mc_rtc::Logger & logger) override;

private:
  sva::PTransformd X_t_s_;
};
}
