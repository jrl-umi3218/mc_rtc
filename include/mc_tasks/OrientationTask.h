/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace mc_tasks
{

/*! \brief Control the orientation of a frame
 *
 * This task is thin wrapper around the appropriate tasks in Tasks.
 *
 */
struct MC_TASKS_DLLAPI OrientationTask : public TrajectoryTaskGeneric<tasks::qp::OrientationTask>
{
public:
  friend struct EndEffectorTask;

  /*! \brief Constructor
   *
   * \param frame Control frame
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  OrientationTask(const mc_rbdyn::RobotFrame & frame, double stiffness = 2.0, double weight = 500.0);

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
  OrientationTask(const std::string & bodyName,
                  const mc_rbdyn::Robots & robots,
                  unsigned int robotIndex,
                  double stiffness = 2.0,
                  double weight = 500);

  /*! \brief Reset the task
   *
   * Set the task objective to the current frame orientation
   */
  void reset() override;

  /*! \brief Set the frame orientation target
   *
   * \param ori Body orientation in world frame
   *
   */
  void orientation(const Eigen::Matrix3d & ori);

  /*! \brief Get the current frame orientation target
   *
   * \returns The body orientation target in world frame
   *
   */
  Eigen::Matrix3d orientation();

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

public:
  mc_rbdyn::ConstRobotFramePtr frame_;
  void addToLogger(mc_rtc::Logger & logger) override;
};

} // namespace mc_tasks
