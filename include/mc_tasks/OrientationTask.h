/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace mc_tasks
{

/*! \brief Control the orientation of a body
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
   * Set the task objective to the current body orientation
   */
  virtual void reset() override;

  /*! \brief Set the body orientation target
   *
   * \param ori Body orientation in world frame
   *
   */
  void orientation(const Eigen::Matrix3d & ori);

  /*! \brief Get the current body orientation target
   *
   * \returns The body orientation target in world frame
   *
   */
  Eigen::Matrix3d orientation();

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

public:
  std::string bodyName;
  unsigned int bIndex;
  virtual void addToLogger(mc_rtc::Logger & logger) override;
  virtual void removeFromLogger(mc_rtc::Logger & logger) override;
};

} // namespace mc_tasks
