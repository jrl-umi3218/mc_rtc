/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace mc_tasks
{

/*! \brief Control a robot's CoM
 *
 * This task is a thin wrapper above the appropriate Tasks types
 *
 */
struct MC_TASKS_DLLAPI CoMTask : public TrajectoryTaskGeneric<tasks::qp::CoMTask>
{
public:
  /*! \brief Constructor
   *
   * \param robots Robots involved in the task
   *
   * \param robotIndex Select the robot which CoM should be controlled
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  CoMTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness = 5.0, double weight = 100.);

  virtual void reset() override;

  /*! \brief Change the CoM target by a given amount
   *
   * \param com Modification applied to the CoM
   *
   */
  void move_com(const Eigen::Vector3d & com);

  /*! \brief Set the CoM target to a given position
   *
   * \param com New CoM target
   *
   */
  void com(const Eigen::Vector3d & com);

  /*! \brief Return the current CoM target
   *
   * \returns The current CoM target
   */
  Eigen::Vector3d com();

protected:
  void addToGUI(mc_rtc::gui::StateBuilder &) override;
  void addToLogger(mc_rtc::Logger & logger) override;
  void removeFromLogger(mc_rtc::Logger & logger) override;

private:
  unsigned int robot_index_;
  Eigen::Vector3d cur_com_;
};

} // namespace mc_tasks
