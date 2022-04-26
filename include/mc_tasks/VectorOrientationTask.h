/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace mc_tasks
{

/*! \brief Control the orientation of a vector attached to a frame
 *
 * This task is thin wrapper around the appropriate tasks in Tasks.
 *
 */
struct MC_TASKS_DLLAPI VectorOrientationTask : public TrajectoryTaskGeneric<tasks::qp::VectorOrientationTask>
{
  /*! \brief Constructor from a robot frame
   *
   * \param frame Control frame
   *
   * \param framveVector Vector to be controlled, experessed in the control frame
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  VectorOrientationTask(const mc_rbdyn::RobotFrame & frame,
                        const Eigen::Vector3d & frameVector,
                        double stiffness = 2.0,
                        double weight = 500.0);

  /*! \brief Constructor with user-specified target
   *
   * \param bodyName Name of the body to control
   *
   * \param bodyVector Vector to be controlled, expressed in the body frame
   *
   * \param targetVector Target for the controlled vector, expressed in the
   * world frame
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  VectorOrientationTask(const std::string & bodyName,
                        const Eigen::Vector3d & bodyVector,
                        const Eigen::Vector3d & targetVector,
                        const mc_rbdyn::Robots & robots,
                        unsigned int robotIndex,
                        double stiffness = 2.0,
                        double weight = 500);

  /*! \brief Constructor with default target */
  VectorOrientationTask(const std::string & bodyName,
                        const Eigen::Vector3d & bodyVector,
                        const mc_rbdyn::Robots & robots,
                        unsigned int robotIndex,
                        double stiffness = 2.0,
                        double weight = 500);

  /*! \brief Reset the task
   *
   * Set the task objective (i.e. target vector in the world frame) to the
   * current body vector
   */
  void reset() override;

  /*! \brief Set world target for the controlled vector
   *
   * \param vector Target vector in the world frame
   *
   */
  void targetVector(const Eigen::Vector3d & vector);

  /**
   * @brief Get the target orientation
   *
   * @return The target orientation in world frame
   */
  Eigen::Vector3d targetVector() const;

  /**
   * @brief Get the current body orientation
   *
   * @return The current body orientation vector in world frame
   */
  Eigen::Vector3d actual() const;

  /*! \brief Return the controlled body */
  std::string body()
  {
    return frame_->body();
  }

  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
  void addToLogger(mc_rtc::Logger & logger) override;

protected:
  mc_rbdyn::ConstRobotFramePtr frame_;
};

} // namespace mc_tasks
