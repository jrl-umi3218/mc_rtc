/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/LookAtTask.h>

namespace mc_tasks
{

/*! \brief Track a frame position with a "gaze" vector.
 * This task is a convenience wrapper for LookAtTask that takes care of
 * automatically updating the gaze target.
 */
struct MC_TASKS_DLLAPI LookAtFrameTask : public LookAtTask
{
public:
  /*! \brief Constructor
   *
   * \param frame Control frame
   *
   * \param frameVector Frame vector in control frame
   *
   * \param targetFrame Frame that is looked at
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  LookAtFrameTask(const mc_rbdyn::RobotFrame & frame,
                  const Eigen::Vector3d & frameVector,
                  const mc_rbdyn::Frame & target,
                  double stiffness = 0.5,
                  double weight = 200.0);

  /*! \brief Constructor
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param bodyName Name of the body to control
   *
   * \param bodyVector Gaze vector for the body.
        For instance [1., 0, 0] will try to align the x axis of the body with the target direction
   *
   * \param surfaceRobotIndex Robot the target surface belongs to
   *
   * \param surfaceNamed Tracked surface
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  LookAtFrameTask(const mc_rbdyn::Robots & robots,
                  unsigned int robotIndex,
                  const std::string & bodyName,
                  const Eigen::Vector3d & bodyVector,
                  unsigned int surfaceRobotIndex,
                  const std::string & surfaceName,
                  double stiffness = 0.5,
                  double weight = 200);

  /*! \brief Update the gaze target from TF position */
  void update(mc_solver::QPSolver &) override;

  /*! \brief Access offset relative to the surface */
  inline const sva::PTransformd & offset() const noexcept
  {
    return offset_;
  }

  /*! \brief Set the offset relative to the surface */
  inline void offset(const sva::PTransformd & off) noexcept
  {
    offset_ = off;
  }

  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  /*! Target frame */
  mc_rbdyn::ConstFramePtr target_;
  /*! Offset to the surface in surface frame */
  sva::PTransformd offset_ = sva::PTransformd::Identity();
};

} // namespace mc_tasks
