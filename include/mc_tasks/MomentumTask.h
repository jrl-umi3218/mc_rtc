/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

namespace mc_tasks
{

/*! \brief Control the momentum of a robot

 * This task is thin wrapper around the appropriate tasks in Tasks.
 *
 */
struct MC_TASKS_DLLAPI MomentumTask : public TrajectoryTaskGeneric<tasks::qp::MomentumTask>
{
  using TrajectoryBase = TrajectoryTaskGeneric<tasks::qp::MomentumTask>;

public:
  /*! \brief Constructor
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
  MomentumTask(const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness = 2.0, double weight = 500);

  /*! \brief Reset the task
   *
   * Set the task target to the current momentum and reset desired velocity and acceleration to zero.
   *
   */
  void reset() override;

  /*! \brief Get the current target momentum */
  sva::ForceVecd momentum() const;

  /*! \brief Set the target momentum
   *
   * \param target Target momentum
   *
   */
  void momentum(const sva::ForceVecd & mom);

  void addToLogger(mc_rtc::Logger & logger) override;

  /*! \brief Load parameters from a Configuration object */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;
};

} // namespace mc_tasks
