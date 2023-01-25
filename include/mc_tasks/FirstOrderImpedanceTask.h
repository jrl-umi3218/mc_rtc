/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/ImpedanceTask.h>

namespace mc_tasks
{

namespace force
{

/*! \brief Impedance-based damping control of the end-effector. */
struct MC_TASKS_DLLAPI FirstOrderImpedanceTask : ImpedanceTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*! \brief Constructor.
      \param surfaceName name of the surface frame to control
      \param robots robots controlled by this task
      \param robotIndex index of the robot controlled by this task
      \param stiffness task stiffness
      \param weight task weight

      \throws If the body the task is attempting to control does not have a sensor attached to it
   */
  FirstOrderImpedanceTask(const std::string & surfaceName,
                          const mc_rbdyn::Robots & robots,
                          unsigned robotIndex,
                          double stiffness = 5.0,
                          double weight = 1000.0);

  /*! \brief Constructor.
      \param frame Frame controlled by this task
      \param stiffness task stiffness
      \param weight task weight

      \throws If the body the task is attempting to control does not have a sensor attached to it
   */
  FirstOrderImpedanceTask(const mc_rbdyn::RobotFrame & Frame, double stiffness = 5.0, double weight = 1000.0);

  /*! \brief Update task. */
  void update(mc_solver::QPSolver & solver) override;
};

} // namespace force

} // namespace mc_tasks
