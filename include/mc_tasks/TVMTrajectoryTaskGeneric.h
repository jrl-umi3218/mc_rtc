/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/tasks_traits.h>

#include <mc_solver/TVMQPSolver.h>

#include <tvm/ControlProblem.h>
#include <tvm/task_dynamics/Proportional.h>
#include <tvm/task_dynamics/ProportionalDerivative.h>

namespace mc_tasks
{

namespace details
{

template<typename ErrorT>
void set_ref_vel(void * task, const Eigen::VectorXd & refVel)
{
  static_cast<ErrorT *>(task)->refVel(refVel);
}

template<typename ErrorT>
void set_ref_accel(void * task, const Eigen::VectorXd & refAccel)
{
  static_cast<ErrorT *>(task)->refAccel(refAccel);
}

struct TVMTrajectoryTaskGeneric
{
  tvm::TaskWithRequirementsPtr task_; // null if the task is not in solver
  bool dynamicIsPD_ = false; // is task_ dynamic a PD or a P, irrelevant if task_ is null
  Eigen::VectorXd dimWeight_;

  using set_ref_vel_t = void (*)(void *, const Eigen::VectorXd &);
  using set_ref_accel_t = void (*)(void *, const Eigen::VectorXd &);

  set_ref_vel_t setRefVel = nullptr;
  set_ref_accel_t setRefAccel = nullptr;

  template<typename ErrorT>
  void init(ErrorT * error)
  {
    dimWeight_ = Eigen::VectorXd::Ones(error->size());
    if constexpr(details::has_refVel_v<ErrorT>) { setRefVel = set_ref_vel<ErrorT>; }
    if constexpr(details::has_refAccel_v<ErrorT>) { setRefAccel = set_ref_accel<ErrorT>; }
  }
};

using TVMTrajectoryTaskGenericPtr = std::shared_ptr<TVMTrajectoryTaskGeneric>;

} // namespace details

} // namespace mc_tasks
