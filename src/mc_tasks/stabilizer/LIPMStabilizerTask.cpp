/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#include <mc_tasks/stabilizer/LIPMStabilizerTask.h>

namespace mc_tasks
{
namespace stabilizer
{
// Repeat static constexpr declarations
// Fixes https://github.com/stephane-caron/lipm_walking_controller/issues/21
// See also https://stackoverflow.com/q/8016780
constexpr double LIPMStabilizerTask::MAX_AVERAGE_DCM_ERROR;
constexpr double LIPMStabilizerTask::MAX_COM_ADMITTANCE;
constexpr double LIPMStabilizerTask::MAX_COP_ADMITTANCE;
constexpr double LIPMStabilizerTask::MAX_DCM_D_GAIN;
constexpr double LIPMStabilizerTask::MAX_DCM_I_GAIN;
constexpr double LIPMStabilizerTask::MAX_DCM_P_GAIN;
constexpr double LIPMStabilizerTask::MAX_DFZ_ADMITTANCE;
constexpr double LIPMStabilizerTask::MAX_DFZ_DAMPING;
constexpr double LIPMStabilizerTask::MAX_FDC_RX_VEL;
constexpr double LIPMStabilizerTask::MAX_FDC_RY_VEL;
constexpr double LIPMStabilizerTask::MAX_FDC_RZ_VEL;
constexpr double LIPMStabilizerTask::MAX_ZMPCC_COM_OFFSET;
constexpr double LIPMStabilizerTask::MIN_DS_PRESSURE;

LIPMStabilizerTask::LIPMStabilizerTask(const mc_rbdyn::Robots & robots,
                                       unsigned int robotIndex,
                                       const std::string & leftFootSurface,
                                       const std::string & rightFootSurface)
: robots_(robots), robotIndex_(robotIndex)
{
  type_ = "lipm_stabilizer";
  name_ = "lipm_stabilizer";
}

LIPMStabilizerTask::~LIPMStabilizerTask() {}

void LIPMStabilizerTask::reset() {}

void LIPMStabilizerTask::dimWeight(const Eigen::VectorXd & dim)
{
  LOG_ERROR_AND_THROW(std::runtime_error, "dimWeight not implemented for task " << type_);
}

Eigen::VectorXd LIPMStabilizerTask::dimWeight() const
{
  LOG_ERROR_AND_THROW(std::runtime_error, "dimWeight not implemented for task " << type_);
}

void LIPMStabilizerTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                            const std::vector<std::string> & activeJointsName,
                                            const std::map<std::string, std::vector<std::array<int, 2>>> & activeDofs)
{
  comTask->selectActiveJoints(solver, activeJointsName, activeDofs);
  leftFootTask->selectActiveJoints(solver, activeJointsName, activeDofs);
  rightFootTask->selectActiveJoints(solver, activeJointsName, activeDofs);
}

void LIPMStabilizerTask::selectUnactiveJoints(
    mc_solver::QPSolver & solver,
    const std::vector<std::string> & unactiveJointsName,
    const std::map<std::string, std::vector<std::array<int, 2>>> & unactiveDofs)
{
  comTask->selectUnactiveJoints(solver, unactiveJointsName, unactiveDofs);
  leftFootTask->selectUnactiveJoints(solver, unactiveJointsName, unactiveDofs);
  rightFootTask->selectUnactiveJoints(solver, unactiveJointsName, unactiveDofs);
}

void LIPMStabilizerTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  comTask->resetJointsSelector(solver);
  leftFootTask->resetJointsSelector(solver);
  rightFootTask->resetJointsSelector(solver);
}

Eigen::VectorXd LIPMStabilizerTask::eval() const
{
  LOG_ERROR_AND_THROW(std::runtime_error, "eval not implemented for task " << type_);
}

/*! \brief Returns the task velocity
 *
 * The vector's dimensions depend on the underlying task
 *
 */
Eigen::VectorXd LIPMStabilizerTask::speed() const
{
  LOG_ERROR_AND_THROW(std::runtime_error, "speed not implemented for task " << type_);
}

void LIPMStabilizerTask::load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config)
{
  LOG_ERROR_AND_THROW(std::runtime_error, "load not implemented for task " << type_);
}

void LIPMStabilizerTask::addToSolver(mc_solver::QPSolver & solver)
{
  MetaTask::addToSolver(*comTask, solver);
  MetaTask::addToSolver(*leftFootTask, solver);
  MetaTask::addToSolver(*rightFootTask, solver);
}

void LIPMStabilizerTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  MetaTask::removeFromSolver(*comTask, solver);
  MetaTask::removeFromSolver(*leftFootTask, solver);
  MetaTask::removeFromSolver(*rightFootTask, solver);
}

void LIPMStabilizerTask::update()
{
  MetaTask::update(*comTask);
  MetaTask::update(*leftFootTask);
  MetaTask::update(*rightFootTask);
}

void LIPMStabilizerTask::addToLogger(mc_rtc::Logger &) {}

void LIPMStabilizerTask::removeFromLogger(mc_rtc::Logger &) {}

void LIPMStabilizerTask::addToGUI(mc_rtc::gui::StateBuilder &) {}

void LIPMStabilizerTask::removeFromGUI(mc_rtc::gui::StateBuilder &) {}

} // namespace stabilizer
} // namespace mc_tasks
