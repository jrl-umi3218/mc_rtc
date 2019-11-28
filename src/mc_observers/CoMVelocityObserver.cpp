/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is  inspired by Stephane's Caron implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#include <mc_control/mc_controller.h>
#include <mc_observers/CoMVelocityObserver.h>
#include <mc_rbdyn/rpy_utils.h>

namespace mc_observers
{
CoMVelocityObserver::CoMVelocityObserver(const std::string & name,
                                         double dt,
                                         const mc_rtc::Configuration & /* config */)
: Observer(name, dt), comVelFilter_(dt, /* cutoff period = */ 0.01)
{
}

void CoMVelocityObserver::reset(const mc_control::MCController & ctl)
{
  comVelFilter_.reset(ctl.robot().com(), ctl.robot().comVelocity());
  realComd_.setZero();
}

bool CoMVelocityObserver::run(const mc_control::MCController & ctl)
{
  comVelFilter_.update(ctl.realRobot().com());
  realComd_ = comVelFilter_.eval();
  return true;
}

void CoMVelocityObserver::updateRobots(const mc_control::MCController & /* ctl */, mc_rbdyn::Robots & /* realRobots */)
{
}

void CoMVelocityObserver::addToLogger(const mc_control::MCController &, mc_rtc::Logger & logger)
{
  logger.addLogEntry("observer_" + name() + "_comd", [this]() { return realComd_; });
  logger.addLogEntry("observer_" + name() + "_filter_cutoffPeriod", [this]() { return comVelFilter_.cutoffPeriod(); });
}
void CoMVelocityObserver::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry("observer_" + name() + "_comd");
  logger.removeLogEntry("observer_" + name() + "_filter_cutoffPeriod");
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("CoMVelocity", mc_observers::CoMVelocityObserver)
