/* Copyright 2018-2019 CNRS-UM LIRMM
 *
 * \author Stéphane Caron
 *
 * This file is part of lipm_walking_controller.
 *
 * lipm_walking_controller is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * lipm_walking_controller is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with lipm_walking_controller. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "EncoderObserver.h"

#include <mc_rbdyn/rpy_utils.h>

namespace mc_observers
{
EncoderObserver::EncoderObserver(const std::string & name, double dt, const mc_rtc::Configuration & config)
: Observer(name, dt, config)
{
  posFromSensor_ = config("PositionFromSensor", true);
  posFromControl_ = config("PositionFromControl", false);

  velFromSensor_ = config("VelocityFromSensor", true);
  velFromEstimation_ = config("VelocityFromEstimation", false);
  velFromControl_ = config("VelocityFromControl", false);

  logEstimation_ = config("LogEstimation", false);

  LOG_SUCCESS("EncoderObserver created")
}

void EncoderObserver::reset(const mc_rbdyn::Robot & realRobot)
{
  LOG_SUCCESS("EncoderObserver reset");
  const auto & enc = robot().encoderValues();
  if(enc.empty())
  {
    LOG_WARNING("[EncoderObserver] Trying to reset observer from a robot with no encoder values available");
  }
  else
  {
    prevEncoders_ = enc;
    encodersVelocity_.resize(enc.size());
    for(unsigned i = 0; i < enc.size(); ++i)
    {
      encodersVelocity_[i] = 0;
    }
  }
}

bool EncoderObserver::run(const mc_rbdyn::Robot & realRobot)
{
  // TODO Estimate encoder velocity

  const auto & enc = robot().encoderValues();
  if(enc.empty()) return false;
  for(unsigned i = 0; i < enc.size(); ++i)
  {
    encodersVelocity_[i] = (enc[i] - prevEncoders_[i]) / dt();
    prevEncoders_[i] = enc[i];
  }
  return true;
}

void EncoderObserver::updateRobot(mc_rbdyn::Robot & realRobot)
{
  const auto & q = robot().encoderValues();
  const auto & alpha = robot().encoderVelocities();

  if(q.size() == robot().refJointOrder().size())
  {
    // Set all joint values and velocities from encoders
    unsigned i = 0;
    for(const auto & ref_joint : realRobot.refJointOrder())
    {
      if(robot().hasJoint(ref_joint))
      {
        const auto joint_index = static_cast<size_t>(robot().mb().jointIndexByName(ref_joint));
        if(posFromControl_)
        {
          realRobot.mbc().q[joint_index][0] = robot().mbc().q[joint_index][0];
        }
        if(posFromSensor_)
        {
          if(!q.empty())
          {
            realRobot.mbc().q[joint_index][0] = q[i];
          }
        }

        if(velFromControl_)
        {
          realRobot.mbc().alpha[joint_index][0] = robot().mbc().alpha[joint_index][0];
        }
        if(velFromSensor_)
        {
          if(!alpha.empty())
          {
            realRobot.mbc().alpha[joint_index][0] = alpha[i];
          }
        }
        if(velFromEstimation_)
        {
          if(!encodersVelocity_.empty())
          {
            realRobot.mbc().alpha[joint_index][0] = encodersVelocity_[i];
          }
        }
      }
      i++;
    }
  }
  if(posFromSensor_ || posFromControl_)
  {
    realRobot.forwardKinematics();
  }
  if(velFromControl_ || velFromSensor_ || velFromEstimation_)
  {
    realRobot.forwardVelocity();
  }
}

void EncoderObserver::addToLogger(mc_rtc::Logger & logger)
{
  if(logEstimation_)
  {
    logger.addLogEntry("observer_" + name() + "_alpha", [this]() { return encodersVelocity_; });
  }
}
void EncoderObserver::removeFromLogger(mc_rtc::Logger & logger)
{
  if(logEstimation_)
  {
    logger.removeLogEntry("observer_" + name() + "_alpha");
  }
}
void EncoderObserver::addToGUI(mc_rtc::gui::StateBuilder & gui) {}
void EncoderObserver::removeFromGUI(mc_rtc::gui::StateBuilder & gui) {}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("Encoder", mc_observers::EncoderObserver)
