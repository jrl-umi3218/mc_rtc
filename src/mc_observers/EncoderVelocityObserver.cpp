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

#include <mc_rbdyn/rpy_utils.h>
#include "EncoderVelocityObserver.h"

namespace mc_observers
{
EncoderVelocityObserver::EncoderVelocityObserver(const std::string& name, double dt, const mc_rtc::Configuration & config) :
    Observer(name, dt, config)
{
  LOG_SUCCESS("EncoderVelocityObserver created")
}

void EncoderVelocityObserver::reset(const mc_rbdyn::Robot & controlRobot, const mc_rbdyn::Robot & realRobot)
{
  LOG_SUCCESS("EncoderVelocityObserver reset");
}

bool EncoderVelocityObserver::run(const mc_rbdyn::Robot & controlRobot, const mc_rbdyn::Robot & realRobot)
{
  return true;
}

void EncoderVelocityObserver::updateRobot(mc_rbdyn::Robot & realRobot)
{
}

void EncoderVelocityObserver::addToLogger(mc_rtc::Logger &logger)
{
}
void EncoderVelocityObserver::removeFromLogger(mc_rtc::Logger &logger)
{
}
void EncoderVelocityObserver::addToGUI(mc_rtc::gui::StateBuilder &gui)
{
}
void EncoderVelocityObserver::removeFromGUI(mc_rtc::gui::StateBuilder &gui)
{
}

} // namespace mc_observers

EXPORT_OBSERVER_MODULE("EncoderVelocity", mc_observers::EncoderVelocityObserver)
