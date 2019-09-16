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

#include <mc_observers/FloatingBasePosObserver.h>
#include <mc_rbdyn/rpy_utils.h>

namespace mc_observers
{
FloatingBasePosObserver::FloatingBasePosObserver(const mc_rbdyn::Robot & controlRobot) :
    controlRobot_(controlRobot),
    orientation_(Eigen::Matrix3d::Identity()),
    position_(Eigen::Vector3d::Zero()),
    leftFootRatio_(0.5)
{
}

void FloatingBasePosObserver::reset(const mc_rbdyn::Robot & realRobot)
{
  run(realRobot);
}

void FloatingBasePosObserver::run(const mc_rbdyn::Robot & realRobot)
{
  estimateOrientation(realRobot);
  estimatePosition(realRobot);
}

void FloatingBasePosObserver::estimateOrientation(const mc_rbdyn::Robot & realRobot)
{
  // Prefixes:
  // c for control-robot model
  // r for real-robot model
  // m for estimated/measured quantities
  sva::PTransformd X_0_rBase = realRobot.posW();
  sva::PTransformd X_0_rIMU = realRobot.bodyPosW(realRobot.bodySensor().parentBody());
  sva::PTransformd X_rIMU_rBase = X_0_rBase * X_0_rIMU.inv();
  Eigen::Matrix3d E_0_mIMU = realRobot.bodySensor().orientation().toRotationMatrix();
  Eigen::Matrix3d E_0_cBase = controlRobot_.posW().rotation();
  Eigen::Matrix3d E_0_mBase = X_rIMU_rBase.rotation() * E_0_mIMU;
  Eigen::Vector3d cRPY = mc_rbdyn::rpyFromMat(E_0_cBase);
  Eigen::Vector3d mRPY = mc_rbdyn::rpyFromMat(E_0_mBase);
  orientation_ = mc_rbdyn::rpyToMat(mRPY(0), mRPY(1), cRPY(2));
}

void FloatingBasePosObserver::estimatePosition(const mc_rbdyn::Robot & realRobot)
{
  sva::PTransformd X_0_c = getAnchorFrame(controlRobot_);
  sva::PTransformd X_0_s = getAnchorFrame(realRobot);
  const sva::PTransformd & X_0_real = realRobot.posW();
  sva::PTransformd X_real_s = X_0_s * X_0_real.inv();
  const Eigen::Vector3d & r_c_0 = X_0_c.translation();
  const Eigen::Vector3d & r_s_real = X_real_s.translation();
  position_ = r_c_0 - orientation_.transpose() * r_s_real;
}

sva::PTransformd FloatingBasePosObserver::getAnchorFrame(const mc_rbdyn::Robot & robot)
{
  sva::PTransformd X_0_l = robot.surface("LeftFoot").X_0_s(robot);
  sva::PTransformd X_0_r = robot.surface("RightFoot").X_0_s(robot);
  return sva::interpolate(X_0_r, X_0_l, leftFootRatio_);
}

void FloatingBasePosObserver::updateRobot(mc_rbdyn::Robot & realRobot)
{
  realRobot.posW(sva::PTransformd{orientation_, position_});
}

void FloatingBasePosObserver::updateBodySensor(mc_rbdyn::Robot & realRobot, const std::string & sensorName)
{
  auto & sensor = realRobot.bodySensor(sensorName);
  sensor.position(position_);
  sensor.orientation(Eigen::Quaterniond(orientation_));
}
} // namespace mc_observers
