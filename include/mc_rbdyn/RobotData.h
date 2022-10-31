/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <memory>

namespace mc_rbdyn
{

/** Hold data and objects that are shared among different views of a robot (control/real/output)
 *
 * The framework enforce consistency between these views
 *
 * This includes:
 * - reference joint order
 * - encoder level readings (position/velocity/torques)
 * - force/body/joint sensors
 * - grippers
 * - devices
 */
struct RobotData
{
};

using RobotDataPtr = std::shared_ptr<RobotData>;

} // namespace mc_rbdyn
