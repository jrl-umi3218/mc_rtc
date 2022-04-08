/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

/** Forward declarations for the mc_rbdyn library */

#pragma once

#include <memory>

namespace mc_rbdyn
{

struct Base;

struct Robots;
using RobotsPtr = std::shared_ptr<Robots>;

struct Robot;
using RobotPtr = std::shared_ptr<Robot>;
using ConstRobotPtr = std::shared_ptr<const Robot>;

} // namespace mc_rbdyn
