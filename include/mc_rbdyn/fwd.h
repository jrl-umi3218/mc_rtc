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

struct RobotFrame;
using RobotFramePtr = std::shared_ptr<RobotFrame>;
using ConstRobotFramePtr = std::shared_ptr<const RobotFrame>;

struct Frame;
using FramePtr = std::shared_ptr<Frame>;
using ConstFramePtr = std::shared_ptr<const Frame>;

struct ForceSensor;

} // namespace mc_rbdyn
