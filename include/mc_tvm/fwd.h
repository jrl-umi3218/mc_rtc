/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <memory>

#pragma once

namespace mc_tvm
{

struct CoM;
using CoMPtr = std::unique_ptr<CoM>;

struct Convex;
using ConvexPtr = std::unique_ptr<Convex>;

struct Frame;
using FramePtr = std::unique_ptr<Frame>;

struct Momentum;
using MomentumPtr = std::unique_ptr<Momentum>;

struct Robot;
using RobotPtr = std::unique_ptr<Robot>;

struct RobotFrame;
using RobotFramePtr = std::unique_ptr<RobotFrame>;

} // namespace mc_tvm
