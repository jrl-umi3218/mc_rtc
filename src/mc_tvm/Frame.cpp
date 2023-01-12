/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_tvm/Frame.h>

namespace mc_tvm
{

Frame::Frame(NewFrameToken, const mc_rbdyn::Frame & frame) : frame_(frame)
{
  // clang-format off
  registerUpdates(
                  Update::Position, &Frame::updatePosition,
                  Update::Velocity, &Frame::updateVelocity);
  // clang-format off

  addOutputDependency(Output::Position, Update::Position);
  addOutputDependency(Output::Velocity, Update::Velocity);
}

void Frame::updatePosition()
{
  position_ = frame_.position();
}

void Frame::updateVelocity()
{
  velocity_ = frame_.velocity();
}

} // namespace mc_tvm
