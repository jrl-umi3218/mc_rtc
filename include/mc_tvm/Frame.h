/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tvm/api.h>
#include <mc_tvm/fwd.h>

#include <mc_rbdyn/Frame.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <RBDyn/Jacobian.h>

#include <tvm/graph/abstract/Node.h>
#include <tvm/internal/MatrixWithProperties.h>

namespace mc_tvm
{

/** Represent a frame
 *
 * It is created through an \ref mc_rbdyn::Frame
 *
 * It caches (for the duration of a TVM graph update) the position and velocity of the related frame
 *
 * Outputs:
 * - Position: position of the frame in world coordinates
 * - Velocity: velocity of the frame in world coordinates
 *
 */
struct MC_TVM_DLLAPI Frame : public tvm::graph::abstract::Node<Frame>
{
  SET_OUTPUTS(Frame, Position, Velocity)
  SET_UPDATES(Frame, Position, Velocity)

  friend struct mc_rbdyn::Frame;

protected:
  struct NewFrameToken
  {
  };

public:
  Frame(NewFrameToken, const mc_rbdyn::Frame & frame);

  /** The frame's position in world coordinates */
  inline const sva::PTransformd & position() const noexcept { return position_; }

  /** The frame's velocity in world coordinates */
  inline const sva::MotionVecd & velocity() const noexcept { return velocity_; }

  /** Returns the associated mc_rbdyn Frame */
  inline const mc_rbdyn::Frame & frame() const noexcept { return frame_; }

protected:
  /** Parent instance */
  const mc_rbdyn::Frame & frame_;
  /* Cache and update functions */
  sva::PTransformd position_ = sva::PTransformd::Identity();
  sva::MotionVecd velocity_ = sva::MotionVecd::Zero();

  void updatePosition();
  void updateVelocity();
};

} // namespace mc_tvm
