/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/api.h>
#include <mc_rbdyn/fwd.h>

#include <mc_rtc/shared.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rbdyn
{

/** Represents a frame
 *
 * This frame is not necessarily associated to a robot as \ref RobotFrame
 *
 * Its position/velocity/acceleration properties are user-provided
 *
 */
struct MC_RBDYN_DLLAPI Frame : public mc_rtc::shared<Frame>
{
protected:
  struct NewFrameToken
  {
  };

public:
  /** Constructor for a frame with no parent */
  Frame(NewFrameToken, const std::string & name) noexcept;

  /** Constructor for a frame with a parent */
  Frame(NewFrameToken, const std::string & name, Frame & parent, sva::PTransformd X_p_f, bool bake) noexcept;

  virtual ~Frame() = default;

  /* Prevent copy and move */
  Frame(const Frame &) = delete;
  Frame(Frame &&) = delete;
  Frame & operator=(const Frame &) = delete;
  Frame & operator=(Frame &&) = delete;

  /** Creates a new frame with no parent
   *
   * \param name Name of the frame
   */
  inline static FramePtr make(const std::string & name) noexcept
  {
    return std::make_shared<Frame>(NewFrameToken{}, name);
  }

  /** Creates a new frame with a parent. The frame is fixed relative to its parent
   *
   * If \p parent is a \ref RobotFrame then this function returns a \ref RobotFrame
   *
   * \param name Name of the new frame
   *
   * \param parent Parent frame
   *
   * \param X_p_f Transformation from the parent to the frame
   *
   * \param baked If true, don't keep track of the parent in the newly created frame
   *
   * \throws If \p parent is actually a \ref RobotFrame this creates a new frame \p name in \ref RoboyFrame, this throws
   * if \p name is alredy a frame in the robot
   *
   */
  static FramePtr make(const std::string & name, Frame & parent, sva::PTransformd X_p_f, bool baked);

  /** Return the frame's name */
  inline const std::string & name() const noexcept
  {
    return name_;
  }

  /** Compute the current (6D) frame's position in inertial frame */
  virtual inline sva::PTransformd position() const noexcept
  {
    return parent_ ? position_ * parent_->position() : position_;
  }

  /** Compute the current frame's velocity in inertial frame */
  virtual inline sva::MotionVecd velocity() const noexcept
  {
    return parent_ ? position_ * parent_->velocity() : velocity_;
  }

  /** Update the frame's position
   *
   * If this frame has no parent, this sets its absolute position.
   *
   * Othewise, it sets the relative transformation from it's parent to itself
   */
  inline Frame & position(sva::PTransformd pos) noexcept
  {
    position_ = pos;
    return *this;
  }

  /** Update the frame's velocity
   *
   * If this frame has no parent this sets its absolute velocity.
   *
   * Otherwise this is a no-op
   */
  inline Frame & velocity(sva::MotionVecd velocity) noexcept
  {
    velocity_ = velocity;
    return *this;
  }

  /** Access the parent's frame (might be nullptr) */
  inline const FramePtr & parent() const noexcept
  {
    return parent_;
  }

protected:
  /** Name of the frame */
  std::string name_;
  /** Direct parent of the frame (if any) */
  FramePtr parent_ = nullptr;
  /** Absolute position for a frame with no parent, relative position otherwise */
  sva::PTransformd position_ = sva::PTransformd::Identity();
  /** Absolute velocity for a frame with no parent, unused otherwise */
  sva::MotionVecd velocity_ = sva::MotionVecd::Zero();
};

} // namespace mc_rbdyn
