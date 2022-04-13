/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Frame.h>

namespace mc_rbdyn
{

/** A frame that belongs to a Robot instance
 *
 * The frame quantities are deduced from the robot's state
 *
 */
struct MC_RBDYN_DLLAPI RobotFrame : public mc_rtc::shared<RobotFrame, Frame>
{
protected:
  struct NewRobotFrameToken : public Frame::NewFrameToken
  {
  };

public:
  friend struct Robot;

  /** Constructor
   *
   * Creates a frame attached to the given body
   *
   * \param name Name of the new frame
   *
   * \param robot Robot to which the frame is attached
   *
   * \param body Parent body of the frame
   *
   * \throws If \p body does not belong to \p robot
   */
  RobotFrame(NewRobotFrameToken, const std::string & name, Robot & robot, const std::string & body);

  /** Constructor
   *
   * Creates a frame attached to the given frame
   *
   * \param name Name of the new frame
   *
   * \param frame Parent frame
   *
   * \param X_p_f Transform from parent to the new frame
   *
   * \param baked If true, attach the newly created frame to the parent body of the provided frame rather than the frame
   * itself
   */
  RobotFrame(NewRobotFrameToken, const std::string & name, RobotFrame & parent, sva::PTransformd X_p_f, bool baked);

  /** The robot to which this frame belongs (const) */
  inline const Robot & robot() const noexcept
  {
    return robot_;
  }

  /** The robot to which this frame belongs */
  inline Robot & robot() noexcept
  {
    return robot_;
  }

  /** The body this frame is attached to */
  const std::string & body() const noexcept;

  /** Computes the frame position */
  sva::PTransformd position() const noexcept;

  /** Computes the frame velocity */
  sva::MotionVecd velocity() const noexcept;

  /** Returns the transformation from the parent's frame/body to the frame */
  inline const sva::PTransformd & X_p_f() const noexcept
  {
    return position_;
  }

  /** Set the transformation from the parent's frame/body to this frame
   *
   * \note This impacts children frame
   *
   * \note It is technically possible to change body frames origin this way but it will create a discrepency between the
   * data in MultiBodyConfig and the data in the frame
   */
  inline RobotFrame & X_p_f(sva::PTransformd pt) noexcept
  {
    position_ = pt;
    return *this;
  }

  /** True if the frame has a force sensor (direct or indirect) attached */
  inline bool hasForceSensor() const noexcept
  {
    return sensor_;
  }

  /** Returns the force sensor attached to the frame (const)
   *
   * \throws if \ref hasForceSensor() is false
   */
  const ForceSensor & forceSensor() const;

  /** Returns the force sensor gravity-free wrench in this frame
   *
   * \throws if \ref hasForceSensor() is false
   */
  sva::ForceVecd wrench() const;

  /** Create a frame whose parent is this frame
   *
   * \param name Name of the new frame
   *
   * \param X_p_f Transformation from this frame to the newly created frame
   *
   * \param baked Attach the newly created frame to the parent body of this frame rather than the frame
   */
  RobotFramePtr makeFrame(const std::string & name, const sva::PTransformd & X_p_f, bool baked = false);

protected:
  /** Robot instance this frame belongs to */
  Robot & robot_;
  /** Parent's body index in the robot's configuration */
  unsigned int bodyMbcIdx_;
  /** Force sensor attached (directly or indirectly) to this frame, nullptr if none */
  const ForceSensor * sensor_ = nullptr;
};

} // namespace mc_rbdyn
