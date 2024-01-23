/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_tasks/TrajectoryTaskGeneric.h>

#include <mc_rbdyn/RobotFrame.h>

namespace mc_tasks
{

/*! \brief Control a frame 6D pose */
struct MC_TASKS_DLLAPI TransformTask : public TrajectoryTaskGeneric
{
public:
  /*! \brief Constructor
   *
   * \param frame Frame controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   */
  TransformTask(const mc_rbdyn::RobotFrame & frame, double stiffness = 2.0, double weight = 500.0);

  /*! \brief Constructor
   *
   * Prefer the frame-based constructor
   *
   * \param surfaceName Name of the surface frame to control
   *
   * \param robots Robots controlled by this task
   *
   * \param robotIndex Index of the robot controlled by this task
   *
   * \param stiffness Task stiffness
   *
   * \param weight Task weight
   *
   */
  TransformTask(const std::string & surfaceName,
                const mc_rbdyn::Robots & robots,
                unsigned int robotIndex,
                double stiffness = 2.0,
                double weight = 500);

  /*! \brief Reset the task
   *
   * Set the task target to the current frame position
   *
   * Reset its target velocity and acceleration to zero.
   *
   */
  void reset() override;

  /*! \brief Get the task's target */
  virtual sva::PTransformd target() const;

  /*! \brief Set the task's target
   *
   * \param pos Target in world frame
   *
   */
  virtual void target(const sva::PTransformd & worldPos);

  /*! \brief Get the task's target velocity
   *
   * \param vel Target velocity in world frame
   */
  virtual void targetVel(const sva::MotionVecd & worldVel);

  /**
   * @brief Targets a robot surface with an optional offset.
   * The offset is expressed in the target contact frame
   *
   * @param robotIndex
   *  Index of the robot to target (could be the environment)
   * @param surfaceName
   *  Surface name in the target robot
   * @param offset
   *  offset defined in the target contact frame
   */
  void targetSurface(unsigned int robotIndex,
                     const std::string & surfaceName,
                     const sva::PTransformd & offset = sva::PTransformd::Identity());

  /**
   * @brief Targets a given frame with an optional offset
   *
   * The offset is given in the target frame
   *
   * \param targetFrame Target frame
   *
   * \param offset Offset relative to \p targetFrame
   */
  void targetFrame(const mc_rbdyn::Frame & targetFrame, const sva::PTransformd & offset = sva::PTransformd::Identity());

  /**
   * @brief Targets a given frame velocity with an optional offset
   *
   * The offset is given in the target frame
   *
   * \param targetFrame Target frame
   *
   * \param offset Offset relative to \p targetFrame
   */
  void targetFrameVelocity(const mc_rbdyn::Frame & targetFrame,
                           const sva::PTransformd & offset = sva::PTransformd::Identity());

  /**
   * @brief Targets a given frame with an optional offset
   *
   * The offset is given in the target frame
   *
   * \param targetFrame Target frame
   *
   * \param offset Offset relative to \p targetFrame
   */
  virtual void target(const mc_rbdyn::Frame & frame, const sva::PTransformd & offset);

  /*! \brief Retrieve the controlled frame name */
  inline const std::string & surface() const noexcept { return frame_->name(); }

  /*! \brief Return the controlled frame (const) */
  const mc_rbdyn::RobotFrame & frame() const noexcept { return *frame_; }

  /** Returns the pose of the frame in the inertial frame */
  inline sva::PTransformd surfacePose() const noexcept { return frame_->position(); }

  /** Add support for the following criterias:
   *
   * - wrench: completed when the surface wrench reaches the given wrench, if
   *   some values are NaN, this direction is ignored. Only valid if the surface
   *   controlled by this task is attached to a force sensor, throws otherwise
   *
   * @throws If wrench is used but the surface is not attached to a force sensor
   */
  std::function<bool(const mc_tasks::MetaTask & task, std::string &)> buildCompletionCriteria(
      double dt,
      const mc_rtc::Configuration & config) const override;

  void addToLogger(mc_rtc::Logger & logger) override;

  using TrajectoryTaskGeneric::damping;
  using TrajectoryTaskGeneric::setGains;
  using TrajectoryTaskGeneric::stiffness;

  /*! \brief Set dimensional stiffness and damping
   *
   * \param stiffness Dimensional stiffness
   *
   * \param damping Dimensional damping
   *
   */
  void setGains(const sva::MotionVecd & stiffness, const sva::MotionVecd & damping)
  {
    return TrajectoryTaskGeneric::setGains(stiffness.vector(), damping.vector());
  }

  /*! \brief Set dimensional stiffness
   *
   * Damping is automatically set to 2*sqrt(stiffness)
   *
   * \param stiffness Dimensional stiffness as a motion vector
   *
   */
  void stiffness(const sva::MotionVecd & stiffness) { return TrajectoryTaskGeneric::stiffness(stiffness.vector()); }

  /*! \brief Get dimensional stiffness as a motion vector */
  sva::MotionVecd mvStiffness() { return sva::MotionVecd(dimStiffness()); }

  /*! \brief Set dimensional damping
   *
   * \param damping Dimensional damping as a motion vector
   *
   */
  void damping(const sva::MotionVecd & damping) { return TrajectoryTaskGeneric::damping(damping.vector()); }

  /*! \brief Get dimensional damping as a motion vector */
  sva::MotionVecd mvDamping() { return sva::MotionVecd(dimDamping()); }

  /*! \brief Set trajectory task's reference velocity from motion vector in frame coordinates.
   *
   * \param velB Reference velocity in frame coordinates
   *
   */
  void refVelB(const sva::MotionVecd & velB) { TrajectoryTaskGeneric::refVel(velB.vector()); }

  /*! \brief Get reference velocity in frame coordinates as a motion vector */
  sva::MotionVecd refVelB() const { return sva::MotionVecd(TrajectoryTaskGeneric::refVel()); }

  /*! \brief Set trajectory task's reference acceleration from motion vector.
   *
   * \param acc Reference acceleration in frame coordinates
   *
   */
  void refAccel(const sva::MotionVecd & accel) { return TrajectoryTaskGeneric::refAccel(accel.vector()); }

  /*! \brief Load parameters from a Configuration object */
  void load(mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) override;

protected:
  mc_rbdyn::ConstRobotFramePtr frame_;

  void addToGUI(mc_rtc::gui::StateBuilder & gui) override;

  /* Don't use parent's refVel() as the velocity frame (spatial or body) is
   * ambiguous. */
  using TrajectoryTaskGeneric::refVel;
};

} // namespace mc_tasks
