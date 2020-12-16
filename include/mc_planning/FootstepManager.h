/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <deque>

#include <hpp/spline/spline_deriv_constraint.h>

#include <mc_planning/api.h>
#include <mc_rtc/log/Logger.h>

namespace mc_planning
{
/*! \brief Enumeration of the left and right feet. */
enum class MC_PLANNING_DLLAPI Foot
{
  Left,
  Right
};

/*! \brief Get the opposite foot. */
Foot opposite(Foot foot)
{
  if(foot == Foot::Left)
  {
    return Foot::Right;
  }
  else
  {
    return Foot::Left;
  }
}

/*! \brief Enumeration of the support phase. */
enum class MC_PLANNING_DLLAPI SupportPhase
{
  DoubleSupport,
  LeftSupport,
  RightSupport
};

/*! \brief Footstep. */
struct MC_PLANNING_DLLAPI Footstep
{
  /*! \brief Constructor.
   *
   *  \param _foot Foot
   *  \param _pose Foot pose
   *  \param _time Time of foot landing
   */
  Footstep(Foot _foot, sva::PTransformd _pose, double _time) : foot(_foot), pose(_pose), landingTime(_time){};

  Foot foot;
  sva::PTransformd pose;
  double landingTime;

  // Time to start ZMP transit to support foot
  double transitStartTime = 0;
  // Time to start swinging the foot
  double swingStartTime = 0;
};

namespace internal
{
/*! \brief See
 * https://github.com/jrl-umi3218/mc_rtc/blob/29b471e7ef317eca3358327c664196ee657a8ab4/include/mc_tasks/lipm_stabilizer/StabilizerTask.h#L664-L671
 */
struct EnumClassHash
{
  template<typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};
} // namespace internal

/*! \brief Contact stance of bipedal robot.
 *
 *  Stance means a set of contacts that are active at the same time.
 */
class MC_PLANNING_DLLAPI Footstance : public std::unordered_map<Foot, Footstep, internal::EnumClassHash>
{
public:
  /*! \brief Constructor. */
  Footstance(){};

  /*! \brief Get the middle pose. */
  sva::PTransformd midPose() const
  {
    if(this->size() == 0)
    {
      return sva::PTransformd::Identity();
    }
    else if(this->size() == 1)
    {
      return this->begin()->second.pose;
    }
    else
    {
      return sva::interpolate(this->at(Foot::Left).pose, this->at(Foot::Right).pose, 0.5);
    }
  }
};

/*! \brief Configuration of FootstepManager. */
struct MC_PLANNING_DLLAPI FootstepManagerConfiguration
{
  // Sum duration of double support phase and single support phase
  // The double support phase includes only the one before swing, not after swing
  double duration = 0.8; // [sec]

  // Duration ratio of double support phase
  double doubleSupportRatio = 0.2; // []

  // ZMP velocity when transitioning to middle pose of both feet after swing
  double midZmpVel = 1.0; // [m/sec]

  // Swing height
  double swingHeight = 0.05; // [m]
};

/*! Footstep manager.
 *
 *  Footstep manager is inputted the target footstep sequentially and outputs the foot pose and the reference ZMP
 * trajectory.
 */
class MC_PLANNING_DLLAPI FootstepManager
{
public:
  /*! \brief Constructor.
   *
   *  \param name Name of the footstep manager
   *  \param dt Sampling time of the preview control
   */
  FootstepManager(const std::string & name = "", double dt = 0.005);

  /*! \brief Reset the footstep manager.
   *
   *  \param currentTime Current time of the controller
   *  \param currentFootstance Current foot stance
   *  \param currentRefZmp Current reference ZMP
   *
   *  \note This method should be called once before you start using FootstepManager.
   */
  void reset(double currentTime, const Footstance & currentFootstance, const Eigen::Vector3d & currentRefZmp);

  /*! \brief Update the footstep manager.
   *
   *  \param currentTime Current time of the controller
   *
   *  \note This method should be called once every control cycle.
   */
  void update(double currentTime);

  /*! \brief Append a target footstep to the queue.
   *
   *  \param _newFootstep Footstep to append
   */
  bool appendFootstep(const Footstep & _newFootstep);

  /*! \brief Make a reference ZMP trajectory.
   *
   *  \param refZmpTraj Matrix to set the trajectory. Number of columns is assumed to be the trajectory length. The
   * trajectory starts with the current reference ZMP.
   */
  void makeZmpTraj(Eigen::Ref<Eigen::Matrix3Xd> refZmpTraj);

  /*! \brief Get the support ratio of left foot. 1 for full left foot support, 0 for full right foot support. */
  double leftFootRatio() const;

  /*! \brief Get dt. */
  double dt() const noexcept
  {
    return dt_;
  }

  /*! \brief Set dt.
   *
   *  \param dt Sampling time of the preview control
   */
  double dt(double _dt)
  {
    return dt_ = _dt;
  }

  /*! \brief Get the foot pose. */
  sva::PTransformd footPose(Foot foot) const
  {
    return currentFootPoses_.at(foot);
  }

  /*! \brief Get the support phase. */
  SupportPhase supportPhase() const noexcept
  {
    return currentSupportPhase_;
  }

  /*! \brief Add entries to the logger. */
  void addToLogger(mc_rtc::Logger & logger);

  /*! \brief Remove entries from the logger. */
  void removeFromLogger(mc_rtc::Logger & logger);

protected:
  std::string name_;

  FootstepManagerConfiguration config_;

  double dt_;

  double currentTime_ = 0;
  Footstance currentFootstance_;
  std::unordered_map<Foot, sva::PTransformd, internal::EnumClassHash> currentFootPoses_;
  SupportPhase currentSupportPhase_;
  Eigen::Vector3d currentRefZmp_;

  std::deque<Footstep> footstepQueue_;

  const Footstep * executingFootstep_ = nullptr;

  std::vector<std::pair<double, Eigen::Vector3d>> swingWayPoints_;
  std::shared_ptr<spline::exact_cubic<double, double, 3, true, Eigen::Vector3d>> swingSpline_;
};
} // namespace mc_planning
