#include <mc_planning/FootstepManager.h>

namespace mc_planning
{
FootstepManager::FootstepManager(const std::string & name, double dt) : name_(name), dt_(dt)
{
  name_ = "footstep_manager";
  if(!name.empty())
  {
    name_ += "_" + name;
  }
}

void FootstepManager::reset(double currentTime,
                            const Footstance & currentFootstance,
                            const Eigen::Vector3d & currentRefZmp)
{
  currentTime_ = currentTime;

  currentFootstance_.clear();
  for(const auto & footstep : currentFootstance)
  {
    currentFootstance_.insert({footstep.first, footstep.second});
  }

  currentFootPoses_.clear();
  currentFootPoses_.insert({Foot::Left, sva::PTransformd::Identity()});
  currentFootPoses_.insert({Foot::Right, sva::PTransformd::Identity()});

  currentSupportPhase_ = SupportPhase::DoubleSupport;

  currentRefZmp_ = currentRefZmp;

  footstepQueue_.clear();

  executingFootstep_ = nullptr;
}

void FootstepManager::update(double currentTime)
{
  currentTime_ = currentTime;

  // Remove old footsteps from footstepQueue_ and update currentFootstance_
  while(!footstepQueue_.empty() && footstepQueue_.front().landingTime < currentTime_)
  {
    const auto & footstep = footstepQueue_.front();
    currentFootstance_.at(footstep.foot) = footstep;
    footstepQueue_.pop_front();
  }

  if(!footstepQueue_.empty() && currentTime_ >= footstepQueue_.front().swingStartTime)
  {
    // Single support
    const Footstep & nextFootstep = footstepQueue_.front();

    if(executingFootstep_ != &nextFootstep)
    {
      // Switched to new single support phase
      executingFootstep_ = &nextFootstep;

      swingWayPoints_ = {
          {nextFootstep.swingStartTime, currentFootstance_.at(nextFootstep.foot).pose.translation()},
          {0.5 * (nextFootstep.swingStartTime + nextFootstep.landingTime),
           0.5 * (currentFootstance_.at(nextFootstep.foot).pose.translation() + nextFootstep.pose.translation())
               + Eigen::Vector3d(0, 0, config_.swingHeight)},
          {nextFootstep.landingTime, nextFootstep.pose.translation()}};
      swingSpline_ = std::make_shared<spline::exact_cubic<double, double, 3, true, Eigen::Vector3d>>(
          swingWayPoints_.begin(), swingWayPoints_.end());
    }

    // Update currentFootPoses_ and currentSupportPhase_
    // \todo: interpolate the swing foot orientation
    currentFootPoses_[nextFootstep.foot] = sva::PTransformd((*swingSpline_)(currentTime_));
    currentFootPoses_.at(opposite(nextFootstep.foot)) = currentFootstance_.at(opposite(nextFootstep.foot)).pose;
    if(nextFootstep.foot == Foot::Left)
    {
      currentSupportPhase_ = SupportPhase::RightSupport;
    }
    else
    {
      currentSupportPhase_ = SupportPhase::LeftSupport;
    }
  }
  else
  {
    // Double support
    if(executingFootstep_)
    {
      executingFootstep_ = nullptr;
    }

    // Update currentFootPoses_ and currentSupportPhase_
    currentFootPoses_.at(Foot::Left) = currentFootstance_.at(Foot::Left).pose;
    currentFootPoses_.at(Foot::Right) = currentFootstance_.at(Foot::Right).pose;
    currentSupportPhase_ = SupportPhase::DoubleSupport;
  }
}

bool FootstepManager::appendFootstep(const Footstep & _newFootstep)
{
  // Check time of new footstep
  if(_newFootstep.landingTime <= currentTime_)
  {
    mc_rtc::log::warning("Ignore a new footstep with past time: {} <= {}", _newFootstep.landingTime, currentTime_);
    return false;
  }
  if(!footstepQueue_.empty())
  {
    const Footstep & lastFootstep = footstepQueue_.back();
    if(_newFootstep.landingTime <= lastFootstep.landingTime)
    {
      mc_rtc::log::warning("Ignore a new footstep earlier than the last footstep: {} <= {}", _newFootstep.landingTime,
                           lastFootstep.landingTime);
      return false;
    }
  }

  // Push to the queue
  footstepQueue_.push_back(_newFootstep);

  // Set the time of new footstep
  Footstep & newFootstep = footstepQueue_.back();
  double duration = config_.duration;
  if(footstepQueue_.size() >= 2)
  {
    const auto & lastSecondFootstep = footstepQueue_.end()[-2];
    duration = std::min(duration, newFootstep.landingTime - lastSecondFootstep.landingTime);
  }
  else
  {
    duration = std::min(duration, newFootstep.landingTime - currentTime_);
  }
  newFootstep.transitStartTime = newFootstep.landingTime - duration;
  newFootstep.swingStartTime = newFootstep.landingTime - (1.0 - config_.doubleSupportRatio) * duration;

  return true;
}

void FootstepManager::makeZmpTraj(Eigen::Ref<Eigen::Matrix3Xd> refZmpTraj)
{
  int trajSize = static_cast<int>(refZmpTraj.cols());
  int colIdx = 0;

  Footstance tmpFootstance = currentFootstance_;
  auto tmpTime = [&]() { return this->currentTime_ + colIdx * this->dt_; };
  auto cntUntil = [&](double goalTime) {
    return std::min(trajSize - colIdx, std::max(0, static_cast<int>((goalTime - tmpTime()) / this->dt_)));
  };
  auto terminalCond = [&]() { return colIdx >= trajSize; };

  // Set the current reference position to the head
  refZmpTraj.col(colIdx) = currentRefZmp_;
  colIdx++;

  for(const auto & focusedFootstep : footstepQueue_)
  {
    const Footstep & supportFootstep = tmpFootstance.at(opposite(focusedFootstep.foot));

    // Move ZMP from the current position to the foot middle position
    {
      int midCnt = cntUntil(focusedFootstep.transitStartTime);
      Eigen::Vector3d midZmp = tmpFootstance.midPose().translation();
      for(int i = 0; i < midCnt; i++)
      {
        const Eigen::Vector3d & lastZmp = refZmpTraj.col(colIdx - 1);
        const Eigen::Vector3d deltaZmp = midZmp - lastZmp;
        if(deltaZmp.norm() < 1e-6)
        {
          refZmpTraj.col(colIdx) = lastZmp;
        }
        else
        {
          refZmpTraj.col(colIdx) = lastZmp + std::min(deltaZmp.norm(), config_.midZmpVel * dt_) * deltaZmp.normalized();
        }
        colIdx++;
      }
      if(terminalCond())
      {
        break;
      }
    }

    // Move ZMP from the current position to the support foot position
    {
      int transitCnt = cntUntil(focusedFootstep.swingStartTime);
      const Eigen::Vector3d & lastZmp = refZmpTraj.col(colIdx - 1);
      for(int i = 0; i < transitCnt; i++)
      {
        double ratio = static_cast<double>(i + 1) / transitCnt;
        refZmpTraj.col(colIdx) = (1 - ratio) * lastZmp + ratio * supportFootstep.pose.translation();
        colIdx++;
      }
      if(terminalCond())
      {
        break;
      }
    }

    // Keep ZMP in the support foot position
    {
      int swingCnt = cntUntil(focusedFootstep.landingTime);
      for(int i = 0; i < swingCnt; i++)
      {
        refZmpTraj.col(colIdx) = supportFootstep.pose.translation();
        colIdx++;
      }
      if(terminalCond())
      {
        break;
      }
    }

    // Update tmpFootstance
    tmpFootstance.at(focusedFootstep.foot) = focusedFootstep;
  }

  // Move ZMP from the current position to the foot middle position
  if(!terminalCond())
  {
    int finCnt = trajSize - colIdx;
    Eigen::Vector3d midZmp = tmpFootstance.midPose().translation();
    for(int i = 0; i < finCnt; i++)
    {
      const Eigen::Vector3d & lastZmp = refZmpTraj.col(colIdx - 1);
      const Eigen::Vector3d deltaZmp = midZmp - lastZmp;
      if(deltaZmp.norm() < 1e-6)
      {
        refZmpTraj.col(colIdx) = lastZmp;
      }
      else
      {
        refZmpTraj.col(colIdx) = lastZmp + std::min(deltaZmp.norm(), config_.midZmpVel * dt_) * deltaZmp.normalized();
      }
      colIdx++;
    }
  }

  // Update currentRefZmp_
  currentRefZmp_ = refZmpTraj.col(1);
}

double FootstepManager::leftFootRatio() const
{
  if(currentSupportPhase_ == SupportPhase::LeftSupport)
  {
    return 1;
  }
  else if(currentSupportPhase_ == SupportPhase::RightSupport)
  {
    return 0;
  }
  else
  {
    const Eigen::Vector3d & leftFootPos = currentFootstance_.at(Foot::Left).pose.translation();
    const Eigen::Vector3d & rightFootPos = currentFootstance_.at(Foot::Right).pose.translation();
    return (currentRefZmp_ - rightFootPos).norm() / (leftFootPos - rightFootPos).norm();
  }
}

void FootstepManager::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry(name_ + "_left_foot", [this]() { return currentFootPoses_.at(Foot::Left); });
  logger.addLogEntry(name_ + "_right_foot", [this]() { return currentFootPoses_.at(Foot::Right); });
}

void FootstepManager::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry(name_ + "_left_foot");
  logger.removeLogEntry(name_ + "_right_foot");
}

} // namespace mc_planning
