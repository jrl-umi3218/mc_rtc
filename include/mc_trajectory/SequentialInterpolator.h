/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_rtc/logging.h>
#include <mc_trajectory/LinearInterpolation.h>

namespace mc_trajectory
{

/**
 * @brief Sequentially interpolate timed values
 */
template<typename Value, typename InterpolationFunction = LinearInterpolation<Value>>
struct SequentialInterpolator
{
  using TimedValue = typename std::pair<double, Value>;
  using TimedValueVector = std::vector<TimedValue>;

  SequentialInterpolator(double dt) : dt_(dt) {}

  void values(const std::vector<TimedValue> & values)
  {
    if(values.size() < 2) mc_rtc::log::error_and_throw<std::runtime_error>("Need at least two values");
    double prevTime = values[0].first;
    for(unsigned i = 1; i < values.size(); ++i)
    {
      if(values[i].first <= prevTime)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("Values must be ordered by strictly ascending time");
      }
      prevTime = values[i].first;
    }
    values_ = values;

    // XXX should allow changing values but not resetting time
    prevIndex_ = 0;
    prevTime_ = values_[0].first;
    currTime_ = prevTime_;
    nextIndex_ = 1;
    nextTime_ = values_[1].first;
    currDuration_ = nextTime_ - prevTime_;
  }

  inline const TimedValueVector & values() const noexcept
  {
    return values_;
  }

  Value compute()
  {
    if(prevIndex_ == nextIndex_)
    {
      currTime_ += dt_;
      return values_.back().second;
    }
    const auto & prevValue = values_[prevIndex_].second;
    const auto & nextValue = values_[nextIndex_].second;
    Value result = interpolator_(prevValue, nextValue, (currTime_ - prevTime_) / currDuration_);
    next();
    return result;
  }

  inline double time() const noexcept
  {
    return currTime_;
  }

protected:
  void next()
  {
    currTime_ += dt_;
    if(currTime_ > nextTime_)
    {
      if(nextIndex_ == values_.size() - 1)
      {
        prevIndex_ = nextIndex_;
      }
      else
      {
        prevIndex_++;
        prevTime_ = nextTime_;
        nextIndex_++;
        nextTime_ = values_[nextIndex_].first;
        currDuration_ = nextTime_ - prevTime_;
      }
    }
  }

protected:
  double dt_ = 0.005;
  InterpolationFunction interpolator_;
  std::vector<TimedValue> values_;

  double currTime_ = 0;
  unsigned prevIndex_ = 0;
  double prevTime_ = 0;
  unsigned nextIndex_ = 1;
  double nextTime_ = 0;
  double currDuration_ = 0;
};

} // namespace mc_trajectory
