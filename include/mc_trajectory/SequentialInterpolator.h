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

  SequentialInterpolator(const TimedValueVector & values)
  {
    this->values(values);
    mc_rtc::log::info("Created interpolator");
  }

  /**
   * @brief Set interpolator values. The current time will be set to the time of the
   * first entry
   *
   * @param values Pairs of time, value. There must be at least one value and
   * they must be sorted in strictly ascending time.
   */
  void values(const TimedValueVector & values)
  {
    checkValues(values);
    values_ = values;
    currTime_ = values.front().first;
    prevIndex_ = 0;
    if(values.size() == 1)
    {
      nextIndex_ = 0;
    }
    else
    {
      nextIndex_ = 1;
    }
    currDuration_ = values_[nextIndex_].first - values_[prevIndex_].first;
  }

  inline const TimedValueVector & values() const noexcept
  {
    return values_;
  }

  /**
   * Compute interpolated value and increment time
   *
   * If time is greater than the last element, return it
   */
  Value compute(double dt)
  {
    mc_rtc::log::info("Compute {}, {}, {} ", prevIndex_, nextIndex_, currTime_);
    if(prevIndex_ == nextIndex_)
    {
      currTime_ += dt_;
      return values_.back().second;
    }
    const auto & prevValue = values_[prevIndex_].second;
    const auto & nextValue = values_[nextIndex_].second;
    const auto & prevTime = values_[prevIndex_].first;
    Value result = interpolator_(prevValue, nextValue, (currTime_ - prevTime) / currDuration_);
    next(dt);
    return result;
  }

  inline double time() const noexcept
  {
    return currTime_;
  }

protected:
  /** Increment time and update indices */
  void next(double dt)
  {
    currTime_ += dt;
    const auto & nextTime = values_[nextIndex_].first;
    if(currTime_ > nextTime)
    {
      if(nextIndex_ == values_.size() - 1)
      {
        prevIndex_ = nextIndex_;
      }
      else
      {
        prevIndex_++;
        nextIndex_++;
        currDuration_ = values_[nextIndex_].first - values_[prevIndex_].first;
      }
    }
  }

  void checkValues(const TimedValueVector & values)
  {
    if(values.empty())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Need at least one value");
    }
    double prevTime = values[0].first;
    for(unsigned i = 1; i < values.size(); ++i)
    {
      if(values[i].first <= prevTime)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("Values must be ordered by strictly ascending time");
      }
      prevTime = values[i].first;
    }
  }

protected:
  double dt_ = 0.005;
  InterpolationFunction interpolator_;
  std::vector<TimedValue> values_; ///< Interpolation values, has at least one element

  double currTime_ = 0;
  size_t prevIndex_ = 0;
  size_t nextIndex_ = 1;
  double currDuration_ = 0;
};

} // namespace mc_trajectory
