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
 *
 * This class provides the logic to perform interpolation for a sequence of
 * values expressed as pairs of (time, Value). It is intended to be used in a
 * real-time context where the compute() function is called once every
 * iteration. Every call to compute() increments time by dt_ and calls the
 * interpolation functor InterpolationFunction. By default, this class computes
 * linear interpolation.
 *
 * Example:
 * \code{.cpp}
 * auto values = std::vector<double, double>
 * {
 * {0., 0.},
 * {1., 100.},
 * {2., 200},
 * }
 * SequentialInterpolator<double> interp{values};
 * interp.compute(); // Compute interpolation at time t = 0
 * interp.compute(); // compute interpolation at time t = 0.005
 * ...
 * \endcode
 *
 * \tparam Value Type of values to interpolate. Must support multiplication by scalar and addition with another Value
 * (arithmetic types, Eigen::Vector, etc)
 * \tparam InterpolationFunction should be a functor taking two values, and a ratio between 0 and 1. This class will
 * call it with the current interval values and the normalized time within this interval.
 */
template<typename Value, typename InterpolationFunction = LinearInterpolation<Value>>
struct SequentialInterpolator
{
  using TimedValue = typename std::pair<double, Value>;
  using TimedValueVector = std::vector<TimedValue>;

  /**
   * @brief Creates an empty interpolator
   *
   * You must call the \ref setValues "values" setter before calling compute()
   *
   * @param dt Controller's timestep
   */
  SequentialInterpolator(double dt) noexcept : dt_(dt) {}

  /**
   * @brief Creates an interpolator with values
   *
   * @param dt Controller's timestep
   * @param values Values must respect the conditions describes in \ref setValues "values"
   *
   * @throws std::runtime_error If values are invalid
   */
  SequentialInterpolator(double dt, const TimedValueVector & values) : dt_(dt)
  {
    this->values(values);
  }

  /**
   * @brief Set interpolator values. The current time will be set to the time of the
   * first entry
   *
   * @param values Pairs of (time, value) ordered by strictly ascending time and
   * spread apart by more than dt
   *
   * \anchor setValues
   */
  void values(const TimedValueVector & values)
  {
    if(values.empty()) return;
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
    intervalDuration_ = values_[nextIndex_].first - values_[prevIndex_].first;
  }

  /** Interpolation values */
  inline const TimedValueVector & values() const noexcept
  {
    return values_;
  }

  /**
   * When true, this interpolator is in a valid state and \ref compute will
   * always return a valid value.
   */
  inline bool hasValues() const noexcept
  {
    return values_.size();
  }

  /**
   * Clears all values
   */
  void clear()
  {
    values_.clear();
  }

  /**
   * Compute interpolated value and move to the next element (increment time)
   *
   * Calls InterpolationFunction::operator(const Value &, const Value &, double ratio) where ratio is the time in the
   * current interval normalized between 0 and 1
   *
   * If time is greater than the last element, return it
   *
   * @throws std::runtime_error If there are no values
   */
  Value compute()
  {
    if(values_.empty())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("SequentialInterpolator requires at least one value");
    }

    if(prevIndex_ == nextIndex_)
    { // past last element
      currTime_ += dt_;
      return values_.back().second;
    }
    const auto & prevValue = values_[prevIndex_].second;
    const auto & nextValue = values_[nextIndex_].second;
    const auto & prevTime = values_[prevIndex_].first;
    Value result = interpolator_(prevValue, nextValue, (currTime_ - prevTime) / intervalDuration_);
    next();
    return result;
  }

  inline double time() const noexcept
  {
    return currTime_;
  }

protected:
  /** Increment time and update indices */
  void next()
  {
    currTime_ += dt_;
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
        intervalDuration_ = values_[nextIndex_].first - values_[prevIndex_].first;
      }
    }
  }

  /** Checks that values are strictly ordered by ascending time and spread by
   * more than dt_ apart.
   **/
  void checkValues(const TimedValueVector & values)
  {
    double prevTime = values.front().first;
    for(unsigned i = 1; i < values.size(); ++i)
    {
      auto nextTime = values[i].first;
      if(nextTime <= prevTime || nextTime - prevTime < dt_)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>("SequentialInterpolator values must be ordered by strictly "
                                                         "ascending time and spread by more than dt apart");
      }
      prevTime = nextTime;
    }
  }

protected:
  double dt_ = 0.005; ///< Timestep
  InterpolationFunction interpolator_; ///< Functor for computing the interpolated values
  std::vector<TimedValue> values_; ///< Interpolation values

  double currTime_ = 0; ///< Current time (starts at the first value)
  size_t prevIndex_ = 0; ///< Index of the previous element (first element before currTime_)
  size_t nextIndex_ = 1; ///< Index of the next element  (first element after currTime_)
  double intervalDuration_ = 0; ///< Duration of the current interval in the sequence
};

} // namespace mc_trajectory
