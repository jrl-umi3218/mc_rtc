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
   */
  SequentialInterpolator() noexcept {}

  /**
   * @brief Creates an interpolator with values
   *
   * @param values Values must respect the conditions describes in \ref setValues "values"
   *
   * @throws std::runtime_error If values are invalid
   */
  SequentialInterpolator(const TimedValueVector & values)
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
    prevIndex_ = 0;
    if(values.size() == 1)
    {
      nextIndex_ = 0;
    }
    else
    {
      nextIndex_ = 1;
    }
    intervalDuration_ = values_[nextIndex_].first - values_.front().first;
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
   * Compute interpolated value at the provided time
   *
   * Calls InterpolationFunction::operator(const Value &, const Value &, double ratio) where ratio is the time in the
   * current interval normalized between 0 and 1
   *
   * Out-of-bound access return the first or last value
   *
   * @throws std::runtime_error If there are no values
   */
  Value compute(double currTime)
  {
    if(values_.empty())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("SequentialInterpolator requires at least one value");
    }

    // Check for out-of-bound access
    if(currTime >= values_.back().first)
    {
      return values_.back().second;
    }
    else if(currTime < values_.front().first)
    {
      return values_.front().second;
    }

    // find the current interval
    nextIndex_ = 0;
    while(++nextIndex_ < values_.size() && values_[nextIndex_].first < currTime)
    {
    }
    prevIndex_ = nextIndex_ - 1;
    intervalDuration_ = values_[nextIndex_].first - values_[prevIndex_].first;

    const auto & prevTime = values_[prevIndex_].first;
    return interpolator_(values_[prevIndex_].second, values_[nextIndex_].second,
                         (currTime - prevTime) / intervalDuration_);
  }

protected:
  /** Checks that values are strictly ordered by ascending time **/
  void checkValues(const TimedValueVector & values)
  {
    if(!std::is_sorted(values.begin(), values.end(),
                       [](const TimedValue & a, const TimedValue & b) { return a.first < b.first; }))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "SequentialInterpolator values must be ordered by strictly ascending time");
    }
  }

protected:
  double dt_ = 0.005; ///< Timestep
  InterpolationFunction interpolator_; ///< Functor for computing the interpolated values
  TimedValueVector values_; ///< Interpolation values

  size_t prevIndex_ = 0; ///< Index of the next element  (first element after currTime_)
  size_t nextIndex_ = 1; ///< Index of the next element  (first element after currTime_)
  double intervalDuration_ = 0; ///< Duration of the current interval in the sequence
};

} // namespace mc_trajectory
