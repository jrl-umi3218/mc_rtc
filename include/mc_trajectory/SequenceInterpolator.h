/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_rtc/logging.h>
#include <mc_trajectory/LinearInterpolation.h>

namespace mc_trajectory
{

/**
 * @brief Interpolate values in a timed sequence
 *
 * This class provides the logic to perform interpolation for a sequence of
 * values expressed as pairs of (time, Value):
 * - ensures that values are sorted by strictly ascending time
 * - selects the appropriate interval corresponding to the desired computation time (O(N))
 * - perform interpolation by calling the InterpolationFunction functor on this
 *   interval (default: linear interpolation). The time ratio within the current
 *   interval is computed as a normalized value between 0 and 1. See
 *   LinearInterpolation for example.
 *
 *   \code{.cpp}
 *   InterpolationFunction::operator(const Value &intervalStart, const Value &intervalEnd, double ratio)
 *   \endcode
 *
 * Example:
 * \code{.cpp}
 * auto values = std::vector<double, double>
 * {
 *  {0., 0.}, // time, value
 *  {1., 100.},
 *  {2., 200},
 * }
 * SequenceInterpolator<double> interp{values}; // Creates a linear interpolator
 * interp.compute(0); // Compute interpolation at time t = 0
 * interp.compute(0.005); // compute interpolation at time t = 0.005
 * interp.compute(1.2); // compute interpolation at time t = 1.2
 * ...
 * \endcode
 *
 * \tparam Value Type of values to interpolate. It must meet the requirements of the InterpolationFunction (typically
 * scalar-Value multiplication and Value-Value addition, e.g arithmetic types, Eigen::Vector, etc) \tparam
 * InterpolationFunction Functor for computing the interpolated value.
 */
template<typename Value, typename InterpolationFunction = LinearInterpolation<Value>>
struct SequenceInterpolator
{
  using TimedValue = typename std::pair<double, Value>;
  using TimedValueVector = std::vector<TimedValue>;

  /**
   * @brief Creates an empty interpolator.
   *
   * You must call the \ref setValues "values" setter before calling compute()
   */
  SequenceInterpolator() noexcept {}

  /**
   * @brief Creates an interpolator with values
   *
   * @param values Values must respect the conditions describes in \ref setValues "values"
   *
   * @throws std::runtime_error If values are invalid
   */
  SequenceInterpolator(const TimedValueVector & values)
  {
    this->values(values);
  }

  /**
   * \anchor setValues
   *
   * @brief Set interpolator values.
   *
   * @param values Pairs of (time, value) ordered by strictly ascending time and
   * spread apart by more than dt
   */
  void values(const TimedValueVector & values)
  {
    if(values.empty()) return;
    if(!std::is_sorted(values.begin(), values.end(),
                       [](const TimedValue & a, const TimedValue & b) { return a.first < b.first; }))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "SequenceInterpolator values must be ordered by strictly ascending time");
    }
    values_ = values;
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

  /** Clears all values */
  void clear()
  {
    values_.clear();
  }

  /**
   * Compute interpolated value at the provided time
   *
   * This function first finds the current interval corresponding to the
   * provided time (complexity O(N) at worst), then calls
   * InterpolationFunction::operator(const Value &, const Value &, double ratio)
   * where ratio is the time in the current interval normalized between 0 and 1
   *
   * @param currTime Time at which to compute the interpolation. Out-of-bound access return the first or last value.
   *
   * @throws std::runtime_error If values() is empty
   */
  Value compute(double currTime)
  {
    if(values_.empty())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("SequenceInterpolator requires at least one value");
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
    auto prevIndex = static_cast<size_t>(nextIndex_ - 1);
    double intervalDuration = values_[nextIndex_].first - values_[prevIndex].first;

    const auto & prevTime = values_[prevIndex].first;
    return interpolator_(values_[prevIndex].second, values_[nextIndex_].second,
                         (currTime - prevTime) / intervalDuration);
  }

protected:
  InterpolationFunction interpolator_; ///< Functor for computing the interpolated values
  TimedValueVector values_; ///< Interpolation values
  size_t nextIndex_ = 1; ///< Index of the next element  (first element after current time)
};

} // namespace mc_trajectory
