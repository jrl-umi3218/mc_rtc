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
 * This class provides an efficient and generic way to perform interpolation within a sequence of (sorted) intervals
 * expressed as pairs of (time, Value):
 * - ensures that values are sorted by strictly ascending time
 * - selects the appropriate interval corresponding to the desired computation time
 *   - O(log n) when interpolating at an arbitrary time (binary search)
 *   - O(1) in most cases when used sequentially (the current interval is cached)
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
    prevIndex_ = 0;
    if(values_.size() > 1)
    {
      nextIndex_ = 1;
      intervalDuration_ = values_[nextIndex_].first - values_[prevIndex_].first;
    }
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
   * Calls the interpolation functor with time normalized between 0 and 1
   *
   * \code{.cpp}
   * InterpolationFunction::operator(const Value &, const Value &, double ratio)
   * \endcode{.cpp}
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
    else if(currTime <= values_.front().first)
    {
      return values_.front().second;
    }

    /*
     * Efficiently update interval index and associated cached values:
     * 1/ Check whether currTime is in the current interval
     * 2/ Check whether currTime is in the next interval
     * 3/ Perform a binary search
     * Note that we don't need to check indices bounds here as they are valid by
     * construction.
     */
    auto updateIndex = [this, currTime]() {
      if(values_[prevIndex_].first < currTime && values_[nextIndex_].first >= currTime)
      {
        return;
      }
      else if(values_[nextIndex_].first < currTime && values_[nextIndex_ + 1].first >= currTime)
      {
        ++prevIndex_;
        ++nextIndex_;
        intervalDuration_ = values_[nextIndex_].first - values_[prevIndex_].first;
      }
      else
      {
        // first element in values_ with time greater or equal to currTime
        auto nextIt = std::lower_bound(std::begin(values_), std::end(values_), currTime,
                                       [](const TimedValue & lhs, const double & rhs) { return lhs.first < rhs; });
        nextIndex_ = static_cast<size_t>(nextIt - values_.begin());
        prevIndex_ = nextIndex_ - 1;
        intervalDuration_ = values_[nextIndex_].first - values_[prevIndex_].first;
      }
    };

    updateIndex();
    const auto & prevTime = values_[prevIndex_].first;
    return interpolator_(values_[prevIndex_].second, values_[nextIndex_].second,
                         (currTime - prevTime) / intervalDuration_);
  }

protected:
  InterpolationFunction interpolator_; ///< Functor for computing the interpolated values
  TimedValueVector values_; ///< Interpolation values
  size_t prevIndex_ = 0; ///< Cache the previous index to optimize lookup when used sequentially
  size_t nextIndex_ = 0; ///< Cache the next index
  double intervalDuration_ = 0; ///< Cache the duration of the current interval
};

} // namespace mc_trajectory
