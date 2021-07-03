/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_planning/api.h>
#include <mc_rtc/logging.h>
#include <vector>

namespace mc_planning
{
/**
 * @brief Stores precomputed values of a function for fast lookup
 * @tparam T Data type for the function arguments and result value
 * @tparam CheckBounds Whether to check out-of-bounds access at runtime
 *
 * This class provides a lookup-table implementation intended for fast evaluation of
 * computationally expensive functions. This is done by pre-generating a table
 * of all possible function results in a given range/resolution, and only
 * accessing the stored values instead of computing the function each time.
 *
 * On construction, the provided funciton is evaluated for values of `x` linearely distributed between
 * `min` and `max`. \f[ [f(min), ..., f(x), ..., f(max)] \in T^\mbox{resolution} \f]
 * Function values \f$ f(x) \f$ may be retrieved by calling the operator()(const T & value)
 *
 * @warning Notes regarding accuracy.
 * - Due to the discretization, precision depends heavily on the chosen
 *   resolution and the function evaluated. Higher resolution improves accuracy
 *   at the expanse of memory usage (`resolution*sizeof(T)`). Conversely, lower
 *   resolution decreases accuracy and memory usage.
 * - Accuracy may be lower near the upper bound due to numerical errors in
 *   linear generation of values in the min-max range. Whenever possible, it is
 *   recommended to choose a conservative upper boundary.
 * - Always rigourously evaluate accuracy for each specific use-case.
 * - This implementation makes no assumption on the underlying structure of the
 *   function. Better results and perfomance could be achieved by exploiting
 *   these properties (such as symetry, non-uniform argument distribution, etc)
 */
template<typename T, bool CheckBounds = true>
struct MC_PLANNING_DLLAPI LookupTable
{
  /**
   * @brief Evaluate and store given function results
   * @see create(size_t resolution, const T & min, const T & max, MappingFunction f)
   */
  template<typename MappingFunction>
  LookupTable(size_t resolution, const T & min, const T & max, MappingFunction f, const std::string & name = "")
  {
    create(resolution, min, max, f, name);
  }

  /**
   * @brief Default constructor
   *
   * To initialize the lookup table, call create(size_t resolution, const T & min, const T & max, MappingFunction f)
   */
  LookupTable() noexcept {};

  /**
   * @brief Evaluate and store given function results
   *
   * \f[ [f(min), ..., f(max)] \in T^\mbox{resolution} \f]
   *
   * @param resolution Number of points to compute
   * @param min Minimum argument value
   * @param max Maximum argument value
   * @param f Function to evaluate
   * @param name Human-readable description of the stored function (used for
   * error reporting)
   */
  template<typename MappingFunction>
  void create(size_t resolution, const T & min, const T & max, MappingFunction f, const std::string & name = "")
  {
    name_ = name;
    prefix_ = "[LookupTable]" + name_.size() ? "[" + name_ + "] " : " ";
    if(min > max)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(prefix_ + "Invalid range (min {} > max {})", min, max);
    }
    if(resolution == 0)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(prefix_
                                                       + "Resolution cannot be equal to zero (strictly positive)");
    }
    table_.resize(resolution);
    min_ = min;
    max_ = max;
    T oldrange = static_cast<T>(resolution);
    T newrange = (max - min);
    T fracRange = newrange / oldrange;
    for(size_t i = 0; i < resolution; ++i)
    {
      table_[i] = f((static_cast<T>(i) * fracRange) + min);
    }
    rangeConversion_ = static_cast<T>(table_.size()) / (max_ - min_);
    maxIndex_ = table_.size() - 1;
  }

  /**
   * @brief Retrieves the computed value \f$ f(x) \f$
   *
   * @note Accuracy depends on the chosen resolution.
   *
   * @throws std::runtime_error
   * If CheckBounds=true and x is out of bounds (x < min or x > max).
   *
   * @param x Value to be retrieved. x must be between \f$ [min, max] \f$.
   *
   * @return Closest pre-computed value of \f$ f(x) \f$
   */
  const T & operator()(const T & x) const
  {
    if(CheckBounds)
    {
      if(table_.empty() || x < min_ || x > max_)
      {
        if(table_.empty())
        {
          mc_rtc::log::error_and_throw<std::runtime_error>(
              prefix_ + "Uninitialized table. Please call create() before use.", min_, x, max_);
        }
        else
        {
          mc_rtc::log::error_and_throw<std::runtime_error>(prefix_ + "Out of bound access ({} <= {} <= {})", min_, x,
                                                           max_);
        }
      }
    }
    else
    {
      assert(table_.empty() || x < min_ || x > max_);
    }
    auto h = std::min(static_cast<size_t>(lround((x - min_) * rangeConversion_)), maxIndex_);
    return table_[h];
  }

protected:
  std::vector<T> table_;
  T min_, max_;
  T rangeConversion_;
  size_t maxIndex_ = 0;
  std::string name_ = {};
  std::string prefix_ = {};
};

} // namespace mc_planning
