/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is inspired by Stephane's Caron original implementation as part of
 * lipm_walking_controller <https://github.com/stephane-caron/lipm_walking_controller>
 */

#pragma once

#include <mc_rtc/logging.h>

#include <Eigen/Core>
#include <map>

namespace mc_filter
{

namespace utils
{

/** Clamp a value in a given interval.
 *
 * \param value Value to clamp.
 * \param lower Lower bound.
 * \param upper Upper bound.
 *
 * \returns clamped value
 */
inline double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

/** Clamp a value in-place in a given interval.
 *
 * \see clamp(double value, double lower, double upper)
 */
inline void clampInPlace(double & value, double lower, double upper)
{
  value = clamp(value, lower, upper);
}

/** Clamp a value in a given interval, issuing a warning when bounds are hit.
 *
 * \see clamp(double value, double lower, double upper)
 *
 * \return clamped value
 */
inline double clampAndWarn(double value, double lower, double upper, const std::string & label)
{
  if(value > upper)
  {
    LOG_WARNING(label << " clamped to " << upper);
    return upper;
  }
  else if(value < lower)
  {
    LOG_WARNING(label << " clamped to " << lower);
    return lower;
  }
  else
  {
    return value;
  }
}

/** Clamp value in-place in a given interval, issuing a warning when bounds are hit.
 *
 * \see clampAndWarn(double value, double lower, double upper, const std::string & label)
 */
inline void clampInPlaceAndWarn(double & value, double lower, double upper, const std::string & label)
{
  value = clampAndWarn(value, lower, upper, label);
}

/**
 * @brief Clamps each component of a vector in a given interval. The same lower
 * and upper bounds will be used for each element of the vector.
 *
 * @param v Vector of values
 * @param lower Lower bound
 * @param upper Upper bound
 *
 * VectorT must meet the following requirements:
 * - Read-write element access operator(size_t)
 * - The value type of VectorT must meet the requirements of std::max and std::min
 *
 * \see clamp(const VectorT& v, const VectorT & lower, const VectorT & upper)
 */
template<typename VectorT>
inline VectorT clamp(const VectorT & v, double lower, double upper)
{
  VectorT result(v.size());
  for(unsigned i = 0; i < v.size(); i++)
  {
    result(i) = clamp(v(i), lower, upper);
  }
  return result;
}

/**
 * @brief Clamps each component of a vector in a given interval with
 * vector bounds
 *
 * VectorT requirements: \see VectorT clamp(const VectorT& v, double lower, double upper)
 */
template<typename VectorT>
inline VectorT clamp(const VectorT & v, const VectorT & lower, const VectorT & upper)
{
  VectorT result(v.size());
  for(unsigned i = 0; i < v.size(); i++)
  {
    result(i) = clamp(v(i), lower(i), upper(i));
  }
  return result;
}

/**
 * @brief Clamps each component of a vector in a given interval with
 * vector bounds issuing a warning when bounds are hit
 *
 * \see clamp(const VectorT& v, const VectorT & lower, const VectorT & upper)
 */
template<typename VectorT>
inline VectorT clampAndWarn(const VectorT & v, const VectorT & lower, const VectorT & upper, const std::string & label)
{
  VectorT result(v.size());
  for(unsigned i = 0; i < v.size(); i++)
  {
    result(i) = clampAndWarn(v(i), lower(i), upper(i), label + " (" + std::to_string(i) + ")");
  }
  return result;
}

/**
 * @brief Clamps each component of a vector in a given interval
 *
 * \see clamp(const VectorT& v, const VectorT & lower, const VectorT & upper)
 */
template<typename VectorT>
inline void clampInPlace(VectorT & v, const VectorT & lower, const VectorT & upper)
{
  for(unsigned i = 0; i < v.size(); i++)
  {
    v(i) = clamp(v(i), lower(i), upper(i));
  }
}

/**
 * @brief Clamps in-place each component of a vector in a given interval
 *
 * \see clamp(const VectorT& v, double lower, double upper)
 */
template<typename VectorT>
inline void clampInPlace(VectorT & v, double lower, double upper)
{
  for(unsigned i = 0; i < v.size(); i++)
  {
    v(i) = clamp(v(i), lower, upper);
  }
}

/**
 * @brief Clamps in-place each component of a vector in a given interval,
 * issuing a warning when bounds are hit
 *
 * \see clampInPlace(VectorT& v, const VectorT & lower, const VectorT & upper)
 */
template<typename VectorT>
inline void clampInPlaceAndWarn(VectorT & vector,
                                const VectorT & lower,
                                const VectorT & upper,
                                const std::string & label)
{
  for(unsigned i = 0; i < vector.size(); i++)
  {
    clampInPlaceAndWarn(vector(i), lower(i), upper(i), label + " (" + std::to_string(i) + ")");
  }
}

} // namespace utils
} // namespace mc_filter
