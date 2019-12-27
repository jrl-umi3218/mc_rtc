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

/** Utility functions and classes.
 *
 */
namespace mc_filter
{

namespace utils
{
/** Clamp a value in a given interval.
 *
 * \param v Value.
 *
 * \param vMin Lower bound.
 *
 * \param vMax Upper bound.
 *
 */
inline double clamp(double v, double vMin, double vMax)
{
  if(v > vMax)
  {
    return vMax;
  }
  else if(v < vMin)
  {
    return vMin;
  }
  else
  {
    return v;
  }
}

/** Clamp a value in a given interval.
 *
 * \param v Reference to value.
 *
 * \param vMin Lower bound.
 *
 * \param vMax Upper bound.
 *
 */
inline void clampInPlace(double & v, double vMin, double vMax)
{
  if(v > vMax)
  {
    v = vMax;
  }
  else if(v < vMin)
  {
    v = vMin;
  }
}

/** Clamp a value in a given interval, issuing a warning when bounds are hit.
 *
 * \param v Value.
 *
 * \param vMin Lower bound.
 *
 * \param vMax Upper bound.
 *
 * \param label Name of clamped value.
 *
 */
inline double clamp(double v, double vMin, double vMax, const char * label)
{
  if(v > vMax)
  {
    LOG_WARNING(label << " clamped to " << vMax);
    return vMax;
  }
  else if(v < vMin)
  {
    LOG_WARNING(label << " clamped to " << vMin);
    return vMin;
  }
  else
  {
    return v;
  }
}

/** Clamp a value in a given interval, issuing a warning when bounds are hit.
 *
 * \param v Reference to value.
 *
 * \param vMin Lower bound.
 *
 * \param vMax Upper bound.
 *
 * \param label Name of clamped value.
 *
 */
inline void clampInPlace(double & v, double vMin, double vMax, const char * label)
{
  if(v > vMax)
  {
    LOG_WARNING(label << " clamped to " << vMax);
    v = vMax;
  }
  else if(v < vMin)
  {
    LOG_WARNING(label << " clamped to " << vMin);
    v = vMin;
  }
}

/** Saturate integrator outputs.
 *
 * \param taskName Name of caller AdmittanceTask.
 *
 * \param vector Integrator output vector.
 *
 * \param bound Output (symmetric) bounds.
 *
 * \param label Name of output vector.
 *
 * \param isClamping Map of booleans describing the clamping state for each
 * direction in ['x', 'y', 'z'].
 *
 */
inline void clampAndWarn(const std::string & taskName,
                         Eigen::Vector3d & vector,
                         const Eigen::Vector3d & bound,
                         const std::string & label,
                         std::map<char, bool> & isClamping)
{
  const char dirName[] = {'x', 'y', 'z'};
  for(unsigned i = 0; i < 3; i++)
  {
    char dir = dirName[i];
    if(vector(i) < -bound(i))
    {
      vector(i) = -bound(i);
      if(!isClamping[dir])
      {
        LOG_WARNING(taskName << ": clamping " << dir << " " << label << " to " << -bound(i));
        isClamping[dir] = true;
      }
    }
    else if(vector(i) > bound(i))
    {
      vector(i) = bound(i);
      if(!isClamping[dir])
      {
        LOG_WARNING(taskName << ": clamping " << dir << " " << label << " to " << bound(i));
        isClamping[dir] = true;
      }
    }
    else if(isClamping[dir])
    {
      LOG_WARNING(taskName << ": " << dir << " " << label << " back within range");
      isClamping[dir] = false;
    }
  }
}

} // namespace utils
} // namespace mc_filter
