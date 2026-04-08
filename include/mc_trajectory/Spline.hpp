/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/logging.h>
#include <mc_trajectory/Spline.h>

#include <stdexcept>

namespace mc_trajectory
{
template<typename T, typename WaypointsT>
Spline<T, WaypointsT>::Spline(double duration, const T & start, const T & target, const WaypointsT & waypoints)
: duration_(duration), start_(start), target_(target), waypoints_(waypoints), needsUpdate_(true)
{
}

template<typename T, typename WaypointsT>
void Spline<T, WaypointsT>::waypoints(const WaypointsT & waypoints)
{
  waypoints_ = waypoints;
  needsUpdate_ = true;
}

template<typename T, typename WaypointsT>
const WaypointsT & Spline<T, WaypointsT>::waypoints() const
{
  return waypoints_;
}

template<typename T, typename WaypointsT>
void Spline<T, WaypointsT>::start(const T & start)
{
  start_ = start;
  needsUpdate_ = true;
}

template<typename T, typename WaypointsT>
const T & Spline<T, WaypointsT>::start() const
{
  return start_;
}

template<typename T, typename WaypointsT>
void Spline<T, WaypointsT>::target(const T & target)
{
  target_ = target;
  needsUpdate_ = true;
}

template<typename T, typename WaypointsT>
const T & Spline<T, WaypointsT>::target() const
{
  return target_;
}

template<typename T, typename WaypointsT>
void Spline<T, WaypointsT>::samplingPoints(unsigned s)
{
  samplingPoints_ = s;
  needsUpdate_ = true;
}

template<typename T, typename WaypointsT>
unsigned Spline<T, WaypointsT>::samplingPoints() const
{
  return samplingPoints_;
}

} // namespace mc_trajectory
