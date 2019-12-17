/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include <mc_trajectory/api.h>

#include <vector>

namespace mc_trajectory
{

template<typename T, typename WaypointsT>
struct Spline
{
public:
  Spline(double duration, const T & start, const T & target, const WaypointsT & waypoints = {});

  /*! \brief Defines waypoints
   *
   * @param waypoints Waypoints.
   * Shouldn't include starting and target orientation (use start() and target() instead).
   * Time should be 0<time<duration.
   */
  void waypoints(const WaypointsT & waypoints);
  /*! \brief Returns waypoints
   *
   * \returns Waypoints.
   * Doesn't include starting and target orienation (use start() and target() instead).
   */
  const WaypointsT & waypoints() const;

  /*! \brief Starting point for the trajectory at time t=0
   *
   * @param pos starting point
   */
  void start(const T & start);
  /*! \brief Starting point at time t=0
   *
   * \returns starting point
   */
  const T & start() const;

  /*! \brief Sets the spline target.
   *
   * @param target final target for the spline
   */
  void target(const T & target);
  /*! \brief Gets the spline target position
   *
   * \returns The spline target position
   */
  const T & target() const;

  /*! \brief Number of sampling points for the trajectory visualization
   *
   * @param s Number of sampling points.
   * If the number of samples is different from the one previously specified,
   * this will trigger a curve recomputation.
   */
  void samplingPoints(unsigned s);

  /*! \brief Gets number of samples
   *
   * \returns number of samples
   */
  unsigned samplingPoints() const;

  /*! \brief Triggers recreation of the curve. Will only occur if the curve
   * parameters were modified (waypoints, target), or the sampling size has
   * changed.
   */
  virtual void update() = 0;

protected:
  double duration_;
  T start_;
  T target_;
  WaypointsT waypoints_;
  unsigned samplingPoints_ = 10;
  std::vector<T> samples_;
  bool needsUpdate_ = false;
};

} // namespace mc_trajectory

#include <mc_trajectory/Spline.hpp>
