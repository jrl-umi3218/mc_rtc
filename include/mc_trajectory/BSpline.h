/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifndef _H_BSPLINETRAJECTORY_H_
#define _H_BSPLINETRAJECTORY_H_

#include <mc_rtc/GUIState.h>
#include <mc_trajectory/api.h>

#include <hpp/spline/bezier_curve.h>
#include <vector>

namespace mc_trajectory
{

struct MC_TRAJECTORY_DLLAPI BSpline
{
  using point_t = Eigen::Vector3d;
  using bezier_curve_t = spline::bezier_curve<double, double, 3, false, point_t>;
  using t_point_t = bezier_curve_t::t_point_t;

public:
  /*! \brief Creates a curve with given waypoints (should includes starting and
   * target position)
   *
   * \param duration duration of the curve
   * \param order curve order
   */
  BSpline(const t_point_t & waypoints, double duration, unsigned int order = 4);
  /*! \brief Creates an empty curve. Call waypoints() to construct a valid curve
   *
   * \param duration duration of the curve
   * \param order curve order
   */
  BSpline(double duration, unsigned int order = 4);

  /*! \brief Triggers recreation of the curve. Will only occur if the curve
   * parameters were modified (waypoints, target), or the sampling size has
   * changed.
   */
  void update();

  /*! \brief Computes the position along the curve at time t, and its
   * derivatives up to the nth order (typically velocity, acceleration)
   *
   * \param t time at which the curve should be evaluated
   * \param der derivation order
   *
   * \returns The position and its derivatives along the curve at time t.
   */
  std::vector<Eigen::Vector3d> splev(double t, unsigned int der = 0);

  /*! \brief Sample positions along the trajectory (for visualization purposes)
   *
   * \param samples number of samples along the curve to compute [minimum = 1]
   *
   * \returns Evenly sampled positions along the trajectory.
   */
  std::vector<Eigen::Vector3d> sampleTrajectory(unsigned samples);

  /*! \brief Sets the curve waypoints.
   * This will trigger recomputation of the curve at the next update() call.
   *
   * \param waypoints Position of all waypoints
   */
  void waypoints(const t_point_t & waypoints);
  /*! \brief Gets all waypoints (including start and target)
   *
   * \returns All of the curve waypoints
   */
  const t_point_t & waypoints() const;

  /*! \brief Sets the curve target.
   * Internally, the target is defined as a waypoint at t=curve duration
   *
   * @param target final target for the curve
   */
  void target(const point_t & target);

  /*! \brief Gets the curve target position
   *
   * \returns The curve target position
   */
  const point_t & target() const;

  /*! \brief Number of sampling points for the trajectory visualization
   *
   * @param s Number of sampling points.
   * If the number of samples is different from the one previously specified,
   * this will trigger a curve recomputation.
   */
  void samplingPoints(const unsigned s);

  /*! \brief Gets number of samples
   *
   * \returns number of samples
   */
  unsigned samplingPoints() const;

  /*! \brief Add GUI elements to control the curve's waypoints and targets
   *
   * @param gui GUI to which the task should be added
   * @param category category in the GUI to which the curve's control belong to
   */
  void addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category);

private:
  double duration;
  unsigned int p;
  std::unique_ptr<bezier_curve_t> spline = nullptr;
  bool needsUpdate_ = false;
  unsigned samplingPoints_ = 10;
  std::vector<point_t> samples_;
  t_point_t waypoints_;
};

} // namespace mc_trajectory

#endif
