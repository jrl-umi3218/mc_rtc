#pragma once

#include <mc_rtc/GUIState.h>
#include <mc_trajectory/api.h>

#include <hpp/spline/exact_cubic.h>
#include <hpp/spline/spline_deriv_constraint.h>
#include <memory>
#include <vector>

namespace mc_trajectory
{

/*! \brief Represents an Exact Cubic spline :  a curve that passes exactly
 * through waypoints in position, while respecting optional constraints on its
 * initial and final velocity and acceleration.
 */
struct MC_TRAJECTORY_DLLAPI ExactCubic
{
public:
  using point_t = Eigen::Vector3d;
  using waypoint_t = std::pair<double, point_t>;
  using exact_cubic_t = spline::exact_cubic<double, double, 3, false, point_t>;
  using spline_deriv_constraint_t = spline::spline_deriv_constraint<double, double, 3, false, point_t>;
  using spline_constraints_t = spline_deriv_constraint_t::spline_constraints;

public:
  /* \brief Construct a curve passing through the specified optional waypoints
   * with optional initial/final constraints.
   * There should be at least two waypoints (initial and final position).
   *
   * \param duration Duration of the curve
   * \param start Starting position at time t=0
   * \param target Target position at time t=duration
   * \param init_vel Initial velocity
   * \param init_acc Initial acceleration
   * \param end_vel Final velocity
   * \param end_acc Final acceleration
   */
  ExactCubic(double duration,
             const point_t & start,
             const point_t & target,
             const std::vector<waypoint_t> & waypoints = {},
             const point_t & init_vel = {},
             const point_t & init_acc = {},
             const point_t & end_vel = {},
             const point_t & end_acc = {});

  /*! \brief Triggers recreation of the curve. Will only occur if the curve
   * parameters were modified (waypoints, target, constraints), or the sampling size has
   * changed.
   */
  void update();

  /*! \brief Sets the curve waypoints.
   * This will trigger recomputation of the curve at the next update() call.
   *
   * \param waypoints Waypoints as pairs of [time, position]
   */
  void waypoints(const std::vector<waypoint_t> & waypoints);

  /*! \brief Updates the position of an existing waypoint
   *
   * \param idx id of the waypoint
   * \param waypoint desired position
   */
  void waypoint(size_t idx, const point_t & waypoint);

  /*! \brief Updates the time at which an existing waypoint should be reached
   *
   * \param idx id of the waypoint
   * \param t time at which the waypoint should be reached
   */
  void waypoint(size_t idx, const double t);

  /*! \brief Gets position of a waypoint
   *
   * \param idx waypoint id
   *
   * \returns Position of the waypoint idx
   */
  const waypoint_t & waypoint(size_t idx) const;
  /*! \brief Gets the time at which a waypoint should be reached
   *
   * \param idx waypoint id
   *
   * \returns Waypoint time
   */
  double waypointTime(size_t idx) const;
  /*! \brief Gets all waypoints (including start and target)
   *
   * \returns All of the curve waypoints
   */
  const std::vector<waypoint_t> & waypoints() const;

  /*! \brief Defines the initial/final velocity and acceleration constraints for
   * the curve.
   *
   * \param init_vel Initial velocity
   * \param init_acc Initial acceleration
   * \param end_vel Final velocity
   * \param end_acc Final acceleration
   */
  void constraints(const point_t & init_vel,
                   const point_t & init_acc,
                   const point_t & end_vel,
                   const point_t & end_acc);

  /*! \brief Starting point for the bezier curve at time t=0
   *
   * @param pos starting position
   */
  void start(const point_t & start);
  /*! \brief Starting point at time t=0
   *
   * \returns starting point
   */
  const point_t & start() const;

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

  /*! \brief Gets the curve's initial velocity
   *
   * \returns initial velocity
   */
  const point_t & init_vel() const;
  /*! \brief Gets the curve's initial acceleration
   *
   * \returns initial acceleration
   */
  const point_t & init_acc() const;
  /*! \brief Gets the curve's final velocity
   *
   * \returns final velocity
   */
  const point_t & end_vel() const;
  /*! \brief Gets the curve's final acceleration
   *
   * \returns final acceleration
   */
  const point_t & end_acc() const;

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
  double duration_;
  point_t start_;
  point_t target_;
  std::vector<waypoint_t> waypoints_;
  spline_constraints_t constraints_;

  std::unique_ptr<spline_deriv_constraint_t> spline_ = nullptr;

  bool needsUpdate_ = false;
  unsigned samplingPoints_ = 10;
  std::vector<Eigen::Vector3d> samples_;
};

} // namespace mc_trajectory
