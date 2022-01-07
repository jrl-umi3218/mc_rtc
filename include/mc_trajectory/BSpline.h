/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_trajectory/Spline.h>
#include <mc_trajectory/api.h>

#include <ndcurves/bezier_curve.h>
#include <vector>

namespace mc_trajectory
{

struct MC_TRAJECTORY_DLLAPI BSpline : public Spline<Eigen::Vector3d, std::vector<Eigen::Vector3d>>
{
  using bezier_curve_t = ndcurves::bezier_curve<double, double, false, Eigen::Vector3d>;
  using waypoints_t = std::vector<Eigen::Vector3d>;

public:
  /*! \brief Creates a curve with given waypoints (should includes starting and
   * target position)
   *
   * \param duration duration of the curve
   * \param start starting position at time t=0
   * \param target target position at time t=duration
   * \param waypoints control points for the bezier curve (excluding start and
   * target points)
   */
  BSpline(double duration,
          const Eigen::Vector3d & start,
          const Eigen::Vector3d & target,
          const waypoints_t & waypoints = {});

  /*! \brief Triggers recreation of the curve. Will only occur if the curve
   * parameters were modified (waypoints, target), or the sampling size has
   * changed.
   */
  void update() override;

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
  std::vector<Eigen::Vector3d> sampleTrajectory();

  /*! \brief Add GUI elements to control the curve's waypoints and targets
   *
   * @param gui GUI to which the task should be added
   * @param category category in the GUI to which the curve's control belong to
   */
  void addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category);

private:
  std::unique_ptr<bezier_curve_t> spline = nullptr;
};

} // namespace mc_trajectory
