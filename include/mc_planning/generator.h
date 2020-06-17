#pragma once
// #include <Math/ControlSystem/LIPMControlByPoleAssignWithExternalForce.h>
// #include <Math/ControlSystem/LinearTimeVariantInvertedPendulum.h>
// #include <Model/State.h>
#include <mc_planning/ClampedCubicSpline.h>
#include <mc_planning/LIPMControlByPoleAssignWithExternalForce.h>
#include <mc_planning/LinearTimeVariantInvertedPendulum.h>
#include <mc_planning/State.h>

namespace mc_planning
{

/**
 * @brief Utility class to generate long and short term trajectories for the
 * CoM.
 *
 * Generate trajectories as introduced in:
 * **Online 3D CoM Trajectory Generation for Multi-Contact Locomotion Synchronizing Contact**, *Mitsuharu Morisawa et
 * al., IROS 2018*
 */
struct MC_PLANNING_DLLAPI generator
{
  /**
   * @brief Construct a simple trajectory generator
   *
   * @param n_preview Size of the future preview window. The full preview window
   * goes from future to past with size `2*n_preview+1`
   * @param dt
   */
  generator(int n_preview, double dt);

  /**
   * @brief Generate the long and short term trajectories
   *
   * @param n_time current time
   */
  void generate(int n_time);

  /**
   * @brief Desired steps
   * @see setSteps(const std::vector<Eigen::Vector3d> & steps)
   */
  const std::vector<Eigen::Vector3d> & Steps() const
  {
    return m_steps;
  }

  /**
   * @brief Sets desired reference steps as (time, CoM X, CoM Y)
   *
   * @param steps Desired steps from which the trajectory will be generated
   */
  void setSteps(const std::vector<Eigen::Vector3d> & steps)
  {
    m_steps = steps;
  }

  /**
   * @brief Add a step
   *
   * Step must be after the last existing timestep
   *
   * @param step
   */
  void push_back(const Eigen::Vector3d & step);

  /**
   * @name Accessors
   */
  ///@{
  const Eigen::Vector3d & IdealCOGPosition() const
  {
    return m_COG_ideal.P;
  }
  const Eigen::Vector3d & IdealCOGVelocity() const
  {
    return m_COG_ideal.V;
  }
  const Eigen::Vector3d & IdealCOGAcceleration() const
  {
    return m_COG_ideal.Vdot;
  }

  const Eigen::Vector3d & CompensatedCOGPosition() const
  {
    return m_COG_cmp.P;
  }
  const Eigen::Vector3d & CompensatedCOGVelocity() const
  {
    return m_COG_cmp.V;
  }
  const Eigen::Vector3d & CompensatedCOGAcceleration() const
  {
    return m_COG_cmp.Vdot;
  }

  const Eigen::Vector3d & OutputCOGPosition() const
  {
    return m_COG_out.P;
  }
  const Eigen::Vector3d & OutputCOGVelocity() const
  {
    return m_COG_out.V;
  }
  const Eigen::Vector3d & OutputCOGAcceleration() const
  {
    return m_COG_out.Vdot;
  }

  const Eigen::Vector3d & IdealZMPPosition() const
  {
    return m_Pcalpha_ideal;
  }
  const Eigen::Vector3d & IdealZMPVelocity() const
  {
    return m_Vcalpha_ideal;
  }

  const Eigen::Vector3d & CompensatedZMPPosition() const
  {
    return m_Pcalpha_cmp;
  }
  const Eigen::Vector3d & CompensatedZMPVelocity() const
  {
    return m_Vcalpha_cmp;
  }

  const Eigen::Vector3d & OutputZMPPosition() const
  {
    return m_Pcalpha_out;
  }
  ///@}

private:
  /**
   * @brief Setup CoM with constant height along the trajectory
   * and iterpolate its position/velocity/acceleration
   *
   * Sets m_COG_ideal for the current time
   *
   * @param n_current Index of the start of the preview window
   */
  void setupCOGHeight(int n_current);

  /**
   * @brief Setup reference trajectories based on the desired steps
   *
   * Sets m_ipm_long.px_ref, m_ipm_long.py_ref, m_ipm_long.w2 and m_Pcalpha_ideal
   *
   * @param n_current Index of the start of the preview window
   */
  void setupTimeTrajectories(int n_current);

  /**
   * @brief Solve long and short term trajectories from the reference time
   * trajectories
   *
   * Computes m_COG_out, m_Pcalpha_out, m_Pcalpha_motion_out, m_COG_cmp
   */
  void generateTrajectories();

private:
  std::shared_ptr<motion_interpolator::InterpolatorBase<int>> m_ComInterp;

  StatePVA m_COG_ideal_pre;
  /**
   * Ideal COG trajectory
   * - X, Y components are computed by the long term trajectories m_ipm_long
   * - Z component is interpolated by m_ComInterp
   */
  StatePVA m_COG_ideal;
  StatePVA m_COG_cmp;
  StatePVA m_COG_out;

  Eigen::Vector3d m_Pcalpha_ideal_pre, m_Pcalpha_ideal, m_Pcalpha_cmp;
  Eigen::Vector3d m_Vcalpha_ideal_pre, m_Vcalpha_ideal, m_Vcalpha_cmp;
  Eigen::Vector3d m_Pcalpha_out;
  Eigen::Vector3d m_Pcalpha_motion_out;

  int m_n_preview; ///< Size of the future preview window. The full preview is 2*m_n_preview+1
  int m_n_steps; ///< Current step
  double m_dt; ///< Timestep
  double m_omega_valpha;
  double m_mass; ///< Robot mass

  linear_control_system::LinearTimeVariantInvertedPendulum m_ipm_long[2];
  linear_control_system::LIPMControlByPoleAssignWithExternalForce m_ipm_short[2];
  Eigen::Vector3d m_poles[2];

  Eigen::VectorXd m_virtual_height[2];
  Eigen::VectorXd m_cog_height; ///< CoM height
  Eigen::VectorXd m_cog_dot_height; ///< CoM velocity
  Eigen::VectorXd m_cog_ddot_height; ///< CoM acceleration
  std::vector<Eigen::Vector3d> m_steps; ///< (time, com_x, com_y)
};

} // namespace mc_planning
