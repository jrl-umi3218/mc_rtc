#pragma once
#include <mc_planning/ClampedCubicSpline.h>
#include <mc_planning/LIPMControlByPoleAssignWithExternalForce.h>
#include <mc_planning/LinearTimeVariantInvertedPendulum.h>
#include <mc_planning/PreviewSteps.h>
#include <mc_planning/State.h>
#include <mc_rtc/log/Logger.h>
#include <list>

namespace mc_planning
{

/**
 * @brief Utility class to generate long and short term trajectories for the CoM.
 *
 * Generate trajectories as introduced in:
 * **Online 3D CoM Trajectory Generation for Multi-Contact Locomotion Synchronizing Contact**, *Mitsuharu Morisawa et
 * al., IROS 2018*
 *
 * @see mc_planning::linear_control_system::LinearTimeVariantInvertedPendulum long-term trajectory generation
 * @see mc_planning::linear_control_system::LIPMControlByPoleAssignWithExternalForce short-term trajectory generation to
 * continuously track the long-term trajectory
 */
struct MC_PLANNING_DLLAPI generator
{
  using PreviewStepsSet = PreviewSteps<Eigen::Vector2d>::PreviewStepsSet;

  /**
   * @brief Construct a simple trajectory generator
   *
   * @param n_preview Size of the future preview window. The full preview window
   * goes from future to past with size `2*n_preview+1`
   * @param dt
   * @mass Robot mass
   * @waist_height Initial height of the waist (constant for now)
   */
  generator(unsigned n_preview, double dt, double mass = 60, double waist_height = 0.8);

  /**
   * @brief Generate the long and short term trajectories
   *
   * @param n_time current time
   */
  void generate(unsigned n_time);

  /**
   * @brief Add to Logger
   */
  void addToLogger(mc_rtc::Logger & logger);
  /**
   * @brief Remove from Logger
   */
  void removeFromLogger(mc_rtc::Logger & logger);

  /**
   * @brief Desired steps
   * @see setSteps(const std::vector<Eigen::Vector3d> & steps)
   */
  const PreviewSteps<Eigen::Vector2d> & steps() const
  {
    return m_steps;
  }

  /**
   * @brief Sets desired reference steps as (time, CoM X, CoM Y)
   *
   * @param steps Desired steps from which the trajectory will be generated
   */
  void steps(const PreviewSteps<Eigen::Vector2d> & steps)
  {
    m_steps = steps;
  }

  /**
   * @brief Change future steps and ensures continuous trajectory
   *
   * @note The previous footstep will be extended until `transitionTime`, and
   * all future steps currently in the trajectory will be removed
   *
   * @param transitionTime Time at which the transition should occur
   * @param futureSteps Future steps
   */
  void changeFutureSteps(double transitionTime, const std::vector<TimedStep<Eigen::Vector2d>> & futureSteps)
  {
    m_steps.changeFutureSteps(transitionTime, futureSteps);
  }

  /**
   * @name Accessors
   */
  ///@{

  /**
   * @brief Sets the poles for the short-term trajectory
   *
   * Typical values range from \f$ [1,1,150] \f$ to \f$ [1,1,300] \f$.
   * The bigger the poles are, the more the faster the short-term trajectory
   * will converge to the long-term trajectory. Too high a value could lead to
   * jerk when the long-term trajectory is not continuous
   *
   * @param poles Poles for the short-term trajectory
   */
  void polesX(const Eigen::Vector3d & poles)
  {
    m_poles[0] = poles;
  }

  /** @brief Gets the short-term trajectory poles */
  const Eigen::Vector3d & polesX() const
  {
    return m_poles[0];
  }

  /**
   * @brief Sets the poles for the short-term trajectory
   *
   * @see polesX(const Eigen::Vector3d & poles)
   */
  void polesY(const Eigen::Vector3d & poles)
  {
    m_poles[0] = poles;
  }

  /** @brief Gets the short-term trajectory poles */
  const Eigen::Vector3d & polesY() const
  {
    return m_poles[0];
  }
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
  void setupCOGHeight(unsigned n_current);

  /**
   * @brief Setup reference trajectories based on the desired steps
   *
   * Sets m_ipm_long.px_ref, m_ipm_long.py_ref, m_ipm_long.w2 and m_Pcalpha_ideal
   *
   * @param n_current Index of the start of the preview window
   */
  void setupTimeTrajectories(unsigned n_current);

  /**
   * @brief Solve long and short term trajectories from the reference time
   * trajectories
   *
   * Computes m_COG_out, m_Pcalpha_out, m_Pcalpha_motion_out, m_COG_cmp
   */
  void generateTrajectories();

private:
  std::shared_ptr<motion_interpolator::InterpolatorBase<unsigned>> m_ComInterp = nullptr;

  StatePVA m_COG_ideal_pre;
  /**
   * Ideal COG trajectory
   * - X, Y components are computed by the long term trajectories m_ipm_long
   * - Z component is interpolated by m_ComInterp
   */
  StatePVA m_COG_ideal;
  StatePVA m_COG_cmp;
  StatePVA m_COG_out;

  /**
   * @name ZMP-related
   * @{
   */
  Eigen::Vector3d m_Pcalpha_ideal_pre = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_Pcalpha_ideal = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_Pcalpha_cmp = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_Vcalpha_ideal_pre = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_Vcalpha_ideal = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_Vcalpha_cmp = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_Pcalpha_out = Eigen::Vector3d::Zero();
  Eigen::Vector3d m_Pcalpha_motion_out = Eigen::Vector3d::Zero();
  /// @}

  unsigned m_n_preview = 0; ///< Size of the future preview window. The full preview is 2*m_n_preview+1
  double m_dt = 0.005; ///< Timestep
  double m_omega_valpha = 0.0;
  double m_mass = 60.; ///< Robot mass
  double m_waist_height; ///< Height of the weight (constant for now)

  std::array<linear_control_system::LinearTimeVariantInvertedPendulum, 2> m_ipm_long;
  std::array<linear_control_system::LIPMControlByPoleAssignWithExternalForce, 2> m_ipm_short;
  /**< Poles for the short-term trajectory.
   * Typical range [1,1,150] ... [1,1,300]
   */
  std::array<Eigen::Vector3d, 2> m_poles = {Eigen::Vector3d{1., 1., 150.}, Eigen::Vector3d{1., 1., 150.}};

  std::array<Eigen::VectorXd, 2> m_virtual_height;
  Eigen::VectorXd m_cog_height; ///< CoM height
  Eigen::VectorXd m_cog_dot_height; ///< CoM velocity
  Eigen::VectorXd m_cog_ddot_height; ///< CoM acceleration

  // unsigned m_n_steps = 0; ///< Current step
  // std::vector<Eigen::Vector3d> m_steps; ///< (time, com_x, com_y)
  PreviewSteps<Eigen::Vector2d> m_steps;
};

} // namespace mc_planning
