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
class generator
{
private:
  std::shared_ptr<motion_interpolator::InterpolatorBase<int>> m_ComInterp;

  StatePVA m_COG_ideal_pre, m_COG_ideal, m_COG_cmp;
  StatePVA m_COG_out;

  Eigen::Vector3d m_Pcalpha_ideal_pre, m_Pcalpha_ideal, m_Pcalpha_cmp;
  Eigen::Vector3d m_Vcalpha_ideal_pre, m_Vcalpha_ideal, m_Vcalpha_cmp;
  Eigen::Vector3d m_Pcalpha_out;
  Eigen::Vector3d m_Pcalpha_motion_out;

  int m_n_preview;
  int m_n_steps;
  double m_dt;
  double m_omega_valpha;
  double m_mass;

  linear_control_system::LinearTimeVariantInvertedPendulum m_ipm_long[2];
  linear_control_system::LIPMControlByPoleAssignWithExternalForce m_ipm_short[2];
  Eigen::Vector3d m_poles[2];

  Eigen::VectorXd m_virtual_height[2];
  Eigen::VectorXd m_cog_height;
  Eigen::VectorXd m_cog_dot_height;
  Eigen::VectorXd m_cog_ddot_height;
  std::vector<Eigen::Vector3d> m_steps; ///< (time, com_x, com_y)

  void setupCOGHeight(int n_current);

  void setupTimeTrajectories(int n_current);

  void generateTrajectories(void);

public:
  const Eigen::Vector3d & IdealCOGPosition(void) const
  {
    return m_COG_ideal.P;
  }
  const Eigen::Vector3d & IdealCOGVelocity(void) const
  {
    return m_COG_ideal.V;
  }
  const Eigen::Vector3d & IdealCOGAcceleration(void) const
  {
    return m_COG_ideal.Vdot;
  }

  const Eigen::Vector3d & CompensatedCOGPosition(void) const
  {
    return m_COG_cmp.P;
  }
  const Eigen::Vector3d & CompensatedCOGVelocity(void) const
  {
    return m_COG_cmp.V;
  }
  const Eigen::Vector3d & CompensatedCOGAcceleration(void) const
  {
    return m_COG_cmp.Vdot;
  }

  const Eigen::Vector3d & OutputCOGPosition(void) const
  {
    return m_COG_out.P;
  }
  const Eigen::Vector3d & OutputCOGVelocity(void) const
  {
    return m_COG_out.V;
  }
  const Eigen::Vector3d & OutputCOGAcceleration(void) const
  {
    return m_COG_out.Vdot;
  }

  const Eigen::Vector3d & IdealZMPPosition(void) const
  {
    return m_Pcalpha_ideal;
  }
  const Eigen::Vector3d & IdealZMPVelocity(void) const
  {
    return m_Vcalpha_ideal;
  }

  const Eigen::Vector3d & CompensatedZMPPosition(void) const
  {
    return m_Pcalpha_cmp;
  }
  const Eigen::Vector3d & CompensatedZMPVelocity(void) const
  {
    return m_Vcalpha_cmp;
  }

  const Eigen::Vector3d & OutputZMPPosition(void) const
  {
    return m_Pcalpha_out;
  }

  const std::vector<Eigen::Vector3d> & Steps(void) const
  {
    return m_steps;
  }
  void setStesps(const std::vector<Eigen::Vector3d> & steps)
  {
    m_steps = steps;
  }

  void push_back(double time, double com_x, double com_y)
  {
    m_steps.push_back(Eigen::Vector3d(time, com_x, com_y));
  }
  void push_back(const Eigen::Vector3d & step)
  {
    m_steps.push_back(step);
  }

  generator(int n_preview, double dt);

  void generate(int n_time);
};

} // namespace mc_planning
