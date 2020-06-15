#include "generator.h"
#include <mc_planning/MathFunction.h>
#include <mc_rtc/constants.h>
#include <iostream>
// #include <Math/MathFunction.h>
// #include <Math/Physics.h>

using namespace mc_planning::motion_interpolator;

const double waist_height = 0.8;
constexpr int X = 0;
constexpr int Y = 1;
constexpr int Z = 2;

namespace mc_planning
{

generator::generator(int n_preview, double dt)
: m_ComInterp(NULL), m_Pcalpha_ideal_pre(Eigen::Vector3d::Zero()), m_Pcalpha_ideal(Eigen::Vector3d::Zero()),
  m_Pcalpha_cmp(Eigen::Vector3d::Zero()), m_Vcalpha_ideal_pre(Eigen::Vector3d::Zero()),
  m_Vcalpha_ideal(Eigen::Vector3d::Zero()), m_Vcalpha_cmp(Eigen::Vector3d::Zero()),
  m_Pcalpha_out(Eigen::Vector3d::Zero()), m_Pcalpha_motion_out(Eigen::Vector3d::Zero()), m_n_preview(n_preview),
  m_n_steps(0), m_dt(dt), m_omega_valpha(0.0), m_mass(60.0)
{
  // setInterpolator<int>(m_ComInterp, "clamped-cubic-spline", m_dt);
  m_ComInterp = new ClampedCubicSpline<int>(1.0, m_dt / 2);

  m_ipm_long[X].Initialize(m_dt, m_n_preview, 20000);
  m_ipm_long[Y].Initialize(m_dt, m_n_preview, 20000);
  m_ipm_short[X].Initialize();
  m_ipm_short[Y].Initialize();

  m_poles[X] << 1.0, 1.0, 150.0;
  m_poles[Y] << 1.0, 1.0, 150.0;

  m_virtual_height[X].setZero(m_n_preview * 2 + 1);
  m_virtual_height[Y].setZero(m_n_preview * 2 + 1);
  m_cog_height.setZero(m_n_preview * 2 + 1);
  m_cog_dot_height.setZero(m_n_preview * 2 + 1);
  m_cog_ddot_height.setZero(m_n_preview * 2 + 1);

  m_ipm_long[X].initMatrices(waist_height);
  m_ipm_long[Y].initMatrices(waist_height);

  m_COG_ideal.P << 0.0, 0.0, waist_height;
}

generator::~generator()
{
  delete m_ComInterp;
}

void generator::setupCOGHeight(int n_current)
{
#if 0
  for( int i = 0 ; i <= 25; i++ ){
    double t = (double)i * 36.0 * m_dt;
    double waist = 0.2 * sin(2.0 * M_PI * t) + waist_height;
    m_ComInterp->push_back(i * 40 + 620,  waist);
  }
#endif
  if(n_current == 0)
  {
    m_ComInterp->push_back(0.0, waist_height);
    m_ComInterp->push_back(lround(m_steps.back()(0) / m_dt), waist_height);
    m_ComInterp->update();
  }

  for(int n_step = -m_n_preview; n_step <= m_n_preview; n_step++)
  {
    m_ComInterp->get(n_current + n_step + m_n_preview, m_cog_height[n_step + m_n_preview],
                     m_cog_dot_height[n_step + m_n_preview], m_cog_ddot_height[n_step + m_n_preview]);
  }

  m_COG_ideal.P(Z) = m_cog_height[m_n_preview];
  m_COG_ideal.V(Z) = m_cog_dot_height[m_n_preview];
  m_COG_ideal.Vdot(Z) = m_cog_ddot_height[m_n_preview];
}

void generator::setupTimeTrajectories(int n_current)
{
  Eigen::VectorXd & px_ref = m_ipm_long[X].p_ref();
  Eigen::VectorXd & py_ref = m_ipm_long[Y].p_ref();
  int n_steps_loop = m_n_steps;
  for(int i = 0; i < m_n_preview * 2 + 1; i++)
  {
    px_ref(i) = 0.0;
    py_ref(i) = 0.0;
    m_virtual_height[X](i) = 0.0;
    m_virtual_height[Y](i) = 0.0;

    double tm = (double)(i + n_current) * m_dt;
    if(tm >= m_steps[n_steps_loop](0))
    {
      px_ref(i) = m_steps[n_steps_loop](1)
                  + (m_steps[n_steps_loop + 1](1) - m_steps[n_steps_loop](1))
                        * polynomial3((tm - m_steps[n_steps_loop](0))
                                      / (m_steps[n_steps_loop + 1](0) - m_steps[n_steps_loop](0)));
      py_ref(i) = m_steps[n_steps_loop](2)
                  + (m_steps[n_steps_loop + 1](2) - m_steps[n_steps_loop](2))
                        * polynomial3((tm - m_steps[n_steps_loop](0))
                                      / (m_steps[n_steps_loop + 1](0) - m_steps[n_steps_loop](0)));
    }
    else
    {
      px_ref(i) = m_steps[n_steps_loop + 1](1);
      py_ref(i) = m_steps[n_steps_loop + 1](2);
    }

    m_ipm_long[X].w2(i) =
        (mc_rtc::constants::GRAVITY + m_cog_ddot_height[i]) / (m_cog_height[i] - m_virtual_height[X](i));
    m_ipm_long[Y].w2(i) =
        (mc_rtc::constants::GRAVITY + m_cog_ddot_height[i]) / (m_cog_height[i] - m_virtual_height[Y](i));

    if(n_steps_loop < m_steps.size() - 1)
    {
      if(tm >= m_steps[n_steps_loop + 1](0)) n_steps_loop++;
    }
  }

  m_Pcalpha_ideal(Z) = (m_virtual_height[X](m_n_preview) + m_virtual_height[Y](m_n_preview)) / 2.0;

  double tm_cur = (double)n_current * m_dt;
  if(m_n_steps < m_steps.size() - 1)
  {
    if(tm_cur > m_steps[m_n_steps + 1](0)) m_n_steps++;
  }

#if 0
  std::cout << "time=" << (double)n_current*m_dt << std::endl;
  std::cout << "cog_height=" << m_cog_height.transpose() << std::endl;
  std::cout << "vr_height=" << m_virtual_height[X].transpose() << std::endl;
#endif
#if 0
  std::cout << "time=" << (double)n_current*m_dt << std::endl;
  std::cout << "cog_height=" << m_cog_height[m_n_preview] << std::endl;
  std::cout << "vr_height=" << m_virtual_height[X][m_n_preview] << std::endl;
  std::cout << "p_ref=" << px_ref[m_n_preview] << "," << py_ref[m_n_preview] << std::endl;
#endif
}

void generator::generateTrajectories(void)
{
  StatePV COG_ideal_pre_next(m_COG_ideal.P, m_COG_ideal.V);
  Eigen::Vector3d Pcalpha_ideal_pre_next(m_Pcalpha_ideal);
  for(int axis = X; axis <= Y; axis++)
  {
    m_ipm_long[axis].update();

    m_ipm_long[axis].getState(0, m_COG_ideal_pre.P(axis), m_COG_ideal_pre.V(axis), m_COG_ideal_pre.Vdot(axis),
                              m_Pcalpha_ideal_pre(axis), m_Vcalpha_ideal_pre(axis));
    m_ipm_long[axis].getState(1, m_COG_ideal.P(axis), m_COG_ideal.V(axis), m_COG_ideal.Vdot(axis),
                              m_Pcalpha_ideal(axis), m_Vcalpha_ideal(axis));

    double omega2 = m_ipm_long[axis].w2(m_n_preview);
    double omega = sqrt(omega2);
    m_ipm_short[axis].setSystemMatrices(omega * m_poles[axis](0), omega * m_poles[axis](1), m_poles[axis](2), omega2,
                                        m_mass);

    m_COG_cmp.P(axis) += COG_ideal_pre_next.P(axis) - m_COG_ideal_pre.P(axis);
    m_COG_cmp.V(axis) += COG_ideal_pre_next.V(axis) - m_COG_ideal_pre.V(axis);
    m_Pcalpha_cmp(axis) += Pcalpha_ideal_pre_next(axis) - m_Pcalpha_ideal(axis);

    m_ipm_short[axis].setStateVariables(m_COG_cmp.P(axis), m_COG_cmp.V(axis), m_Pcalpha_cmp(axis));
    double Fext = 0.0;
    m_ipm_short[axis].update(Fext, m_dt);
    m_ipm_short[axis].getStateVariables(m_COG_cmp.P(axis), m_COG_cmp.V(axis), m_Pcalpha_cmp(axis), m_Vcalpha_cmp(axis));
  }

  m_COG_out.P = m_COG_ideal.P + m_COG_cmp.P;
  m_COG_out.V = m_COG_ideal.V + m_COG_cmp.V;
  m_COG_out.Vdot = m_COG_ideal.Vdot + m_COG_cmp.Vdot;

  m_Pcalpha_out = m_Pcalpha_ideal + m_Pcalpha_cmp;
  m_Pcalpha_motion_out = m_Pcalpha_out + m_Vcalpha_ideal * m_omega_valpha * m_dt;
}

void generator::generate(int n_time)
{
  setupCOGHeight(n_time);
  setupTimeTrajectories(n_time);
  generateTrajectories();

#if 0
  std::cout << "COG_out = " << m_COG_out.P.transpose() << std::endl;
  std::cout << "Pcalpha_out = " << m_Pcalpha_out.transpose() << std::endl;

  std::cout << "VCOGdot_ideal = " << m_COG_ideal.Vdot.transpose() << std::endl;
  std::cout << "VCOGdot_cmp = " << m_COG_cmp.Vdot.transpose() << std::endl;
#endif
}

} // namespace mc_planning
