#include <mc_planning/MathFunction.h>
#include <mc_planning/generator.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/logging.h>

using namespace mc_planning::motion_interpolator;

constexpr int X = 0;
constexpr int Y = 1;
constexpr int Z = 2;

namespace mc_planning
{

generator::generator(const CenteredPreviewWindow & preview, double mass, double waist_height)
: preview_(preview), m_ComInterp(std::make_shared<ClampedCubicSpline<unsigned>>(1.0, preview_.dt() / 2.)),
  // XXX Should be only one computation for both to speed up computation
  m_ipm_long{
      linear_control_system::LinearTimeVariantInvertedPendulum(preview, 20000),
      linear_control_system::LinearTimeVariantInvertedPendulum(preview, 20000),
  },
  m_mass(mass), m_waist_height(waist_height)
{
  mc_rtc::log::info("Creating generator with mass={}, waist_height={}", mass, waist_height);

  auto initialize = [this, &waist_height](unsigned axis) {
    m_ipm_short[axis].Initialize();
    m_poles[X] << 1.0, 1.0, 150.0;
    m_virtual_height[axis].setZero(preview_.size());
    m_ipm_long[axis].initMatrices(waist_height);
  };
  initialize(X);
  initialize(Y);

  auto previewSize = preview_.size();
  mc_rtc::log::info("Initializing with size {} ", previewSize);
  m_cog_height.setZero(previewSize);
  m_cog_dot_height.setZero(previewSize);
  m_cog_ddot_height.setZero(previewSize);
  m_COG_ideal.P << 0.0, 0.0, waist_height;
}

void generator::setupCOGHeight(unsigned n_current)
{
  // The first time, initialize the whole trajectory
  // with constant waist height
  // Ignores lateral sway during the trajectory?
  if(n_current == 0)
  {
    m_ComInterp->push_back(0u, m_waist_height);
    m_ComInterp->push_back(preview_.index(Time(m_steps.back().t())), m_waist_height);
    m_ComInterp->update();
  }

  for(const auto & step : preview_.all())
  {
    auto n = step.index();
    m_ComInterp->get(n_current + n, m_cog_height[n], m_cog_dot_height[n], m_cog_ddot_height[n]);
  }

  // Interpolated value at current time
  unsigned current = preview_.center();
  m_COG_ideal.P(Z) = m_cog_height[current];
  m_COG_ideal.V(Z) = m_cog_dot_height[current];
  m_COG_ideal.Vdot(Z) = m_cog_ddot_height[current];
}

void generator::setupTimeTrajectories(unsigned n_current)
{
  auto currentPreview = preview_.all(Index(n_current));
  m_steps.nextWindow(currentPreview.startTime());
  Eigen::VectorXd & px_ref = m_ipm_long[X].p_ref();
  Eigen::VectorXd & py_ref = m_ipm_long[Y].p_ref();

  for(const auto & current : currentPreview)
  {
    unsigned i = current.localIndex();
    const double currTime = current.time();

    m_virtual_height[X](i) = 0.0;
    m_virtual_height[Y](i) = 0.0;

    // Move to next step if necessary
    m_steps.update(currTime);
    const auto & prevStep = m_steps.previous();
    if(!m_steps.isLastStep())
    {
      const auto & nextStep = m_steps.next();
      /**
       * @brief Interpolates the reference ZMP from one step to the next
       * @param axis 0: Step CoM x, 0: Step CoM Y
       */
      auto interpolateRefXY = [&](const Eigen::Index axis) {
        return prevStep.step()(axis)
               + (nextStep.step()(axis) - prevStep.step()(axis))
                     * polynomial3((currTime - prevStep.t()) / (nextStep.t() - prevStep.t()));
      };

      px_ref(i) = interpolateRefXY(0);
      py_ref(i) = interpolateRefXY(1);
    }
    else
    { /* No previous step provided, start with the ZMP at the next step */
      px_ref(i) = prevStep.step().x();
      py_ref(i) = prevStep.step().y();
    }

    m_ipm_long[X].w2(i) =
        (mc_rtc::constants::GRAVITY + m_cog_ddot_height[i]) / (m_cog_height[i] - m_virtual_height[X](i));
    m_ipm_long[Y].w2(i) =
        (mc_rtc::constants::GRAVITY + m_cog_ddot_height[i]) / (m_cog_height[i] - m_virtual_height[Y](i));
  }

  unsigned current = preview_.center();
  m_Pcalpha_ideal(Z) = (m_virtual_height[X](current) + m_virtual_height[Y](current)) / 2.0;
}

void generator::generateTrajectories()
{
  StatePV COG_ideal_pre_next(m_COG_ideal.P, m_COG_ideal.V);
  Eigen::Vector3d Pcalpha_ideal_pre_next(m_Pcalpha_ideal);

  /**
   * Computes long-term trajectory
   */
  auto computeLongTerm = [&](unsigned axis) {
    m_ipm_long[axis].update();

    // Current time
    m_ipm_long[axis].getState(0, m_COG_ideal_pre.P(axis), m_COG_ideal_pre.V(axis), m_COG_ideal_pre.Vdot(axis),
                              m_Pcalpha_ideal_pre(axis), m_Vcalpha_ideal_pre(axis));
    // Next timestep
    m_ipm_long[axis].getState(1, m_COG_ideal.P(axis), m_COG_ideal.V(axis), m_COG_ideal.Vdot(axis),
                              m_Pcalpha_ideal(axis), m_Vcalpha_ideal(axis));
  };

  /**
   * Compute short term trajectory
   * Depends on the long term trajectory results
   *
   * Generates the CoM and ZMP modification to be applied to the ideal long-term so that the short term
   * - m_COG_cmp: Modification of the ideal CoG state (position and velocity)
   * - m_Pcalpha_cmp: Modifiecation of the ideal ZMP state (position and velocity)
   **/
  auto computeShortTerm = [&](unsigned axis) {
    unsigned current = preview_.center();
    double omega2 = m_ipm_long[axis].w2(current);
    double omega = sqrt(omega2);
    m_ipm_short[axis].setSystemMatrices(omega * m_poles[axis](0), omega * m_poles[axis](1), m_poles[axis](2), omega2,
                                        m_mass);

    m_COG_cmp.P(axis) += COG_ideal_pre_next.P(axis) - m_COG_ideal_pre.P(axis);
    m_COG_cmp.V(axis) += COG_ideal_pre_next.V(axis) - m_COG_ideal_pre.V(axis);
    m_Pcalpha_cmp(axis) += Pcalpha_ideal_pre_next(axis) - m_Pcalpha_ideal(axis);

    m_ipm_short[axis].setStateVariables(m_COG_cmp.P(axis), m_COG_cmp.V(axis), m_Pcalpha_cmp(axis));
    double Fext = 0.0;
    m_ipm_short[axis].update(Fext, preview_.dt());
    m_ipm_short[axis].getStateVariables(m_COG_cmp.P(axis), m_COG_cmp.V(axis), m_Pcalpha_cmp(axis), m_Vcalpha_cmp(axis));
  };

  /** @brief Computes both long-term and short-term trajectories */
  auto computeTrajectories = [&](unsigned axis) {
    computeLongTerm(axis);
    computeShortTerm(axis);
  };

  /**
   * Computes the desired output CoG and ZMP state composed
   * of the ideal long-term trajectory output modified by the short-term
   * trajectory to ensure continuity
   */
  auto applyShortTermCompensation = [this]() {
    m_COG_out.P = m_COG_ideal.P + m_COG_cmp.P;
    m_COG_out.V = m_COG_ideal.V + m_COG_cmp.V;
    m_COG_out.Vdot = m_COG_ideal.Vdot + m_COG_cmp.Vdot;

    m_Pcalpha_out = m_Pcalpha_ideal + m_Pcalpha_cmp;
    m_Pcalpha_motion_out = m_Pcalpha_out + m_Vcalpha_ideal * m_omega_valpha * preview_.dt();
  };

  computeTrajectories(X);
  computeTrajectories(Y);
  applyShortTermCompensation();
}

void generator::generate(unsigned n_time)
{
  if(!m_steps.isValid())
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[generator] Invalid reference provided, call steps()");
  }
  setupCOGHeight(n_time);
  setupTimeTrajectories(n_time);
  generateTrajectories();
}

void generator::addToLogger(mc_rtc::Logger & logger)
{
  // clang-format off
  // Requested ideal trajectories
  logger.addLogEntry("generator_IdealCOGPosition",       [this]() { return this->IdealCOGPosition(); });
  logger.addLogEntry("generator_IdealZMPPosition",       [this]() { return this->IdealZMPPosition(); });
  // Generated trajectories
  logger.addLogEntry("generator_OutputCOGPosition",      [this]() { return this->OutputCOGPosition(); });
  logger.addLogEntry("generator_OutputZMPPosition",      [this]() { return this->OutputZMPPosition(); });
  logger.addLogEntry("generator_CompensatedCOGPosition", [this]() { return this->CompensatedCOGPosition(); });
  logger.addLogEntry("generator_CompensatedZMPPosition", [this]() { return this->CompensatedZMPPosition(); });
  // clang-format on
}

void generator::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntry("generator_IdealCOGPosition");
  logger.removeLogEntry("generator_IdealZMPPosition");
  logger.removeLogEntry("generator_OutputCOGPosition");
  logger.removeLogEntry("generator_OutputZMPPosition");
  logger.removeLogEntry("generator_CompensatedCOGPosition");
  logger.removeLogEntry("generator_CompensatedZMPPosition");
}

} /* namespace mc_planning */
