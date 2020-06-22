#pragma once
// #include <Math/ControlSystem/LIPMControlByPoleAssignWithExternalForce.h>
// #include <Math/ControlSystem/LinearTimeVariantInvertedPendulum.h>
// #include <Model/State.h>
#include <mc_planning/ClampedCubicSpline.h>
#include <mc_planning/LIPMControlByPoleAssignWithExternalForce.h>
#include <mc_planning/LinearTimeVariantInvertedPendulum.h>
#include <mc_planning/State.h>
#include <mc_rtc/log/Logger.h>
#include <list>

namespace mc_planning
{

template<typename Step>
struct TimedStep
{
  TimedStep(double t, const Step & step) noexcept : t_(t), step_(step) {}

  const double & t() const noexcept
  {
    return t_;
  }

  double & t() noexcept
  {
    return t_;
  }

  const Step & step() const noexcept
  {
    return step_;
  }

  Step & step() noexcept
  {
    return step_;
  }

  // For std::set
  bool operator<(const TimedStep & t) const noexcept
  {
    return (this->t_ < t.t_);
  }

  bool operator<=(const TimedStep & t) const noexcept
  {
    return (this->t_ <= t.t_);
  }

  void operator+(const TimedStep & t) noexcept
  {
    t_ += t.t();
    step_ += t.step();
  }

  TimedStep & operator+=(const TimedStep & rhs) noexcept
  {
    t_ += rhs.t();
    step_ += rhs.step();
    return *this;
  }

  void operator-(const TimedStep & t) noexcept
  {
    t_ += t.t();
    step_ += t.step();
  }

  TimedStep & operator-=(const TimedStep & rhs) noexcept
  {
    t_ -= rhs.t();
    step_ -= rhs.step();
    return *this;
  }

protected:
  double t_;
  Step step_;
};

template<typename Step>
bool operator<(const TimedStep<Step> & s1, const TimedStep<Step> & s2)
{
  return s1 < s2;
}

template<typename Step>
std::ostream & operator<<(std::ostream & os, const TimedStep<Step> & step)
{
  os << step.t() << " -> " << step.step().transpose();
  return os;
}

/**
 * @brief Helper class to manage a list of discrete steps and provide efficient
 * sequential lookup of next/previous step.
 *
 * Steps have the following properties:
 * - Each step is defined by
 *   - It's start time Step::t()
 *   - It's value Step::step()  (may be any arbitrary type)
 * - Steps are discrete events and do not have a specified duration
 * - Steps are sorted from oldest to most recent
 *
 * Example
 * \code{.cpp}
 *  PreviewSteps<Eigen::Vector2d> steps;
 *  steps.add(        {1.6, {-0.2, 0.0}});
 *  steps.addRelative({1.6, { 0.0, 0.0}});
 *  steps.addRelative({0.1, { 0.0, 0.0}});
 *  steps.addRelative({1.6, { 0.0, 0.0}});
 *  steps.addRelative({0.1, { 0.2, 0.095}});
 *  steps.addRelative({1.6, { 0.0, 0.0}});
 *  steps.addRelative({0.1, { 0.0, -0.19}});
 *  steps.addRelative({1.6, { 0.0, 0.0}});
 *  steps.addRelative({0.1, {-0.2, 0.095}});
 *  steps.addRelative({1.6, { 0.0, 0.0}});
 *  steps.initialize();
 *  mc_rtc::log::info("Desired steps:\nTime\tCoM X\tCoM Y\n{}", mc_rtc::io::to_string(steps.steps(), "\n"));
 *
 *  double dt = 0.005;
 *  double previewTime = 1.6;
 *  const unsigned n_preview = static_cast<unsigned>(std::lround(previewTime / dt));
 *
 *  // Iterate over a preview window and perform computations
 *  // based on the previous and next steps
 *  // In a real-world use, this would most likely be used to compute a
 *  // smooth trajectory between the discrete steps.
 *  auto iteratePreview = [&](double startTime)
 *  {
 *    for(unsigned i = 0; i < n_preview * 2 + 1; i++)
 *    {
 *      double currTime = startTime + i*dt;
 *      // Update time within the preview window
 *      steps.update(currTime);
 *      if(!steps.isLastStep())
 *      {
 *        const auto & nextStep = steps.next();
 *        const auto & prevStep = steps.previous();
 *        // Do something with next and previous steps, such as interpolate
 *        // between them to generate a smooth reference trajectory
 *      }
 *      else
 *      {
 *        // We are past the last step, handle this case
 *        const auto & lastStep = steps.previous();
 *      }
 *    }
 *  }
 *
 * // Real-time control loop
 * bool run()
 * {
 *  // Start next preview window
 *  steps.nextWindow(t);
 *  iteratePreview(t);
 *  t+=dt;
 *  // Now you can modify future steps for the next preview window
 *  double transitionTime = 0.2; // 0.2 second to transition
 *  double delay = 0.5; // Transition in 0.5s in the future. The previous step
 *  will be extended until the transition starts.
 *  steps.changeFutureSteps(
 *  {
 *    t+previewTime+delay, // Extend the previous step until this time
 *    {t+previewTime+delay+transitionTime, {0., -0.15}} // Transition to the
 *    next steps
 *  });
 * }
 * \endcode
 */
template<typename Step>
struct MC_PLANNING_DLLAPI PreviewSteps
{
  using value_type = TimedStep<Step>;
  using PreviewStepsSet = std::set<TimedStep<Step>>;
  using iterator = typename PreviewStepsSet::iterator;
  using reverse_iterator = typename PreviewStepsSet::reverse_iterator;

  /**
   * @brief Inserts a new step
   *
   * A step may be added at any time, including within the current preview
   * window
   *
   * @param step Step to add
   */
  void add(const TimedStep<Step> & step)
  {
    if(step.t() < (*steps_.begin()).t())
    {
      mc_rtc::log::error("[PreviewSteps] can't create a step before the start of the preview window");
      return;
    }
    steps_.insert(step);
  }

  void reset(const PreviewStepsSet & steps)
  {
    steps_ = steps;
  }

  /**
   * @brief Adds a step relative to the last step
   *
   * @throws std::runtime_error If steps() is empty
   *
   * @param step Relative step to add
   */
  void addRelative(const TimedStep<Step> & step)
  {
    if(steps_.empty())
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[PreviewSteps] can't add a relative step to an empty set");
      return;
    }
    auto last = this->back();
    last += step;
    add(last);
  }

  /**
   * @brief Replaces all footsteps following the first of the new provided steps
   *
   * @param nextSteps
   */
  void replaceAfter(const PreviewStepsSet & nextSteps)
  {
    if(nextSteps.empty()) return;
    deleteAfter(nextSteps.begin()->t());
    std::copy(nextSteps.begin(), nextSteps.end(), std::inserter(steps_, steps_.begin()));
  }

  void deleteAfter(double t)
  {
    auto it = std::find_if(steps_.begin(), steps_.end(), [t](const value_type & step) { return step.t() >= t; });
    if(it != steps_.end())
    {
      steps_.erase(it, steps_.end());
    }
  }

  void changeFutureSteps(double startTransition, const std::vector<value_type> & steps)
  {
    mc_rtc::log::info("Delete after {}", startTransition);
    deleteAfter(startTransition);
    // Extend previous step until start of transition
    auto transitionStep = back();
    mc_rtc::log::info("Duplicating previous step {}", transitionStep);
    transitionStep.t() = startTransition;
    mc_rtc::log::info("Pre-Transition step {}", transitionStep);
    steps_.insert(transitionStep);
    mc_rtc::log::info("Inserting steps");
    for(const auto & step : steps)
    {
      mc_rtc::log::info("Step {}", step);
      steps_.insert(step);
    }
  }

  /**
   * @brief To be called after adding the initial footsteps
   *
   * Requires at least one footstep.
   *
   * @throws std::runtime_error if there isn't at least one footstep
   */
  void initialize()
  {
    if(steps_.size() < 1)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[PreviewSteps] requires at least one step");
    }
    else if(steps_.size() == 1)
    {
      previous_ = steps_.begin();
      next_ = previous_;
    }
    else
    {
      previous_ = steps_.begin();
      next_ = std::next(previous_);
      mc_rtc::log::info("Init {}, {}", previous_->t(), next_->t());
    }
  }

  /**
   * @name Real-time functions
   * These functions are intended to be called sequentially while iterating over the preview window in the following
   * way: <ul> <li>nextWindow(double t): Sets up a new preview window starting at time t, updates the starting step, and
   * removes old steps prior to t</li> <li>for each time in the preview window: <ul> <li>next(), previous() provide the
   * previous/next step for the current time. Warning: if the last step is reached (`isLastStep() == true`), then next()
   * = previous()</li> <li>update(double t): move along the preview window</li>
   *    </ul>
   * </li>
   * <li>Repeat with the next preview window</li>
   * </ul>
   * @{
   */

  /**
   * @brief Updates the time along the preview window
   *
   * Finds the previous and next step for the current time in the preview window
   *
   * @param t Curent time within the preview window
   */
  void update(double t) noexcept
  {
    if(!isLastStep() && t >= next_->t())
    {
      // mc_rtc::log::info("Change step");
      previous_ = next_;
      ++next_;
      // if(next_ == steps_.end())
      // { // only happens when the end of the step plan is in the current preview horizon (rare)
      //   next_ = previous_;
      // }
      // mc_rtc::log::info("After Change step: {}, {}", previous().t(), next().t());
    }
  }

  /**
   * @brief Whether we reached the last step
   *
   * @waning When on the last step, there is no next step available, and next() == previous()
   */
  bool isLastStep() const noexcept
  {
    return next_ == steps_.end();
  }

  /**
   * @brief Moves to the next preview window starting at time t
   *
   * Removes old footsteps outside of the window, and update previous/next
   * steps accordingly
   *
   * @param t Time of the start of the preview window
   */
  void nextWindow(double t)
  {
    // Check whether old footsteps need to be removed
    if(steps_.size() > 2 && t > std::next(steps_.begin())->t())
    {
      steps_.erase(steps_.begin());
    }

    previous_ = steps_.begin();
    next_ = std::next(previous_);
    // mc_rtc::log::info("Rewinding for next window: prev {}, next {}", previous(), next());
  }

  /**
   * @brief Returns the previous step w.r.t last call to update(double t)
   *
   * @warning If no next step is available, then next() = previous(). You can
   * check whether this is the last step ahead of time by calling isLastStep()
   *
   * @return Next step
   */
  const TimedStep<Step> & previous() const noexcept
  {
    return *previous_;
  }

  /**
   * @brief Returns the next step w.r.w last call to update(double t)
   *
   * @warning If no next step is available, then next() = previous(). You can
   * check whether this is the last step ahead of time by calling isLastStep()
   *
   * @return Next step
   */
  const TimedStep<Step> & next() const noexcept
  {
    return *next_;
  }
  /** @} */ // end of group real-time

  /**
   * @brief Get the closest footstep prior to time t
   *
   * @param t Lookup time
   *
   * @note This funciton is not intended for real-time performace and has a
   * complexity in `O(steps)`
   *
   * @return Nearest past footstep with time <= t
   */
  const TimedStep<Step> & previous(double t) const noexcept
  {
    if(t >= back().t()) return back();
    auto it =
        std::find_if(steps_.rbegin(), steps_.rend(), [&t](const TimedStep<Step> & step) { return step.t() <= t; });
    if(it != steps_.rend())
    {
      return *it;
    }
    else
    {
      return *steps_.begin();
    }
  }

  /**
   * @brief Get the closest future footstep after time t
   *
   * @param t Lookup time
   *
   * @note This function is not intended for real-time performace and has a
   * complexity in `O(steps)`.
   *
   * @return Nearest future footstep with time <= t
   */
  const TimedStep<Step> & next(double t) const noexcept
  {
    auto it = std::find_if(steps_.begin(), steps_.end(), [&t](const TimedStep<Step> & step) { return step.t() >= t; });
    if(it != steps_.end())
    {
      return *it;
    }
    return previous();
  }

  const PreviewStepsSet & steps() const noexcept
  {
    return steps_;
  }

  const TimedStep<Step> & back() const noexcept
  {
    return *std::prev(steps_.end());
  }

  const TimedStep<Step> & front() const noexcept
  {
    return *steps_.begin();
  }

  bool isValid() const noexcept
  {
    return steps_.size() > 1;
  }

protected:
  PreviewStepsSet steps_; // Ordered set of pairs of [time, Step]

  /**
   * @name Real-time update
   * @{
   */
  typename PreviewStepsSet::iterator previous_ = std::end(steps_); ///< Previous footstep
  typename PreviewStepsSet::iterator next_ = std::end(steps_); ///< Next footstep
  ///@}
};

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
  void changeFutureSteps(double transitionTime,
                         const std::vector<TimedStep<Eigen::Vector2d>> & futureSteps)
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
