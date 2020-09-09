/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/*! Interface used to load observers */

#include <mc_observers/api.h>
#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>

namespace mc_rtc
{
namespace gui
{
struct StateBuilder;
} // namespace gui
} // namespace mc_rtc

namespace mc_control
{
struct MCController;
} // namespace mc_control

namespace mc_rbdyn
{

struct Robots;
}

namespace mc_observers
{

/**
 * @brief State observation API
 *
 * All new observers must inherit from this Observer class, and implement the
 * required virtual functions (at least reset, run, updateRobots). The framework
 * provides a mechanism to load state observation pipelines (see ObserverPipeline) consisting
 * of multiple sequences of observers running sequentially, each estimating part
 * of the robot state. In that case, observers are loaded by the framework from
 * libraries, and must be exported using the EXPORT_OBSERVER_MODULE(NAME, TYPE)
 * function defined in ObserverMacros.h
 *
 * Each observer has access to the public members of its parent controller, and thus can
 * access properties of the robots, sensors, datastore, logger, gui,
 * and set its estimation results to the controller's realRobots instances.
 */
struct MC_OBSERVERS_DLLAPI Observer
{
  Observer(const std::string & type, double dt) : type_(type), dt_(dt) {}
  virtual ~Observer() = default;

  /**
   * @brief Configure observer
   *
   * @param config Configuration for this observer. Refer to the JSON Schema
   * documentation fo each observer for details on supported entries:
   * https://jrl-umi3218.github.io/mc_rtc/json.html#Observers-objects
   */
  virtual void configure(const mc_control::MCController & /*ctl*/, const mc_rtc::Configuration & /*config*/) {}

  /*! \brief Reset estimator.
   *
   * The reset function should make sure that the observer is started in a state
   * consistent with the current robot state.
   *
   * This function is called after MCController::reset()
   **/
  virtual void reset(const mc_control::MCController & ctl) = 0;

  /*! \brief Compute observer state
   *
   * This function is called before MCController::run()
   **/
  virtual bool run(const mc_control::MCController & ctl) = 0;

  /*! \brief Update the real robot state from the observed state
   *
   * This function is expected to write the estimated parameters to the
   * corresponding real robot instances ctl.realRobots(), and compute the necessary information
   * to ensure a consistent state (e.g calling forward velocity after
   * updating joint velocities, etc).
   *
   * This function is called after Observer::run() if the observer is declared
   * to update the real robot in the pipeline configuration.
   *
   * \param ctl Controller running this observer
   **/
  virtual void update(mc_control::MCController & ctl) = 0;

  /**
   * @brief Set the observer's name
   *
   * @param name Name of the observer
   */
  inline void name(const std::string & name)
  {
    name_ = name;
  }

  /**
   * @brief Returns the observer's name
   */
  inline const std::string & name() const noexcept
  {
    return name_;
  }

  /*! \brief Add observer entries to the logger under the categrory "category + name()". */
  void addToLogger_(const mc_control::MCController & ctl, mc_rtc::Logger & logger, const std::string & category = "")
  {
    addToLogger(ctl, logger, category + "_" + name_);
  }

  /*! \brief Remove observer from logger. */
  void removeFromLogger_(mc_rtc::Logger & logger, std::string category = "")
  {
    removeFromLogger(logger, category + "_" + name_);
  }

  /*! \brief Add observer to the gui under category {category, name()} */
  void addToGUI_(const mc_control::MCController & ctl,
                 mc_rtc::gui::StateBuilder & gui,
                 std::vector<std::string> category = {})
  {
    category.push_back(name_);
    addToGUI(ctl, gui, category);
  }

  /*! \brief Remove observer from gui
   *
   * Default implementation removes category {category, name()}
   */
  void removeFromGUI_(mc_rtc::gui::StateBuilder & gui, std::vector<std::string> category = {})
  {
    category.push_back(name_);
    removeFromGUI(gui, category);
  };

  /*! \brief Short description of the observer
   *
   * Used to display a short summary of the observers pipeline to the user, the description should be as short as
   * possible while showing the important information about the observer.
   *
   * The output might end up looking like:
   * Encoders (Position+Velocity) -> KinematicInertial (cutoff=0.01) -> BodySensor
   *
   * \returns Short description of the observer.
   */
  inline const std::string & desc() const noexcept
  {
    return desc_;
  }

  inline const std::string type() const noexcept
  {
    return type_;
  }

  /*! Informative message about the last error */
  inline const std::string & error() const noexcept
  {
    return error_;
  }

  /*! Controller timestep */
  inline double dt() const noexcept
  {
    return dt_;
  }

protected:
  /*! \brief Add observer from logger
   *
   * Default implementation does nothing, each observer implementation is
   * responsible for adding its own log entries
   *
   * @param category Category in which to log this observer
   */
  virtual void addToLogger(const mc_control::MCController &, mc_rtc::Logger &, const std::string & /* category */) {}

  /*! \brief Remove observer from logger
   *
   * Default implementation does nothing, each observer implementation is
   * responsible for removing all logs entry that it added.
   *
   * @param category Category in which this observer entries are logged
   */
  virtual void removeFromLogger(mc_rtc::Logger &, const std::string & /* category */) {}

  /*! \brief Add observer information the GUI.
   *
   * Default implementation does nothing, each observer implementation is
   * responsible for adding its own elements to the GUI.
   *
   * @param category Category in which to add this observer
   */
  virtual void addToGUI(const mc_control::MCController &,
                        mc_rtc::gui::StateBuilder &,
                        const std::vector<std::string> & /* category */)
  {
  }

  /**
   * @brief Remove observer from GUI
   *
   * Default implementation removes the category
   *
   * @param gui
   * @param category Categroy in which the observer was added
   */
  virtual void removeFromGUI(mc_rtc::gui::StateBuilder &, const std::vector<std::string> & category);

protected:
  std::string name_; ///< Observer name
  std::string type_; ///< Observer type
  std::string desc_; ///< Short description of the observer and its configuration
  std::string error_; ///< Descriptive error message to show if the observer failed

  double dt_; ///< Timestep
};

using ObserverPtr = std::shared_ptr<mc_observers::Observer>;

} // namespace mc_observers

#ifdef WIN32
#  define OBSERVER_MODULE_API __declspec(dllexport)
#else
#  if __GNUC__ >= 4
#    define OBSERVER_MODULE_API __attribute__((visibility("default")))
#  else
#    define OBSERVER_MODULE_API
#  endif
#endif
