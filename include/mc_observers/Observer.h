/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/*! Interface used to load observers */

#include <mc_observers/api.h>

#include <mc_rtc/gui/StateBuilder.h>
#include <mc_rtc/log/Logger.h>

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
  virtual ~Observer() {}

  /**
   * @brief Configure observer
   *
   * @param config Configuration for this observer
   */
  virtual void configure(const mc_control::MCController & /*ctl*/, const mc_rtc::Configuration & /*config*/) {}

  /*! \brief Reset estimator.
   *
   * The reset function should make sure that the observer is started in a state
   * consistent with the current robot state. In most cases, during the first
   * iteration(s) all measurement information required by the observers may not
   * yet be available (e.g filters requiring a history of inputs, finite
   * differences requiring at least two measurements, etc). In addition some
   * observer may require some additional input information from the controller
   * (e.g position of an anchor frame for KinematicInertial observers)
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
   * corresponding real robot instances, and compute the necessary information
   * to ensure a consistent state (e.g calling forward velocity after
   * updating joint velocities, etc).
   *
   * This function is called after Observer::run() if the observer is declared
   * to update the real robot in the pipeline configuration.
   **/
  virtual void updateRobots(mc_control::MCController & ctl) = 0;

  /**
   * @brief Set the observer's name
   *
   * @param name Name of the observer
   */
  void name(const std::string & name)
  {
    name_ = name;
  }

  /**
   * @brief Returns the observer's name
   */
  const std::string & name() const;

  /*! \brief Add observer to the logger.
   *
   * Default implementation calls the observers' addToLogger function with
   * category
   */
  void addToLogger_(mc_control::MCController & ctl, const std::string & category = "")
  {
    addToLogger(ctl, category + "_" + name_);
  }

  /*! \brief Remove observer from logger
   *
   * Default implementation does nothing, each observer implementation is
   * responsible for removing all logs entry that it added.
   */
  virtual void removeFromLogger_(mc_control::MCController & ctl, std::string category = "")
  {
    removeFromLogger(ctl, category + "_" + name_);
  }

  /*! \brief Add observer information the GUI.
   *
   * Default implementation does nothing, each observer implementation is
   * responsible for adding its own elements to the GUI. Default observers will
   * be shown under the tab "Observers->observer name".
   */
  virtual void addToGUI_(mc_control::MCController & ctl, std::vector<std::string> category = {})
  {
    category.push_back(name_);
    addToGUI(ctl, category);
  }
  /*! \brief Remove observer from gui
   *
   * Default implementation does nothing. Each observer is responsible from
   * removing any element it added to the GUI.
   */
  virtual void removeFromGUI_(mc_control::MCController & ctl, std::vector<std::string> category = {})
  {
    category.push_back(name_);
    removeFromGUI(ctl, category);
  };

  /*! \brief Short description of the observer
   *
   * Used to display a short summary of the observers pipeline to the user, the description should be as short as
   * possible while showing the important information about the observer.
   *
   * The output might end up looking like:
   * Observers: Encoders (Position+Velocity) -> KinematicInertial (cutoff=0.01) -> BodySensor
   *
   * \returns Short description of the observer. Default implementation returns
   * its name.
   */
  virtual const std::string & desc() const;

  const std::string type() const
  {
    return type_;
  }

  double dt() const
  {
    return dt_;
  }

protected:
  virtual void addToLogger(mc_control::MCController &, const std::string &) {}
  virtual void removeFromLogger(mc_control::MCController &, const std::string &) {}
  virtual void addToGUI(mc_control::MCController &, const std::vector<std::string> &) {}
  virtual void removeFromGUI(mc_control::MCController &, const std::vector<std::string> &) {}

protected:
  std::string name_;
  std::string type_;

  /* Short descriptive description of the observer used for CLI logging */
  std::string desc_;

  double dt_;
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
