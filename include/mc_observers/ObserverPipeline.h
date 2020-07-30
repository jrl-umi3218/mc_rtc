/*
 * Copyright 2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_observers/Observer.h>
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
 * @brief State observation pipeline
 */
struct MC_OBSERVERS_DLLAPI ObserverPipeline
{
  ObserverPipeline(mc_control::MCController & ctl, const std::string & name);
  virtual ~ObserverPipeline() = default;

  /* May be called multiple times */
  void configure(const mc_rtc::Configuration & config);

  /* Load the observers */
  void create(const mc_rtc::Configuration & config);

  /* Reinitialize pipeline configuration from loaded configuration
   * by configure()
   */
  void reconfigure();
  /* Initialize based on the current robot state */
  void reset();
  /* Run this observservation pipeline */
  bool run();

  /* Const accessor to an observer
   *
   * @param name Name of the observer
   * @throws std::runtime_error if the observer is not part of the pipeline
   */
  const Observer & observer(const std::string & name) const
  {
    if(!observersByName_.count(name))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[ObserverPipeline::{}] No observer named {}", name_, name);
    }
    return *observersByName_.at(name);
  }

  bool hasObserver(const std::string & name) const
  {
    return observersByName_.count(name);
  }

  bool hasPipelineObserver(const std::string & name) const
  {
    return std::find_if(
               pipelineObservers_.begin(), pipelineObservers_.end(),
               [&name](const std::pair<mc_observers::ObserverPtr, bool> & obs) { return obs.first->name() == name; })
           != pipelineObservers_.end();
  }

  const std::vector<mc_observers::ObserverPtr> & observers() const
  {
    return observers_;
  }

  const std::vector<std::pair<mc_observers::ObserverPtr, bool>> & pipelineObservers() const
  {
    return pipelineObservers_;
  }

  /* Non-const variant */
  Observer & observer(const std::string & name)
  {
    return const_cast<Observer &>(static_cast<const ObserverPipeline *>(this)->observer(name));
  }

  /*! \brief Short description of the pipeline */
  virtual const std::string & desc() const
  {
    return desc_;
  }
  const std::string & name() const
  {
    return name_;
  }

  void addToLogger();
  void removeFromLogger();
  void addToGUI();
  void removeFromGUI(mc_rtc::gui::StateBuilder &);

protected:
  void observers(const std::vector<ObserverPtr> & loadedObservers)
  {
    observers_ = loadedObservers;
    for(const auto & observer : observers_)
    {
      if(observersByName_.count(observer->name()))
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "[ObserverPipeline::{}] Observer names must be unique: {} created more than once.", observer->name());
      }
      observersByName_[observer->name()] = observer;
    }
  }

protected:
  mc_control::MCController & ctl_; ///< Controller to which this pipeline is bound
  std::string name_ = {"DefaultPipeline"}; ///< Name of this pipeline
  std::string desc_ = {""};

  mc_rtc::Configuration config_; ///< Initial configuration (from configuration files)

  /* Short descriptive description of the observer used for CLI logging */

  std::vector<mc_observers::ObserverPtr> observers_; ///< Loaded observers
  std::map<std::string, mc_observers::ObserverPtr>
      observersByName_; ///< Mapping between loaded observers and their name

  /** Observers that will be run by the pipeline.
   *
   * The pair contains:
   * - The observer to run
   * - A boolean set to true if the observer updates the real robot instance
   *
   * Provided by MCGlobalController */
  std::vector<std::pair<mc_observers::ObserverPtr, bool>> pipelineObservers_;
};

} // namespace mc_observers
