/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/*! Interface used to load observers */

#include <mc_observers/api.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/GUIState.h>
#include <mc_rtc/log/Logger.h>

namespace mc_observers
{

struct MC_OBSERVER_DLLAPI Observer
{
 public:
  Observer(const std::string& name, double dt, const mc_rtc::Configuration & config = {});
  virtual ~Observer();
  virtual const std::string & name() const;
  double dt() const;

  /** Reset floating base estimate.
   *
   * \param robot Robot from which the initial pose is to be estimated
   *
   */
  virtual void reset(const mc_rbdyn::Robot & controlRobot, const mc_rbdyn::Robot & robot) = 0;

  /** Update floating-base transform of real robot.
   *
   * \param realRobot Measured robot state, to be updated.
   *
   */
  virtual bool run(const mc_rbdyn::Robot & controlRobot, const mc_rbdyn::Robot & realRobot) = 0;

  /** Write observed floating-base transform to the robot's configuration.
   *
   * \param robot Robot state to write to.
   *
   */
  virtual void updateRobot(mc_rbdyn::Robot & robot) = 0;

  virtual void addToLogger(mc_rtc::Logger &) {}
  virtual void removeFromLogger(mc_rtc::Logger &) {}
  virtual void addToGUI(mc_rtc::gui::StateBuilder &) {}
  virtual void removeFromGUI(mc_rtc::gui::StateBuilder &);

 private:
  std::string name_;
  double dt_;
};

} // namespace mc_observers


/* Set of macros to assist with the writing of an Observer */

#ifdef WIN32
#  define OBSERVER_MODULE_API __declspec(dllexport)
#else
#  if __GNUC__ >= 4
#    define OBSERVER_MODULE_API __attribute__((visibility("default")))
#  else
#    define OBSERVER_MODULE_API
#  endif
#endif

#define EXPORT_OBSERVER_MODULE(NAME, TYPE)                                   \
  extern "C"                                                              \
  {                                                                       \
    OBSERVER_MODULE_API void MC_RTC_OBSERVER_MODULE(std::vector<std::string> & names) \
    {                                                                     \
      names = {NAME};                                                     \
    }                                                                     \
    OBSERVER_MODULE_API void destroy(mc_observers::Observer * ptr)              \
    {                                                                     \
      delete ptr;                                                         \
    }                                                                     \
    OBSERVER_MODULE_API unsigned int create_args_required()              \
    {                                                                    \
      return 3;                                                          \
    }                                                                    \
    OBSERVER_MODULE_API mc_observers::Observer * create(const std::string & name, const double & dt, const mc_rtc::Configuration & config)    \
    {                                                                     \
      return new TYPE(name, dt, config);                                          \
    }                                                                     \
  }

#define EXPORT_OBSERVER_MODULE_SIMPLE(NAME, TYPE)                                   \
  extern "C"                                                              \
  {                                                                       \
    OBSERVER_MODULE_API void MC_RTC_OBSERVER_MODULE(std::vector<std::string> & names) \
    {                                                                     \
      names = {NAME};                                                     \
    }                                                                     \
    OBSERVER_MODULE_API void destroy(mc_observers::Observer * ptr)              \
    {                                                                     \
      delete ptr;                                                         \
    }                                                                     \
    OBSERVER_MODULE_API unsigned int create_args_required()              \
    {                                                                    \
      return 2;                                                          \
    }                                                                    \
    OBSERVER_MODULE_API mc_observers::Observer * create(const std::string & name, const double & dt, const mc_rtc::Configuration & config)    \
    {                                                                     \
      return new TYPE(name, dt);                                          \
    }                                                                     \
  }
