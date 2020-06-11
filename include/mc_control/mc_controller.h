/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/MCController.h>

#include <mc_rtc/version.h>

/** Set of macros to assist with the writing of an MCController */

/** A simple compile-time versus run-time version checking macro
 *
 * If you are not relying on CONTROLLER_CONSTRUCTOR or
 * SIMPLE_CONTROLLER_CONSTRUCTOR you should use this in your MC_RTC_CONTROLLER
 * implementation
 *
 */
#define CONTROLLER_CHECK_VERSION(NAME)                                                                               \
  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())                                                                    \
  {                                                                                                                  \
    mc_rtc::log::error("{} was compiled with {} but mc_rtc is currently at version {}, you might experience subtle " \
                       "issues and should recompile your code",                                                      \
                       NAME, mc_rtc::MC_RTC_VERSION, mc_rtc::version());                                             \
  }

/** Provides a handle to construct the controller with Json config */
#define CONTROLLER_CONSTRUCTOR(NAME, TYPE)                                                                        \
  extern "C"                                                                                                      \
  {                                                                                                               \
    CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> & names)                                \
    {                                                                                                             \
      CONTROLLER_CHECK_VERSION(NAME)                                                                              \
      names = {NAME};                                                                                             \
    }                                                                                                             \
    CONTROLLER_MODULE_API void destroy(mc_control::MCController * ptr)                                            \
    {                                                                                                             \
      delete ptr;                                                                                                 \
    }                                                                                                             \
    CONTROLLER_MODULE_API mc_control::MCController * create(const std::string &,                                  \
                                                            const std::shared_ptr<mc_rbdyn::RobotModule> & robot, \
                                                            const double & dt,                                    \
                                                            const mc_control::Configuration & conf)               \
    {                                                                                                             \
      return new TYPE(robot, dt, conf);                                                                           \
    }                                                                                                             \
  }

/** Provides a handle to construct a generic controller */
#define SIMPLE_CONTROLLER_CONSTRUCTOR(NAME, TYPE)                                                                 \
  extern "C"                                                                                                      \
  {                                                                                                               \
    CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> & names)                                \
    {                                                                                                             \
      CONTROLLER_CHECK_VERSION(NAME)                                                                              \
      names = {NAME};                                                                                             \
    }                                                                                                             \
    CONTROLLER_MODULE_API void destroy(mc_control::MCController * ptr)                                            \
    {                                                                                                             \
      delete ptr;                                                                                                 \
    }                                                                                                             \
    CONTROLLER_MODULE_API mc_control::MCController * create(const std::string &,                                  \
                                                            const std::shared_ptr<mc_rbdyn::RobotModule> & robot, \
                                                            const double & dt,                                    \
                                                            const mc_control::Configuration &)                    \
    {                                                                                                             \
      return new TYPE(robot, dt);                                                                                 \
    }                                                                                                             \
  }
