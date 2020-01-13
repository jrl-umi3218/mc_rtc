/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_observers/Observer.h>

#include <mc_rtc/version.h>

/* Set of macros to assist with the writing of an Observer */

/** A simple compile-time versus run-time version checking macro
 *
 * If you are not relying on EXPORT_OBSERVER_MODULE you should use this in your
 * MC_RTC_OBSERVER_MODULE implementation
 *
 */
#define OBSERVER_MODULE_CHECK_VERSION(NAME)                                                                       \
  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())                                                                 \
  {                                                                                                               \
    LOG_ERROR(NAME << " was compiled with " << mc_rtc::MC_RTC_VERSION << " but mc_rtc is currently at version "   \
                   << mc_rtc::version() << ", you might experience subtle issues and should recompile your code") \
  }

#define EXPORT_OBSERVER_MODULE(NAME, TYPE)                                                    \
  extern "C"                                                                                  \
  {                                                                                           \
    OBSERVER_MODULE_API void MC_RTC_OBSERVER_MODULE(std::vector<std::string> & names)         \
    {                                                                                         \
      OBSERVER_MODULE_CHECK_VERSION(NAME)                                                     \
      names = {NAME};                                                                         \
    }                                                                                         \
    OBSERVER_MODULE_API void destroy(mc_observers::Observer * ptr)                            \
    {                                                                                         \
      delete ptr;                                                                             \
    }                                                                                         \
    OBSERVER_MODULE_API unsigned int create_args_required()                                   \
    {                                                                                         \
      return 3;                                                                               \
    }                                                                                         \
    OBSERVER_MODULE_API mc_observers::Observer * create(const std::string & name,             \
                                                        const double & dt,                    \
                                                        const mc_rtc::Configuration & config) \
    {                                                                                         \
      return new TYPE(name, dt, config);                                                      \
    }                                                                                         \
  }
