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
#define OBSERVER_MODULE_CHECK_VERSION(NAME)                                                                      \
  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())                                                                \
  {                                                                                                              \
    mc_rtc::log::error(                                                                                          \
        "{} was compiled with {} but mc_rtc is currently at version {}, you might experience subtle issues and " \
        "should recompile your code",                                                                            \
        NAME, mc_rtc::MC_RTC_VERSION, mc_rtc::version());                                                        \
  }

#ifndef MC_RTC_BUILD_STATIC

#  define EXPORT_OBSERVER_MODULE(NAME, TYPE)                                                           \
    extern "C"                                                                                         \
    {                                                                                                  \
      OBSERVER_MODULE_API void MC_RTC_OBSERVER_MODULE(std::vector<std::string> & names)                \
      {                                                                                                \
        OBSERVER_MODULE_CHECK_VERSION(NAME)                                                            \
        names = {NAME};                                                                                \
      }                                                                                                \
      OBSERVER_MODULE_API void destroy(mc_observers::Observer * ptr)                                   \
      {                                                                                                \
        delete ptr;                                                                                    \
      }                                                                                                \
      OBSERVER_MODULE_API unsigned int create_args_required()                                          \
      {                                                                                                \
        return 2;                                                                                      \
      }                                                                                                \
      OBSERVER_MODULE_API mc_observers::Observer * create(const std::string & type, const double & dt) \
      {                                                                                                \
        return new TYPE{type, dt};                                                                     \
      }                                                                                                \
    }

#else

#  include <mc_observers/ObserverLoader.h>

#  define EXPORT_OBSERVER_MODULE(NAME, TYPE)                                                           \
    namespace                                                                                          \
    {                                                                                                  \
    static auto registered = []() {                                                                    \
      using fn_t = std::function<TYPE *(const double &)>;                         \
      mc_observers::ObserverLoader::register_object(                                                   \
          NAME, fn_t([](const double & dt) { return new TYPE(NAME, dt); })); \
      return true;                                                                                     \
    }();                                                                                               \
    }

#endif
