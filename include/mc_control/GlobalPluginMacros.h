/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>

#include <mc_rtc/version.h>

/* Set of macros to assist with the writing of a GlobalPlugin */

/** A simple compile-time versus run-time version checking macro
 *
 * If you are not relying on EXPORT_MC_RTC_PLUGIN you should use this in your
 * MC_RTC_GLOBAL_PLUGIN implementation
 *
 */
#define MC_RTC_GLOBAL_PLUGIN_CHECK_VERSION(NAME)                                                                     \
  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())                                                                    \
  {                                                                                                                  \
    mc_rtc::log::error("{} was compiled with {} but mc_rtc is currently at version {}, you might experience subtle " \
                       "issues and should recompile your code",                                                      \
                       NAME, mc_rtc::MC_RTC_VERSION, mc_rtc::version());                                             \
  }

#ifndef MC_RTC_BUILD_STATIC

#  define EXPORT_MC_RTC_PLUGIN(NAME, TYPE)                                          \
    extern "C"                                                                      \
    {                                                                               \
      GLOBAL_PLUGIN_API void MC_RTC_GLOBAL_PLUGIN(std::vector<std::string> & names) \
      {                                                                             \
        MC_RTC_GLOBAL_PLUGIN_CHECK_VERSION(NAME)                                    \
        names = {NAME};                                                             \
      }                                                                             \
                                                                                    \
      GLOBAL_PLUGIN_API void destroy(mc_control::GlobalPlugin * ptr)                \
      {                                                                             \
        delete ptr;                                                                 \
      }                                                                             \
                                                                                    \
      GLOBAL_PLUGIN_API mc_control::GlobalPlugin * create(const std::string &)      \
      {                                                                             \
        return new TYPE();                                                          \
      }                                                                             \
    }

#else

#  include <mc_control/GlobalPluginLoader.h>

#  define EXPORT_MC_RTC_PLUGIN(NAME, TYPE)                                                               \
    namespace                                                                                            \
    {                                                                                                    \
    static auto registered = []() {                                                                      \
      using fn_t = std::function<TYPE *()>;                                                              \
      mc_control::GlobalPluginLoader::loader().register_object(NAME, fn_t([]() { return new TYPE(); })); \
      return true;                                                                                       \
    }();                                                                                                 \
    }

#endif
