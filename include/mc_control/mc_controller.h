/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/MCController.h>

#ifndef MC_RTC_BUILD_STATIC

#  include <mc_rtc/version.h>

/** Set of macros to assist with the writing of an MCController */

/** A simple compile-time versus run-time version checking macro
 *
 * If you are not relying on CONTROLLER_CONSTRUCTOR or
 * SIMPLE_CONTROLLER_CONSTRUCTOR you should use this in your MC_RTC_CONTROLLER
 * implementation
 *
 */
#  define CONTROLLER_CHECK_VERSION(NAME)                                                                \
    if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())                                                     \
    {                                                                                                   \
      mc_rtc::log::error(                                                                               \
          "{} was compiled with {} but mc_rtc is currently at version {}, you might experience subtle " \
          "issues and should recompile your code",                                                      \
          NAME, mc_rtc::MC_RTC_VERSION, mc_rtc::version());                                             \
    }

/** Provides a handle to construct the controller with Json config */
#  define CONTROLLER_CONSTRUCTOR(NAME, TYPE)                                                          \
    extern "C"                                                                                        \
    {                                                                                                 \
      CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> & names)                  \
      {                                                                                               \
        CONTROLLER_CHECK_VERSION(NAME)                                                                \
        names = {NAME};                                                                               \
      }                                                                                               \
      CONTROLLER_MODULE_API void destroy(mc_control::MCController * ptr)                              \
      {                                                                                               \
        delete ptr;                                                                                   \
      }                                                                                               \
      CONTROLLER_MODULE_API unsigned int create_args_required()                                       \
      {                                                                                               \
        return 4;                                                                                     \
      }                                                                                               \
      CONTROLLER_MODULE_API mc_control::MCController * create(const std::string &,                    \
                                                              const mc_rbdyn::RobotModulePtr & robot, \
                                                              const double & dt,                      \
                                                              const mc_control::Configuration & conf) \
      {                                                                                               \
        return new TYPE(robot, dt, conf);                                                             \
      }                                                                                               \
    }

/** Provides all functions except create */
#  define MULTI_CONTROLLERS_CONSTRUCTOR(NAME0, NEWCTL0, NAME1, NEWCTL1)              \
    extern "C"                                                                       \
    {                                                                                \
      CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> & names) \
      {                                                                              \
        CONTROLLER_CHECK_VERSION(NAME0)                                              \
        names = {NAME0, NAME1};                                                      \
      }                                                                              \
                                                                                     \
      CONTROLLER_MODULE_API void destroy(mc_control::MCController * ptr)             \
      {                                                                              \
        delete ptr;                                                                  \
      }                                                                              \
      CONTROLLER_MODULE_API unsigned int create_args_required()                      \
      {                                                                              \
        return 4;                                                                    \
      }                                                                              \
      CONTROLLER_MODULE_API mc_control::MCController * create(                       \
          const std::string & name,                                                  \
          const mc_rbdyn::RobotModulePtr & rm,                                       \
          const double & dt,                                                         \
          [[maybe_unused]] const mc_control::Configuration & config)                 \
      {                                                                              \
        if(name == NAME0) { return new NEWCTL0; }                                    \
        return new NEWCTL1;                                                          \
      }                                                                              \
    }

/** Provides a handle to construct a generic controller */
#  define SIMPLE_CONTROLLER_CONSTRUCTOR(NAME, TYPE)                                                   \
    extern "C"                                                                                        \
    {                                                                                                 \
      CONTROLLER_MODULE_API void MC_RTC_CONTROLLER(std::vector<std::string> & names)                  \
      {                                                                                               \
        CONTROLLER_CHECK_VERSION(NAME)                                                                \
        names = {NAME};                                                                               \
      }                                                                                               \
      CONTROLLER_MODULE_API void destroy(mc_control::MCController * ptr)                              \
      {                                                                                               \
        delete ptr;                                                                                   \
      }                                                                                               \
      CONTROLLER_MODULE_API unsigned int create_args_required()                                       \
      {                                                                                               \
        return 4;                                                                                     \
      }                                                                                               \
      CONTROLLER_MODULE_API mc_control::MCController * create(const std::string &,                    \
                                                              const mc_rbdyn::RobotModulePtr & robot, \
                                                              const double & dt,                      \
                                                              const mc_control::Configuration &)      \
      {                                                                                               \
        return new TYPE(robot, dt);                                                                   \
      }                                                                                               \
    }

#else

#  include <mc_control/ControllerLoader.h>

#  define CONTROLLER_CONSTRUCTOR(NAME, TYPE)                                                             \
    namespace                                                                                            \
    {                                                                                                    \
    static auto registered = []()                                                                        \
    {                                                                                                    \
      using fn_t = std::function<TYPE *(const std::shared_ptr<mc_rbdyn::RobotModule> &, const double &,  \
                                        const mc_control::Configuration &)>;                             \
      mc_control::ControllerLoader::loader().register_object(                                            \
          NAME, fn_t([](const std::shared_ptr<mc_rbdyn::RobotModule> & robot, const double & dt,         \
                        const mc_control::Configuration & conf) { return new TYPE(robot, dt, conf); })); \
      return true;                                                                                       \
    }();                                                                                                 \
    }

#  define MULTI_CONTROLLERS_CONSTRUCTOR(NAME0, NEWCTL0, NAME1, NEWCTL1)                                   \
    namespace                                                                                             \
    {                                                                                                     \
    static auto registered = []()                                                                         \
    {                                                                                                     \
      using TYPE0 = decltype(NEWCTL0);                                                                    \
      using TYPE1 = decltype(NEWCTL1);                                                                    \
      using fn0_t = std::function<TYPE0 *(const std::shared_ptr<mc_rbdyn::RobotModule> &, const double &, \
                                          const mc_control::Configuration &)>;                            \
      mc_control::ControllerLoader::loader().register_object(                                             \
          NAME0, fn_t([](const std::shared_ptr<mc_rbdyn::RobotModule> & robot, const double & dt,         \
                         const mc_control::Configuration & conf) { return new NEWCTL0; }));               \
      using fn1_t = std::function<TYPE1 *(const std::shared_ptr<mc_rbdyn::RobotModule> &, const double &, \
                                          const mc_control::Configuration &)>;                            \
      mc_control::ControllerLoader::loader().register_object(                                             \
          NAME1, fn_t([](const std::shared_ptr<mc_rbdyn::RobotModule> & robot, const double & dt,         \
                         const mc_control::Configuration & conf) { return new NEWCTL1; }));               \
      return true;                                                                                        \
    }();                                                                                                  \
    }

#  define SIMPLE_CONTROLLER_CONSTRUCTOR(NAME, TYPE)                                                     \
    namespace                                                                                           \
    {                                                                                                   \
    static auto registered = []()                                                                       \
    {                                                                                                   \
      using fn_t = std::function<TYPE *(const std::shared_ptr<mc_rbdyn::RobotModule> &, const double &, \
                                        const mc_control::Configuration &)>;                            \
      mc_control::ControllerLoader::loader().register_object(                                           \
          NAME, fn_t([](const std::shared_ptr<mc_rbdyn::RobotModule> & robot, const double & dt,        \
                        const mc_control::Configuration &) { return new TYPE(robot, dt); }));           \
      return true;                                                                                      \
    }();                                                                                                \
    }

#endif
