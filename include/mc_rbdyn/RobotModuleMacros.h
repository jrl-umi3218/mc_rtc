/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include <mc_rbdyn/RobotModule.h>

#include <mc_rtc/version.h>

/* Set of macros to assist with the writing of a RobotModule */

/** A simple compile-time versus run-time version checking macro
 *
 * If you are not relying on ROBOT_MODULE_DEFAULT_CONSTRUCTOR or
 * ROBOT_MODULE_CANONIC_CONSTRUCTOR you should use this in your
 * MC_RTC_ROBOT_MODULE implementation
 *
 */
#define ROBOT_MODULE_CHECK_VERSION(NAME)                                                                             \
  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())                                                                    \
  {                                                                                                                  \
    mc_rtc::log::error("{} was compiled with {} but mc_rtc is currently at version {}, you might experience subtle " \
                       "issues and should recompile your code",                                                      \
                       NAME, mc_rtc::MC_RTC_VERSION, mc_rtc::version());                                             \
  }

/*! ROBOT_MODULE_COMMON
 * Declare a destroy symbol and CLASS_NAME symbol
 * Constructor should be declared by the user
 */
#define ROBOT_MODULE_COMMON(NAME)                                             \
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names) \
  {                                                                           \
    ROBOT_MODULE_CHECK_VERSION(NAME)                                          \
    names = {NAME};                                                           \
  }                                                                           \
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)                  \
  {                                                                           \
    delete ptr;                                                               \
  }

/*! ROBOT_MODULE_DEFAULT_CONSTRUCTOR
 * Declare an external symbol for creation using a default constructor
 * Also declare destruction symbol
 * Exclusive of ROBOT_MODULE_CANONIC_CONSTRUCTOR
 */
#define ROBOT_MODULE_DEFAULT_CONSTRUCTOR(NAME, TYPE)                     \
  extern "C"                                                             \
  {                                                                      \
    ROBOT_MODULE_COMMON(NAME)                                            \
    ROBOT_MODULE_API unsigned int create_args_required()                 \
    {                                                                    \
      return 1;                                                          \
    }                                                                    \
    ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string &) \
    {                                                                    \
      return new TYPE();                                                 \
    }                                                                    \
  }

/*! ROBOT_MODULE_CANONIC_CONSTRUCTOR
 * Declare an external symbol for creation using the cannonical constructor (const string &, const
 * string &) Also declare destruction symbol Exclusive of ROBOT_MODULE_DEFAULT_CONSTRUCTOR
 */
#define ROBOT_MODULE_CANONIC_CONSTRUCTOR(NAME, TYPE)                          \
  extern "C"                                                                  \
  {                                                                           \
    ROBOT_MODULE_COMMON(NAME)                                                 \
    ROBOT_MODULE_API unsigned int create_args_required()                      \
    {                                                                         \
      return 3;                                                               \
    }                                                                         \
    ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string &,      \
                                                    const std::string & path, \
                                                    const std::string & name) \
    {                                                                         \
      return new TYPE(path, name);                                            \
    }                                                                         \
  }
