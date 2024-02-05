/*
 * Copyright 2015-2024 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

/** This header defines a number of utility functions related to forming path in the environment
 *
 * They are thin wrapper around boost::filesystem to be later replaced by std::filesystem after Ubuntu 18.04 support is
 * fully dropped
 */

#include <mc_rtc/utils_api.h>

#include <string>

namespace mc_rtc
{

/** Returns the path to the OS temporary directory
 *
 * \param suffix Added (with a path separator) to the returned path
 */
MC_RTC_UTILS_DLLAPI std::string temp_directory_path(const std::string & suffix = "");

/** Returns the path to the user's config directory
 *
 * On Linux/macOS this returns ${HOME}/.config/mc_rtc folder
 *
 * On Windows this returns the %APPDATA%/mc_rtc folder
 *
 * \param suffix Added (with a path separator) to the returned path
 */
MC_RTC_UTILS_DLLAPI std::string user_config_directory_path(const std::string & suffix = "");

} // namespace mc_rtc
