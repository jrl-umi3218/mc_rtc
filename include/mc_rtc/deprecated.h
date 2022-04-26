/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/logging.h>

/** Helper functions to emit deprecation messages when loading JSON/YAML from old versions */

namespace mc_rtc
{

namespace log
{

/** Used when \p replace should be used instead of \p old */
MC_RTC_UTILS_DLLAPI void deprecated(const std::string & source, const std::string & old, const std::string & replace);

} // namespace log

} // namespace mc_rtc
