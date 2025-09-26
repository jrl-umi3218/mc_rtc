/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/logging.h>
#include <mc_rtc/macros/deprecated.h>
#include "mc_rtc/utils_api.h"

/** Helper functions to emit deprecation messages when loading JSON/YAML from old versions */

namespace mc_rtc
{

namespace log
{

/** Used when \p replace should be used instead of \p old */
MC_RTC_UTILS_DLLAPI void deprecated(const std::string & source,
                                    const std::string & old,
                                    const std::string & replace,
                                    const std::string & details = "");

} // namespace log

} // namespace mc_rtc
