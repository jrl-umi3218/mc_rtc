/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/deprecated.h>

namespace mc_rtc
{

namespace log
{

void deprecated(const std::string & source, const std::string & old, const std::string & replace)
{
  mc_rtc::log::warning("[MC_RTC_DEPRECATED][{}] Use of \"{}\" is deprecated, use \"{}\" instead", source, old, replace);
}

} // namespace log

} // namespace mc_rtc
