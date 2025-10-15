/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rtc/deprecated.h>

namespace mc_rtc
{

namespace log
{

void deprecated(const std::string & source,
                const std::string & old,
                const std::string & replace,
                const std::string & details)
{
  if(details.empty())
  {
    mc_rtc::log::warning("[MC_RTC_DEPRECATED][{}] Use of \"{}\" is deprecated, use \"{}\" instead", source, old,
                         replace);
  }
  else
  {
    mc_rtc::log::warning("[MC_RTC_DEPRECATED][{}] Use of \"{}\" is deprecated, use \"{}\" instead: {}", source, old,
                         replace, details);
  }
}

} // namespace log

} // namespace mc_rtc
