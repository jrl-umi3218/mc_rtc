/*
 * Copyright 2015-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/utils_api.h>

#include <string>
#include <typeinfo>

namespace mc_rtc
{

namespace internal
{

MC_RTC_UTILS_DLLAPI std::string demangle(const char * name);

} // namespace internal

/** Return a human-readable type name
 *
 * Only use for displaying messages as the name is not guaranteed to be the
 * same across compilers and invokation
 */
template<typename T>
std::string type_name()
{
  return internal::demangle(typeid(T).name());
}

} // namespace mc_rtc
