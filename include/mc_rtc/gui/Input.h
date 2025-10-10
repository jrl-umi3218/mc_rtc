/*
 * Copyright 2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/ArrayInput.h>
#include <mc_rtc/gui/Checkbox.h>
#include <mc_rtc/gui/IntegerInput.h>
#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/StringInput.h>

namespace mc_rtc::gui
{

/** Generic helper to build an input from a variable, the most appopriate type is automatically selected */
template<typename T>
auto Input(const std::string & name, T & value)
{
  if constexpr(std::is_same_v<T, bool>) { return Checkbox(name, value); }
  else if constexpr(std::is_same_v<T, std::string>) { return StringInput(name, value); }
  else if constexpr(std::is_floating_point_v<T>) { return NumberInput(name, value); }
  else if constexpr(mc_rtc::internal::is_integral_v<T>) { return IntegerInput(name, value); }
  else
  {
    return ArrayInput(name, value);
  }
}

} // namespace mc_rtc::gui
