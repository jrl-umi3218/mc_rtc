/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc::gui
{

namespace details
{

/** This element should implement a dialog to input a string */
template<typename GetT, typename SetT>
struct StringInputImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::StringInput;

  StringInputImpl(const std::string & name, GetT get_fn, SetT set_fn)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn)
  {
  }

  /** Invlaid element */
  StringInputImpl() {}
};

} // namespace details

/** Helper function to create a StringInputImpl */
template<typename GetT, typename SetT>
auto StringInput(const std::string & name, GetT get_fn, SetT set_fn)
{
  return details::StringInputImpl(name, get_fn, set_fn);
}

/** Helper function to create a StringInputImpl from a variable */
inline auto StringInput(const std::string & name, std::string & value)
{
  return details::StringInputImpl(name, details::read(value), details::write(value));
}

} // namespace mc_rtc::gui
