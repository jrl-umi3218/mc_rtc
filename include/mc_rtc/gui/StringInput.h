/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc
{

namespace gui
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

/** Helper function to create a StringInputImpl */
template<typename GetT, typename SetT>
StringInputImpl<GetT, SetT> StringInput(const std::string & name, GetT get_fn, SetT set_fn)
{
  return StringInputImpl<GetT, SetT>(name, get_fn, set_fn);
}

} // namespace gui

} // namespace mc_rtc
