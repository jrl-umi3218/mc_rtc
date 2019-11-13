/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc
{

namespace gui
{

/** This element should implement a dialog to input an integer */
template<typename GetT, typename SetT>
struct IntegerInputImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::IntegerInput;

  IntegerInputImpl(const std::string & name, GetT get_fn, SetT set_fn)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn)
  {
  }

  /** Invlaid element */
  IntegerInputImpl() {}
};

/** Helper function to create a IntegerInputImpl */
template<typename GetT, typename SetT>
IntegerInputImpl<GetT, SetT> IntegerInput(const std::string & name, GetT get_fn, SetT set_fn)
{
  return IntegerInputImpl<GetT, SetT>(name, get_fn, set_fn);
}

} // namespace gui

} // namespace mc_rtc
