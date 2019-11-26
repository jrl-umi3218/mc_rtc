/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc
{

namespace gui
{

/** This element should implement a dialog to input a number */
template<typename GetT, typename SetT>
struct NumberInputImpl : public CommonInputImpl<GetT, SetT>
{
  static constexpr auto type = Elements::NumberInput;

  NumberInputImpl(const std::string & name, GetT get_fn, SetT set_fn)
  : CommonInputImpl<GetT, SetT>(name, get_fn, set_fn)
  {
  }

  /** Invlaid element */
  NumberInputImpl() {}
};

/** Helper function to create a NumberInputImpl */
template<typename GetT, typename SetT>
NumberInputImpl<GetT, SetT> NumberInput(const std::string & name, GetT get_fn, SetT set_fn)
{
  return NumberInputImpl<GetT, SetT>(name, get_fn, set_fn);
}
} // namespace gui

} // namespace mc_rtc
