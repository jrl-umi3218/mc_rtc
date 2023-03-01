/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc::gui
{

namespace details
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

} // namespace details

/** Helper function to create a NumberInputImpl */
template<typename GetT, typename SetT>
auto NumberInput(const std::string & name, GetT get_fn, SetT set_fn)
{
  return details::NumberInputImpl(name, get_fn, set_fn);
}

/** Helper function to create a NumberInputImpl from a variable */
template<typename T>
auto NumberInput(const std::string & name, T & value)
{
  return details::NumberInputImpl(name, details::read(value), details::write(value));
}

} // namespace mc_rtc::gui
