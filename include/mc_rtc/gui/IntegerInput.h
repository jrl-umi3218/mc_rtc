/*
 * Copyright 2015-2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc::gui
{

namespace details
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

} // namespace details

/** Helper function to create a IntegerInputImpl */
template<typename GetT, typename SetT>
auto IntegerInput(const std::string & name, GetT get_fn, SetT set_fn)
{
  return details::IntegerInputImpl(name, get_fn, set_fn);
}

/** Helper function to create an IntegerInputImpl from a reference */
template<typename T>
auto IntegerInput(const std::string & name, T & value)
{
  return details::IntegerInputImpl(name, details::read(value), details::write(value));
}

} // namespace mc_rtc::gui
