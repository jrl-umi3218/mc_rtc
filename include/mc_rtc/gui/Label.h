/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc::gui
{

namespace details
{

/** Label should display data
 *
 * \tparam GetT Can return anything that can be serialized but the element will work best with numbers or strings
 *
 * \see ArrayLabelImpl to display arrays
 */
template<typename GetT>
struct LabelImpl : public DataElement<GetT>
{
  static constexpr auto type = Elements::Label;

  LabelImpl(const std::string & name, GetT get_fn) : DataElement<GetT>(name, get_fn) {}

  /** Invalid element */
  LabelImpl() {}
};

} // namespace details

/** Helper function to create a Label element */
template<typename GetT, std::enable_if_t<std::is_invocable_v<GetT>, int> = 0>
auto Label(const std::string & name, GetT get_fn)
{
  return details::LabelImpl(name, get_fn);
}

/** Helper function to create a Label element from a variable */
template<typename T, std::enable_if_t<!std::is_invocable_v<T>, int> = 0>
auto Label(const std::string & name, T && value)
{
  return details::LabelImpl(name, details::read(std::forward<T>(value)));
}

} // namespace mc_rtc::gui
