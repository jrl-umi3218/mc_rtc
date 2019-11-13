/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc
{

namespace gui
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

/** Helper function to create a Label element */
template<typename GetT>
LabelImpl<GetT> Label(const std::string & name, GetT get_fn)
{
  return LabelImpl<GetT>(name, get_fn);
}

} // namespace gui

} // namespace mc_rtc
