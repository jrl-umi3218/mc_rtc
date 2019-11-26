/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/elements.h>

namespace mc_rtc
{

namespace gui
{

/** Checkbox display a toggable checkbox
 *
 * \tparam GetT Should return a boolean providing the state of the button
 *
 * \tparam Callback When the checkbox is called, this is called
 *
 */
template<typename GetT, typename Callback>
struct CheckboxImpl : public VoidCallbackElement<DataElement<GetT>, Callback>
{
  static constexpr auto type = Elements::Checkbox;

  CheckboxImpl(const std::string & name, GetT get_fn, Callback cb)
  : VoidCallbackElement<DataElement<GetT>, Callback>(name, cb, get_fn)
  {
    static_assert(details::CheckReturnType<GetT, bool>::value,
                  "Checkbox element getter callback must return a boolean");
  }

  /** Invalid element */
  CheckboxImpl() {}
};

/** Helper function to create a Checkbox */
template<typename GetT, typename Callback>
CheckboxImpl<GetT, Callback> Checkbox(const std::string & name, GetT get_fn, Callback cb)
{
  return CheckboxImpl<GetT, Callback>(name, get_fn, cb);
}

} // namespace gui

} // namespace mc_rtc
