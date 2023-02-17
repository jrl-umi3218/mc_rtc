/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc::gui
{

namespace details
{

/** Button should provide a clickable button
 *
 * When the button is clicked, Callback() is called
 *
 * \tparam Callback The callback that will be called when clicking the button
 *
 */
template<typename Callback>
struct ButtonImpl : public VoidCallbackElement<Element, Callback>
{
  static constexpr auto type = Elements::Button;

  template<typename... Args>
  ButtonImpl(const std::string & name, Callback cb, Args &&... args)
  : VoidCallbackElement<Element, Callback>(name, cb, std::forward<Args>(args)...)
  {
  }

  /** Invalid element */
  ButtonImpl() {}
};

} // namespace details

/** Helper function to create a ButtonImpl */
template<typename Callback>
auto Button(const std::string & name, Callback cb)
{
  return details::ButtonImpl(name, cb);
}

} // namespace mc_rtc::gui
