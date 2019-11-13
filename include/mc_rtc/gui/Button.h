/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/elements.h>

namespace mc_rtc
{

namespace gui
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

/** Helper function to create a ButtonImpl */
template<typename Callback>
ButtonImpl<Callback> Button(const std::string & name, Callback cb)
{
  return ButtonImpl<Callback>(name, cb);
}

} // namespace gui

} // namespace mc_rtc
