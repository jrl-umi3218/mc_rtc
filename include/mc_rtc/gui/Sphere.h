#pragma once

#include <mc_rtc/gui/Visual.h>
#include <mc_rtc/visual_utils.h>

namespace mc_rtc::gui
{

/** Creates a Sphere
 *
 * \tparam GetPos Callback to get the sphere position
 *
 * \tparam GetRadius A double (fixed radius) or callback to get the radius
 *
 * \tparam GetColor A color (fixed color) or callback to get the color
 *
 * If \tparam GetRadius and/or \tparam GetColor are callbacks they are invoked immediately
 */
template<typename GetPos, typename GetRadius = double, typename GetColor = const mc_rtc::gui::Color &>
auto Sphere(const std::string & name,
            GetRadius radius_fn,
            GetPos get_pos_fn,
            GetColor color_fn = mc_rtc::gui::Color::Red)
{
  auto sphere =
      mc_rtc::makeVisualSphere(details::GetValueOrCallbackValue(radius_fn), details::GetValueOrCallbackValue(color_fn));
  auto get_visual_fn = [=]() mutable -> const rbd::parsers::Visual &
  {
    if constexpr(std::is_invocable_v<GetRadius>) { mc_rtc::getVisualSphere(sphere).radius = radius_fn(); }
    if constexpr(std::is_invocable_v<GetColor>) { mc_rtc::details::setVisualColor(sphere, color_fn()); }
    return sphere;
  };
  return Visual(name, get_visual_fn, get_pos_fn);
}

} // namespace mc_rtc::gui
