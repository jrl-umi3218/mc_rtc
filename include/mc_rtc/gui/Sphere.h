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
  constexpr bool has_radius_cb = std::is_invocable_v<GetRadius>;
  constexpr bool has_color_cb = std::is_invocable_v<GetColor>;
  double radius = details::GetValueOrCallbackValue(radius_fn);
  mc_rtc::gui::Color color = details::GetValueOrCallbackValue(color_fn);
  auto sphere = mc_rtc::makeVisualSphere(radius, color);
  std::function<rbd::parsers::Visual &()> get_visual_fn = [sphere]() mutable -> rbd::parsers::Visual & {
    return sphere;
  };
  if constexpr(has_radius_cb)
  {
    get_visual_fn = [get_visual_fn, radius_fn]() -> rbd::parsers::Visual & {
      auto & sphere = get_visual_fn();
      mc_rtc::getVisualSphere(sphere).radius = radius_fn();
      return sphere;
    };
  }
  if constexpr(has_color_cb)
  {
    get_visual_fn = [get_visual_fn, color_fn]() -> rbd::parsers::Visual & {
      auto & visual = get_visual_fn();
      mc_rtc::details::setVisualColor(visual, color_fn());
      return visual;
    };
  }
  return Visual(name, get_visual_fn, get_pos_fn);
}

} // namespace mc_rtc::gui
