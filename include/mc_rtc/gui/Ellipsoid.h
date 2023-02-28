#pragma once

#include <mc_rtc/gui/Visual.h>
#include <mc_rtc/visual_utils.h>

namespace mc_rtc::gui
{

/** Creates an Ellipsoid
 *
 * \tparam GetPos Callback to get the ellipsoid position
 *
 * \tparam GetSize An Eigen::Vector3d (fixed size) or callback to get the ellipsoid size
 *
 * \tparam GetColor A color (fixed color) or callback to get the color
 *
 * If \tparam GetSize and/or \tparam GetColor are callbacks they are invoked immediately
 */
template<typename GetPos, typename GetSize = const Eigen::Vector3d &, typename GetColor = const mc_rtc::gui::Color &>
auto Ellipsoid(const std::string & name,
               GetSize size_fn,
               GetPos get_pos_fn,
               GetColor color_fn = mc_rtc::gui::Color::Red)
{
  auto ellipsoid = mc_rtc::makeVisualSuperellispoid(details::GetValueOrCallbackValue(size_fn), 1.0, 1.0,
                                                    details::GetValueOrCallbackValue(color_fn));
  auto get_visual_fn = [=]() mutable -> const rbd::parsers::Visual & {
    if constexpr(std::is_invocable_v<GetSize>)
    {
      mc_rtc::getVisualSuperellipsoid(ellipsoid).size = size_fn();
    }
    if constexpr(std::is_invocable_v<GetColor>)
    {
      mc_rtc::details::setVisualColor(ellipsoid, color_fn());
    }
    return ellipsoid;
  };
  return Visual(name, get_visual_fn, get_pos_fn);
}

} // namespace mc_rtc::gui
