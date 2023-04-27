#pragma once

#include <mc_rtc/gui/Visual.h>
#include <mc_rtc/visual_utils.h>

namespace mc_rtc::gui
{

/** Parameters for building a Cylinder */
struct MC_RTC_GUI_DLLAPI CylinderParameters
{
  double radius;
  double length;
};

/** Creates a Cylinder
 *
 * \tparam GetPos Callback to get the cylinder position
 *
 * \tparam GetParams A CylinderParameters (fixed size) or callback to get the parameters
 *
 * \tparam GetColor A color (fixed color) or callback to get the color
 *
 * If \tparam GetParams and/or \tparam GetColor are callbacks they are invoked immediately
 */
template<typename GetPos, typename GetParams = const CylinderParameters &, typename GetColor = const mc_rtc::gui::Color &>
auto Cylinder(const std::string & name,
              GetParams params_fn,
              GetPos get_pos_fn,
              GetColor color_fn = mc_rtc::gui::Color::Red)
{
  CylinderParameters params = details::GetValueOrCallbackValue(params_fn);
  auto visual = mc_rtc::makeVisualCylinder(params.radius, params.length, details::GetValueOrCallbackValue(color_fn));
  auto get_visual_fn = [=]() mutable -> const rbd::parsers::Visual &
  {
    if constexpr(std::is_invocable_v<GetParams>)
    {
      auto & cylinder = mc_rtc::getVisualCylinder(visual);
      auto params = params_fn();
      cylinder.radius = params.radius;
      cylinder.length = params.length;
    }
    if constexpr(std::is_invocable_v<GetColor>) { mc_rtc::details::setVisualColor(visual, color_fn()); }
    return visual;
  };
  return Visual(name, get_visual_fn, get_pos_fn);
}

} // namespace mc_rtc::gui
