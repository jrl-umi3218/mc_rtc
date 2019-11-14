/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/plot/options.h>

namespace mc_rtc
{

namespace gui
{

namespace plot
{

namespace impl
{

/** For a given plot, this holds options on the ordinate plot and a callback
 * that returns the current value */
template<typename GetXT, typename GetYT>
struct AbscissaOrdinate
{
  static constexpr Type type = Type::AbscissaOrdinate;

  AbscissaOrdinate(const std::string & name, GetXT get_x_fn, GetYT get_y_fn, Color color, Style style, Side side)
  : name_(name), get_x_fn_(get_x_fn), get_y_fn_(get_y_fn), color_(color), style_(style), side_(side)
  {
    static_assert(details::CheckReturnType<GetXT, double>::value,
                  "AbscissaOrdinate x-callback should return a single floating-point value");
    static_assert(details::CheckReturnType<GetYT, double>::value,
                  "AbscissaOrdinate y-callback should return a single floating-point value");
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.start_array(7);
    builder.write(static_cast<uint64_t>(type));
    builder.write(name_);
    builder.write(get_x_fn_());
    builder.write(get_y_fn_());
    color_.write(builder);
    builder.write(static_cast<uint64_t>(style_));
    builder.write(static_cast<uint64_t>(side_));
    builder.finish_array();
  }

private:
  std::string name_;
  GetXT get_x_fn_;
  GetYT get_y_fn_;
  Color color_;
  Style style_;
  Side side_;
};

} // namespace impl

/** Helper to create an impl::Ordinate */
template<typename GetXT, typename GetYT>
impl::AbscissaOrdinate<GetXT, GetYT> XY(const std::string & name,
                                        GetXT get_x_fn,
                                        GetYT get_y_fn,
                                        Color color,
                                        Style style = Style::Solid,
                                        Side side = Side::Left)
{
  return impl::AbscissaOrdinate<GetXT, GetYT>(name, get_x_fn, get_y_fn, color, style, side);
}

} // namespace plot

} // namespace gui

} // namespace mc_rtc
