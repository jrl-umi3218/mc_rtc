/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/details/traits.h>
#include <mc_rtc/gui/plot/types.h>

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

  AbscissaOrdinate & style(Style style)
  {
    style_ = style;
    return *this;
  }

  AbscissaOrdinate & side(Side side)
  {
    side_ = side;
    return *this;
  }

protected:
  std::string name_;
  GetXT get_x_fn_;
  GetYT get_y_fn_;
  mutable Color color_;
  Style style_;
  Side side_;
};

/** Allows to provide an ordinate with changing color */
template<typename GetXT, typename GetYT, typename GetColor>
struct AbscissaOrdinateWithColor : public AbscissaOrdinate<GetXT, GetYT>
{
  AbscissaOrdinateWithColor(const std::string & name, GetXT get_x, GetYT get_y, GetColor color, Style style, Side side)
  : AbscissaOrdinate<GetXT, GetYT>(name, get_x, get_y, color(), style, side), get_color_(color)
  {
    static_assert(details::CheckReturnType<GetColor, Color>::value,
                  "AbscissaOrdinate color callback should return a color");
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    this->color_ = get_color_();
    AbscissaOrdinate<GetXT, GetYT>::write(builder);
  }

private:
  GetColor get_color_;
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

/** Helper to create an impl::OrdinateWithColor */
template<typename GetXT, typename GetYT, typename GetColor>
impl::AbscissaOrdinateWithColor<GetXT, GetYT, GetColor> XY(const std::string & name,
                                                           GetXT get_x_fn,
                                                           GetYT get_y_fn,
                                                           GetColor get_color_fn,
                                                           Style style = Style::Solid,
                                                           Side side = Side::Left)
{
  return impl::AbscissaOrdinateWithColor<GetXT, GetYT, GetColor>(name, get_x_fn, get_y_fn, get_color_fn, style, side);
}

} // namespace plot

} // namespace gui

} // namespace mc_rtc
