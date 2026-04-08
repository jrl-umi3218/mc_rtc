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
template<typename GetT>
struct Ordinate
{
  static constexpr Type type = Type::Ordinate;

  Ordinate(std::string_view name, GetT get_fn, Color color, Style style, Side side)
  : name_(name), get_fn_(get_fn), color_(color), style_(style), side_(side)
  {
    static_assert(details::CheckReturnType<GetT, double>::value,
                  "Ordinate callback should return a single floating-point value");
    cache_.reserve(16);
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.start_array(6);
    builder.write(static_cast<uint64_t>(type));
    builder.write(name_);
    builder.write(cache_);
    color_.write(builder);
    builder.write(static_cast<uint64_t>(style_));
    builder.write(static_cast<uint64_t>(side_));
    builder.finish_array();
    cache_.resize(0);
  }

  void update() const { cache_.push_back(get_fn_()); }

  Ordinate & style(Style style)
  {
    style_ = style;
    return *this;
  }

  Ordinate & side(Side side)
  {
    side_ = side;
    return *this;
  }

protected:
  std::string name_;
  GetT get_fn_;
  mutable Color color_;
  mutable std::vector<double> cache_;
  Style style_;
  Side side_;
};

/** Allows to provide an ordinate with changing color */
template<typename GetT, typename GetColor>
struct OrdinateWithColor : public Ordinate<GetT>
{
  OrdinateWithColor(std::string_view name, GetT get_fn, GetColor color, Style style, Side side)
  : Ordinate<GetT>(name, get_fn, color(), style, side), get_color_(color)
  {
    static_assert(details::CheckReturnType<GetColor, Color>::value, "Ordinate color callback should return a color");
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    this->color_ = get_color_();
    Ordinate<GetT>::write(builder);
  }

private:
  GetColor get_color_;
};

} // namespace impl

/** Helper to create an impl::Ordinate */
template<typename GetT>
impl::Ordinate<GetT> Y(std::string_view name,
                       GetT get_fn,
                       Color color,
                       Style style = Style::Solid,
                       Side side = Side::Left)
{
  return impl::Ordinate<GetT>(name, get_fn, color, style, side);
}

/** Helper to create an impl::OrdinateWithColor */
template<typename GetT, typename GetColor>
impl::OrdinateWithColor<GetT, GetColor> Y(std::string_view name,
                                          GetT get_fn,
                                          GetColor get_color,
                                          Style style = Style::Solid,
                                          Side side = Side::Left)
{
  return impl::OrdinateWithColor<GetT, GetColor>(name, get_fn, get_color, style, side);
}

} // namespace plot

} // namespace gui

} // namespace mc_rtc
