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
template<typename UpdateCacheT>
struct AbscissaOrdinate
{
  static constexpr Type type = Type::AbscissaOrdinate;

  using CacheT = std::vector<std::array<double, 2>>;

  AbscissaOrdinate(std::string_view name, UpdateCacheT update_fn, Color color, Style style, Side side)
  : name_(name), update_fn_(update_fn), color_(color), style_(style), side_(side)
  {
    static_assert(details::has_compatible_signature_v<UpdateCacheT, void(CacheT &)>,
                  "AbscissaOrdinate callback should update its cache");
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

  void update() const { update_fn_(cache_); }

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
  UpdateCacheT update_fn_;
  mutable Color color_;
  mutable std::vector<std::array<double, 2>> cache_;
  Style style_;
  Side side_;
};

/** Allows to provide an ordinate with changing color */
template<typename UpdateCacheT, typename GetColor>
struct AbscissaOrdinateWithColor : public AbscissaOrdinate<UpdateCacheT>
{
  AbscissaOrdinateWithColor(std::string_view name, UpdateCacheT update_cache, GetColor color, Style style, Side side)
  : AbscissaOrdinate<UpdateCacheT>(name, update_cache, color(), style, side), get_color_(color)
  {
    static_assert(details::CheckReturnType<GetColor, Color>::value,
                  "AbscissaOrdinate color callback should return a color");
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    this->color_ = get_color_();
    AbscissaOrdinate<UpdateCacheT>::write(builder);
  }

private:
  GetColor get_color_;
};
} // namespace impl

/** Helper to create an impl::AbscissaOrdinate|impl::AbscissaOrdinateWithColor with chunky updates
 *
 * This is meant to be used to create/update graphs with a bunch of data at once
 */
template<typename UpdateCacheT, typename MaybeGetColor>
auto XYChunk(std::string_view name,
             UpdateCacheT update_fn,
             MaybeGetColor color,
             Style style = Style::Solid,
             Side side = Side::Left)
{
  if constexpr(std::is_same_v<std::decay_t<MaybeGetColor>, Color>)
  {
    return impl::AbscissaOrdinate(name, update_fn, color, style, side);
  }
  else { return impl::AbscissaOrdinateWithColor(name, update_fn, color, style, side); }
}

/** Helper to create an impl::AbscissaOrdinate|impl::AbscissaOrdinateWithColor */
template<typename GetXT, typename GetYT, typename MaybeGetColor>
auto XY(std::string_view name,
        GetXT get_x_fn,
        GetYT get_y_fn,
        MaybeGetColor color,
        Style style = Style::Solid,
        Side side = Side::Left)
{
  static_assert(details::CheckReturnType<GetXT, double>::value,
                "XY x-callback should return a single floating-point value");
  static_assert(details::CheckReturnType<GetYT, double>::value,
                "XY y-callback should return a single floating-point value");
  using XYCacheT = std::vector<std::array<double, 2>>;
  if constexpr(std::is_same_v<std::decay_t<MaybeGetColor>, Color>)
  {
    return impl::AbscissaOrdinate(
        name, [get_x_fn, get_y_fn](XYCacheT & cache) { cache.push_back({get_x_fn(), get_y_fn()}); }, color, style,
        side);
  }
  else
  {
    static_assert(details::CheckReturnType<MaybeGetColor, Color>::value, "XY color callback should return a color");
    return impl::AbscissaOrdinateWithColor(
        name, [get_x_fn, get_y_fn](XYCacheT & cache) { cache.push_back({get_x_fn(), get_y_fn()}); }, color, style,
        side);
  }
}

} // namespace plot

} // namespace gui

} // namespace mc_rtc
