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
template<typename GetT>
struct Ordinate
{
  static constexpr PlotType type = PlotType::Ordinate;

  Ordinate(const std::string & name, GetT get_fn, Color color, PlotStyle style, Side side, Range range)
  : name_(name), get_fn_(get_fn), color_(color), style_(style), side_(side), range_(range)
  {
    static_assert(details::CheckReturnType<GetT, double>::value,
                  "Ordinate should return a single floating-point value");
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.start_array(7);
    builder.write(static_cast<uint64_t>(PlotType::Ordinate));
    builder.write(name_);
    builder.write(get_fn_());
    color_.write(builder);
    builder.write(static_cast<uint64_t>(style_));
    builder.write(static_cast<uint64_t>(side_));
    range_.write(builder);
    builder.finish_array();
  }

private:
  std::string name_;
  GetT get_fn_;
  Color color_;
  PlotStyle style_;
  Side side_;
  Range range_;
};

} // namespace impl

/** Helper to create an impl::Ordinate */
template<typename GetT>
impl::Ordinate<GetT> Y(const std::string & name,
                       GetT get_fn,
                       Color color,
                       PlotStyle style = PlotStyle::Solid,
                       Side side = Side::Left,
                       Range range = {})
{
  return impl::Ordinate<GetT>(name, get_fn, color, style, side, range);
}

} // namespace plot

} // namespace gui

} // namespace mc_rtc
