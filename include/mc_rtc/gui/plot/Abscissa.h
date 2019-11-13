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

/** For a given plot, this holds options on the abscissa axis and a
 * callback that returns the current abscissa value */
template<typename GetT>
struct Abscissa
{
  static constexpr PlotType type = PlotType::Ordinate;

  Abscissa(const std::string & name, GetT get_fn, Range range) : name_(name), get_fn_(get_fn), range_(range)
  {
    static_assert(details::CheckReturnType<GetT, double>::value,
                  "Abscissa should return a single floating-point value");
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.start_array(3);
    builder.write(name_);
    builder.write(get_fn_());
    range_.write(builder);
    builder.finish_array();
  }

private:
  std::string name_;
  GetT get_fn_;
  Range range_;
};

} // namespace impl

/** Helper to create an impl::Abscissa */
template<typename GetT>
impl::Abscissa<GetT> X(const std::string & name, GetT get_fn, Range range = {})
{
  return impl::Abscissa<GetT>(name, get_fn, range);
}

} // namespace plot

} // namespace gui

} // namespace mc_rtc
