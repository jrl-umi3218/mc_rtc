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

/** For a given plot, this holds options on the abscissa axis and a
 * callback that returns the current abscissa value */
template<typename GetT>
struct Abscissa
{
  static constexpr Type type = Type::Abscissa;

  Abscissa(AxisConfiguration config, GetT get_fn) : config_(config), get_fn_(get_fn)
  {
    static_assert(details::CheckReturnType<GetT, double>::value,
                  "Abscissa should return a single floating-point value");
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.start_array(2);
    config_.write(builder);
    builder.write(get_fn_());
    builder.finish_array();
  }

private:
  AxisConfiguration config_;
  GetT get_fn_;
};

} // namespace impl

/** Helper to create an impl::Abscissa */
template<typename GetT>
impl::Abscissa<GetT> X(AxisConfiguration config, GetT get_fn)
{
  return impl::Abscissa<GetT>(config, get_fn);
}

} // namespace plot

} // namespace gui

} // namespace mc_rtc
