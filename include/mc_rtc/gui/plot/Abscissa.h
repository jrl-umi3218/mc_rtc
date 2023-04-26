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
    cache_.reserve(16);
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.start_array(2);
    config_.write(builder);
    builder.write(cache_);
    builder.finish_array();
    cache_.resize(0);
  }

  void update() const { cache_.push_back(get_fn_()); }

  Abscissa & range(const Range & range)
  {
    config_.range = range;
    return *this;
  }

  Abscissa & min(double min)
  {
    config_.range.min = min;
    return *this;
  }

  Abscissa & max(double max)
  {
    config_.range.max = max;
    return *this;
  }

private:
  AxisConfiguration config_;
  GetT get_fn_;
  mutable std::vector<double> cache_;
};

} // namespace impl

/** Helper to create an impl::Abscissa */
template<typename GetT>
impl::Abscissa<GetT> X(AxisConfiguration config, GetT get_fn)
{
  return impl::Abscissa<GetT>(config, get_fn);
}

/** Helper to create an impl::Abscissa without limits */
template<typename GetT>
impl::Abscissa<GetT> X(std::string_view legend, GetT get_fn)
{
  return impl::Abscissa<GetT>(AxisConfiguration(legend), get_fn);
}

} // namespace plot

} // namespace gui

} // namespace mc_rtc
