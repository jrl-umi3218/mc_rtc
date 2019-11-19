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

/** This hold information on (x,y) points in a polygon and how to plot them */
template<typename GetT>
struct Polygon
{
  static constexpr Type type = Type::Polygon;

  Polygon(const std::string & name, GetT get_fn, Side side) : name_(name), get_fn_(get_fn), side_(side)
  {
    static_assert(details::CheckReturnType<GetT, PolygonDescription>::value,
                  "Polygon element callback must return a PolygonDescription");
  }

  void write(mc_rtc::MessagePackBuilder & builder) const
  {
    builder.start_array(4);
    builder.write(static_cast<uint64_t>(type));
    builder.write(name_);
    get_fn_().write(builder);
    builder.write(static_cast<uint64_t>(side_));
    builder.finish_array();
  }

  Polygon & side(Side side)
  {
    side_ = side;
    return *this;
  }

private:
  std::string name_;
  GetT get_fn_;
  Side side_;
};

} // namespace impl

/** Helper function to create an impl::Polygon */
template<typename GetT>
impl::Polygon<GetT> Polygon(const std::string & name, GetT get_fn, Side side = Side::Left)
{
  return impl::Polygon<GetT>(name, get_fn, side);
}

} // namespace plot

} // namespace gui

} // namespace mc_rtc
